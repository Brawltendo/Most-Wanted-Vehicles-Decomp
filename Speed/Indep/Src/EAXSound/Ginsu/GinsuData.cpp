#include "EAXSound/Ginsu/GinsuData.h"
#include "EAXSound/Ginsu/Ginsu.h"
#include "EAXSound/Ginsu/GinsuHelper.h"


// FUNCTIONAL MATCH
// the loop in this function is very stubborn to byte match since MSVC unrolls it (and it doesn't wanna unroll it the same way)
// all the code produces the same result as the original game though
void GinsuSynthData::DecodeBlock(int block)
{
	// Each block of XAS v0 encoded data is 19 bytes long, and gets decoded to 32 floating-point samples
	// Samples are output in the -32768/+32768 range

	mCurrentBlock = block;
	uint8_t* src = &mSampleData[block * BLOCKSIZE_BYTES];
	
	// decode first 2 PCM samples, uses first 4 bytes of the current block
	// these samples are then used to decode the rest of the block
	mSample[0] = static_cast<float>((src[0] & 0xf0) + (static_cast<int8_t>(src[1]) << 8));
	mSample[1] = static_cast<float>((src[2] & 0xf0) + (static_cast<int8_t>(src[3]) << 8));
	float* out = &mSample[2];

	int filt = src[0] & 0x0f;
	float f0 = xafilterf[0][filt];
	float f1 = xafilterf[1][filt];
	int shift = src[2] & 0x0f;
	src += 4; // already ran through 4 bytes of the block, so increment it

	// decode the remaining 30 samples for this block
	for (int i = 0; i < 15; ++i)
	{
		int s0 = src[i] >> 4;
		int s1 = src[i] & 0x0f;
		
		// each sample is decoded using the 2 samples before it
		out[0] = (xatablef[shift][s0] + (f1 * out[-2])) + (f0 * out[-1]);
		out[1] = (xatablef[shift][s1] + (f1 * out[-1])) + (f0 * out[+0]);
		out += 2;
	}
}

// MATCHING
GinsuSynthData::GinsuSynthData()
{
	mMinFrequency = 0.f;
	mMaxFrequency = 0.f;
	mSegCount = 0;
	mCycleCount = 0;
	mSampleCount = 0;
	mSampleRate = 0;
}

// MATCHING
bool GinsuSynthData::BindToData(void* ptr)
{
	GinsuDataLayout* memdata = static_cast<GinsuDataLayout*>(ptr);

	mMinFrequency = 0.f;
	mMaxFrequency = 0.f;
	mSegCount = 0;
	mCycleCount = 0;
	mSampleCount = 0;
	mSampleRate = 0;

	// validate Ginsu header before binding data
	if (memdata->id[0]  == GINSU_ID[0]   // G
	&&  memdata->id[1]  == GINSU_ID[1]   // n
	&&  memdata->id[2]  == GINSU_ID[2]   // s
	&&  memdata->id[3]  == GINSU_ID[3]   // u
	&&  memdata->ver[0] == GINSU_VER[0]) // 2
	{
		mMinFrequency = memdata->minFrequency;
		mMaxFrequency = memdata->maxFrequency;
		mSegCount = memdata->segCount;
		mCycleCount = memdata->cycleCount;
		mSampleCount = memdata->sampleCount;
		mSampleRate = memdata->sampleRate;
		mFreqPos = reinterpret_cast<int*>(memdata->data);
		mCyclePos = &mFreqPos[mSegCount + 1];
		mSampleData = reinterpret_cast<uint8_t*>(&mCyclePos[mCycleCount + 1]);
		mCurrentBlock = -1;
		int minperiod = mSampleCount;

		for (int i = 0; i < mCycleCount; i++)
		{
			if (mCyclePos[i + 1] - mCyclePos[i] < minperiod)
				minperiod = mCyclePos[i + 1] - mCyclePos[i];
		}
		mMinPeriod = minperiod;
		// data binding succeeded
		return true;
	}
	else
	{
		// data binding failed
		return false;
	}
}

// MATCHING
int GinsuSynthData::FrequencyToSample(float freq)
{
	int samp;
	if (mSegCount < 1)
	{
		samp = 0;
	}
	else if (freq <= mMinFrequency)
	{	
		samp = mFreqPos[0];
	}
	else if (freq >= mMaxFrequency)
	{
		samp = mFreqPos[mSegCount];
	}
	else
	{
		float seg = (freq - mMinFrequency) * mSegCount / (mMaxFrequency - mMinFrequency);
		int i = IntFloor(seg);
		float a = mFreqPos[i+1] - mFreqPos[i];
		samp = IntRound(a * (seg - i) + mFreqPos[i]);
	}
	return samp;
}

// MATCHING
int GinsuSynthData::CycleToSample(float cycle)
{
	int samp;
	if (mCycleCount < 1)
	{
		samp = 0;
	}
	else if (cycle <= 0.f)
	{	
		samp = mCyclePos[0];
	}
	else if (mCycleCount <= cycle)
	{
		samp = mCyclePos[mCycleCount];
	}
	else
	{
		int i = IntFloor(cycle);
		float a = mCyclePos[i+1] - mCyclePos[i];
		samp = IntRound(a * (cycle - i) + mCyclePos[i]);
	}
	return samp;
}

// MATCHING
float GinsuSynthData::CyclePeriod(float cycle)
{
	// can't have less than one cycle
	if (mCycleCount < 1)
	{
    	return 0.f;
	}
	else
	{
		float startPeriod;
		float endPeriod;
		int i = IntFloor(cycle);
		if (i < 1)
		{
			if (i < 0)
			{
				cycle = 0.f;
				i = 0;
			}
			startPeriod = (mCyclePos[1] - mCyclePos[0]);
			endPeriod   = (mCyclePos[2] - mCyclePos[0]) * 0.5f;
		}
		else
		{
			if (i >= mCycleCount - 1)
			{
				if (i >= mCycleCount)
				{
					i = mCycleCount - 1;
					cycle = mCycleCount;
				}
				startPeriod = (mCyclePos[mCycleCount] - mCyclePos[mCycleCount-2]) * 0.5f;
				endPeriod   = (mCyclePos[mCycleCount] - mCyclePos[mCycleCount-1]);
			}
			else
			{
				startPeriod = (mCyclePos[i+1] - mCyclePos[i-1]) * 0.5f;
				endPeriod   = (mCyclePos[i+2] - mCyclePos[i]) * 0.5f;
			}
		}
		return startPeriod + (cycle - i) * (endPeriod - startPeriod);
	}
}

// MATCHING
float GinsuSynthData::SampleToCycle(int sample)
{
	
	if (mCycleCount < 1)
		return 0.f;
	if (sample <= mCyclePos[0])
		return 0.f;
	if (sample >= mCyclePos[mCycleCount])
		return mCycleCount;

	int low = 0;
	int high = mCycleCount;
	int guess = 0;
	int* currCycleSamp = NULL;
	for (;;)
	{
		int cycle = 0;
		for (;;)
		{
			float s1 = static_cast<float>(sample - mCyclePos[low]) 
					 / static_cast<float>(mCyclePos[high] - mCyclePos[low]) 
					 * static_cast<float>(high - low);
			guess = IntFloor(s1) + low;
			currCycleSamp = &mCyclePos[guess];
			cycle = currCycleSamp[0];
			if (sample >= currCycleSamp[0])
				break;
			
			float s2 = static_cast<float>(cycle - sample) / mMinPeriod;
			int newLow = guess - IntCeil(s2);
			if (newLow > low)
				low = newLow;
			high = guess;
		}

		if (sample < currCycleSamp[1])
			break;
		
		low = guess + 1;
		int newHigh = guess + IntCeil(static_cast<float>(sample - cycle) / mMinPeriod) + 1;
		if (newHigh < high)
			high = newHigh;
	}
	return (static_cast<float>(sample) - mCyclePos[guess]) / (static_cast<float>(mCyclePos[guess+1]) - mCyclePos[guess]) + guess;
}

// MATCHING
// <@>PRINT_ASM
bool GinsuSynthData::GetSamples(int startSample, int numSamples, short* dest)
{
	if (startSample < 0 || startSample + numSamples - 1 >= mSampleCount)
		return false;
	if (startSample >> 5 != mCurrentBlock)
		GinsuSynthData::DecodeBlock(startSample >> 5);
	int index = startSample & 0x1F;
	for (int i = 0; i < numSamples; ++i)
	{
		int endSample = convertsample(mSample[index]);
		++index;
		dest[i] = endSample;
		if (index == BLOCKSIZE_SAMPLES)
		{
			DecodeBlock(mCurrentBlock + 1);
			index = 0;
		}
	}
	return true;
}