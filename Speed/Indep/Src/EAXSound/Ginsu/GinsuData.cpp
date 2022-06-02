#include "EAXSound/Ginsu/GinsuData.h"
#include "EAXSound/Ginsu/Ginsu.h"


// <@>PRINT_ASM
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