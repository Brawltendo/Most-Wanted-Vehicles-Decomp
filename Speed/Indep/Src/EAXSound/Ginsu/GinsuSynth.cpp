#include "EAXSound/Ginsu/GinsuSynth.h"
#include "EAXSound/Ginsu/GinsuData.h"
#include "EAXSound/Ginsu/GinsuHelper.h"

// SND
extern "C" {
#include "packages/snd/9/source/library/cmn/ssys.h"
#include "packages/snd/9/source/library/cmn/spktplay.h"
#include "packages/snd/9/source/library/cmn/sattrdef.h"
#include "packages/snd/9/source/library/cmn/splysdef.h"
}

#include "math/bmath.h"


void GinsuSynthesis::PacketReleaseCallback(void* samples, void* clientdata)
{
	GinsuSynthesis* object = reinterpret_cast<GinsuSynthesis*>(clientdata);
	object->HandlePacketRelease(reinterpret_cast<short*>(samples));
}

// <@>PRINT_ASM
void GinsuSynthesis::HandlePacketRelease(short* samples)
{
	if (mSynthData)
		mCurrentCycle = mSynthData->SampleToCycle(mCurrentPos);

	int jumpDist = 0;
	int sampleCount = 0;
	float jumpTime;
	float change = static_cast<float>(mTargetPos - mCurrentPos) / mPacketCountdown;
	if (mNoJumpRemaining < mPacketSize )
	{
		if (mSynthData)
		{
			float playbackCycle = mSynthData->SampleToCycle(mPlaybackPos);
			float playbackRate = mPacketSize / mSynthData->CyclePeriod(playbackCycle);
			float currentRate = change / mSynthData->CyclePeriod(mCurrentCycle);
			float posDiff = mCurrentCycle - playbackCycle;
			float rateDiff = currentRate - playbackRate;
			float nojumpTime = static_cast<float>(mNoJumpRemaining) / static_cast<float>(mPacketSize);
			float nojumpDist = nojumpTime * rateDiff + posDiff;
			float packetDist = rateDiff + posDiff;
			if (IntFloor(nojumpDist) != IntFloor(packetDist))
			{
				jumpDist = nojumpDist < packetDist ? IntCeil(nojumpDist) : IntFloor(nojumpDist);
				jumpTime = (jumpDist - posDiff) / rateDiff;

				if (!jumpDist)
					jumpTime = 1.f;
			}
			else if (!(bAbs(nojumpDist) > IntCeil(bAbs(rateDiff))))
			{
				jumpTime = 1.f;
			}
			else
			{
				jumpDist = nojumpDist < 0.f ? IntCeil(nojumpDist) : IntFloor(nojumpDist);
				jumpTime = nojumpTime;

				if (!jumpDist)
					jumpTime = 1.f;
			}			
		}
		else
		{
			jumpTime = 1.f;
		}
	}
	else
	{
		jumpTime = 1.f;
	}

	sampleCount = IntRound(mPacketSize * jumpTime);
	if (mSynthData)
		mSynthData->GetSamples(mPlaybackPos, sampleCount, samples);

	mPlaybackPos += sampleCount;
	mNoJumpRemaining -= sampleCount;
	if (mNoJumpRemaining < 0)
		mNoJumpRemaining = 0;

	if (jumpDist)
	{
		short* dest;
		short buff[256];
		if (mSynthData)
		{
			dest = &samples[sampleCount];
			mSynthData->GetSamples(mPlaybackPos, mOverlapSize, dest);
			float cycle = mSynthData->SampleToCycle(mPlaybackPos);
			mPlaybackPos = mSynthData->CycleToSample(cycle + jumpDist);
			mSynthData->GetSamples(mPlaybackPos, mOverlapSize, buff);
		}

		float blend = 0.f;
		float blendstep = 1.f / mOverlapSize;
		for (int i = 0; i < mOverlapSize; ++i)
		{
			float val = (buff[i] - dest[i]) * blend + dest[i];
			dest[i] = IntRound(val);
			blend += blendstep;
		}

		sampleCount += mOverlapSize;
		mPlaybackPos += mOverlapSize;
		mNoJumpRemaining = mNoJumpSize - mOverlapSize;
		int count = (mPacketSize - sampleCount);
		if (count > 0)
		{
			if (mSynthData)
				mSynthData->GetSamples(mPlaybackPos, count, &samples[sampleCount]);
			sampleCount = mPacketSize;
			mPlaybackPos += count;
			mNoJumpRemaining -= count;
		}
	}

	mCurrentPos = IntRound(mCurrentPos + change);
	if (mPacketCountdown > 1)
    	--mPacketCountdown;

	SNDPACKET sp;
	sp.numframes = sampleCount;
	sp.psamples[0] = samples;
	SNDSYS_entercritical();
	SNDPKTPLAY_submit(mPacketHandle, &sp);
	SNDSYS_leavecritical();
}

// MATCHING
bool GinsuSynthesis::SetSynthData(class GinsuSynthData& data)
{
	int samprate = data.GetSampleRate();
	if (samprate == 0)
		return false;
	if (mSampleRate > 0 && samprate != mSampleRate)
		return false;

	int nojumpsize  = IntRound(static_cast<float>(samprate) * 0.011f);
	int packetsize  = IntRound(static_cast<float>(samprate) * 0.011f);
	int overlapsize = IntRound(static_cast<float>(samprate) * 0.0005f);
	if (overlapsize + packetsize > mMaxPacketSize)
		return false;

	SNDSYS_entercritical();
	mNoJumpSize = nojumpsize;
	mPacketSize = packetsize;
	mOverlapSize = overlapsize;
	mSynthData = &data;
	SNDSYS_leavecritical();

	return true;
}

// MATCHING
bool GinsuSynthesis::UpdateFrequency(float targetFreq, float latency)
{
	int sample = 0;
	if (mSynthData)
		sample = mSynthData->FrequencyToSample(targetFreq);

	SNDSYS_entercritical();
	mTargetPos = sample;
	mPacketCountdown = IntFloor(latency * 0.090909094f) + 1;
	SNDSYS_leavecritical();

	return true;
}