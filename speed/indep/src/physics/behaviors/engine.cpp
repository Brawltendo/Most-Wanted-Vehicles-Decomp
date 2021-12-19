#include "physics/behaviors/engine.h"

#include "math/bmath.h"
#include "math/mathcommon.h"


// MATCHING
EngineRacer::Clutch::Clutch()
{
    mState = ENGAGED;
    mTime = 0.f;
	mEngageTime = 0.f;
}

// MATCHING
void EngineRacer::Clutch::Disengage()
{
	if (mState == ENGAGED)
		mState = DISENGAGED;
}

// MATCHING
void EngineRacer::Clutch::Engage(float time)
{
	if (mState == DISENGAGED)
	{
		if (time > 0.f)
			mState = ENGAGING;
		else
			mState = ENGAGED;
		mTime = time;
		mEngageTime = time;
	}
}

// MATCHING
void EngineRacer::Clutch::Reset()
{
	mState = ENGAGED;
	mTime = 0.f;
}

// MATCHING
float EngineRacer::Clutch::Update(float dT)
{
	if (mTime > 0.f)
	{
		mTime -= dT;
		// engage the clutch when the timer hits or drops below zero
		if (mTime <= 0.f && (mState - 1) == ENGAGED)
			mState = ENGAGED;
	}

	// return clutch ratio
	switch (mState)
	{
		case ENGAGED:
			return 1.f;
		case ENGAGING:
			return 1.f - UMath::InverseLerp(mTime, 0.f, mEngageTime) * 0.75f;
		case DISENGAGED:
			return 0.25f;
		default:
			return 1.f;
	}
}

// MATCHING
EngineRacer::Clutch::State EngineRacer::Clutch::GetState()
{
	return mState;
}
