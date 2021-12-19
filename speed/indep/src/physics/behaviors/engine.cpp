#include "physics/behaviors/engine.h"

#include "math/bmath.h"
#include "math/mathcommon.h"

float EngineRacer::Clutch::GetClutchRatio(float dT)
{
	if (mTime > 0.f)
	{
		mTime -= dT;
		// engage the clutch when the timer hits or drops below zero
		int state = mState;
		if (mTime <= 0.f && mState)
			mState = ENGAGED;
	}

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