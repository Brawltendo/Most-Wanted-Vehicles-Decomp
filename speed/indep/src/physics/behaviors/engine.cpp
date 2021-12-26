#include "physics/behaviors/engine.h"

#include "math/bmath.h"
#include "math/mathcommon.h"


// MATCHING
float EngineRacer::GetBrakingTorque(float engine_torque, float rpm)
{
	uint32_t numpts = mEngineInfo.Num_ENGINE_BRAKING();
	if (numpts > 1)
	{
		float ratio;
		float rpm_min = mEngineInfo.IDLE();
		float rpm_max = mEngineInfo.MAX_RPM();
		float rpm_clamped = UMath::Clamp_(rpm, mEngineInfo.IDLE(), mEngineInfo.RED_LINE());
		uint32_t index = UMath::InterpolateIndex(numpts - 1, rpm_clamped, rpm_min, rpm_max, ratio);

		float base = mEngineInfo.ENGINE_BRAKING(index);
		uint32_t second_index = index + 1;
		float step = mEngineInfo.ENGINE_BRAKING(UMath::Min(second_index, numpts - 1));
		float load_pct = (step - base) * ratio + base;
		return -(UMath::Clamp_(load_pct, 0.f, 1.f) * engine_torque);
	}
	else
		return -(engine_torque * mEngineInfo.ENGINE_BRAKING(0));
}

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
