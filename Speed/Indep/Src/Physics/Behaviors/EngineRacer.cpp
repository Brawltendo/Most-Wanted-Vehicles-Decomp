#include "physics/behaviors/engineracer.h"

#include "math/bmath.h"
#include "math/mathcommon.h"
#include "physics/physicsinfo.hpp"

#include "interfaces/simables/ISuspension.h"


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
uint32_t EngineRacer::GetNumGearRatios()
{
	return mTrannyInfo.Num_GEAR_RATIO();
}

// MATCHING
float EngineRacer::GetGearRatio(uint32_t idx)
{
	return mTrannyInfo.GEAR_RATIO(idx);
}

// MATCHING
float EngineRacer::GetGearEfficiency(uint32_t idx)
{
	return mTrannyInfo.GEAR_EFFICIENCY(idx);
}

// MATCHING
float EngineRacer::GetFinalGear()
{
	return mTrannyInfo.FINAL_GEAR();
}

// MATCHING
float EngineRacer::GetRatioChange(uint32_t from, uint32_t to)
{
	float ratio1 = mTrannyInfo.GEAR_RATIO(from);
	float ratio2 = mTrannyInfo.GEAR_RATIO(to);

	if (ratio1 > 0.f && ratio2 > FLT_EPSILON)
		return ratio1 / ratio2;
	else
		return 0.f;
}

// MATCHING
float EngineRacer::GetShiftDelay(uint32_t gear)
{
	return mTrannyInfo.SHIFT_SPEED() * mTrannyInfo.GEAR_RATIO(gear);
}

// MATCHING
bool EngineRacer::RearWheelDrive()
{
	return mTrannyInfo.TORQUE_SPLIT() < 1.f;
}

// MATCHING
bool EngineRacer::FrontWheelDrive()
{
	return mTrannyInfo.TORQUE_SPLIT() > 0.f;
}

// NOT MATCHING
void EngineRacer::LimitFreeWheels(float w)
{
	uint32_t numwheels = mSuspension->GetNumWheels();
	for (uint32_t i = 0; i < numwheels; ++i)
	{
		if (!mSuspension->IsWheelOnGround(i))
		{
			if (i < 2)
			{
				if (!FrontWheelDrive())
					continue;
			}
			else if (!RearWheelDrive())
				continue;
			
			float ww = mSuspension->GetWheelAngularVelocity(i);
			float ww_final = ww;
			if (ww * w < 0.f)
				ww = 0.f;
			else if (ww > 0.f)
			{
				/* if (ww < w);
				else
					ww = w; */
				//ww = ww < w ? w : ww;
				ww = UMath::Min(ww, w);
			}
			else if (ww < 0.f)
			{
				if (ww > w)
					ww = w;
				//ww = ww > w ? w : ww;
				//ww = UMath::Max(ww, w);
			}
			/* else if ((ww > 0.f && ww < w) || (ww < 0.f && ww > w))
				ww = w; */
			mSuspension->SetWheelAngularVelocity(i, ww);
		}
	}
}

// MATCHING
float EngineRacer::GetDifferentialAngularVelocity(bool locked)
{
	float into_gearbox = 0.f;
	bool in_reverse = GetGear() == G_REVERSE;

	if (FrontWheelDrive())
	{
		float w_vel = (mSuspension->GetWheelAngularVelocity(0)
					+ mSuspension->GetWheelAngularVelocity(1)) 
					* 0.5f;
		if (!locked)
		{
			if (UMath::Abs(w_vel) > 0.f)
				into_gearbox = w_vel;
		}
		else
		{
			if (in_reverse)
			{
				if (!(w_vel < 0.f))
					w_vel = 0.f;
			}
			else
			{
				if (!(w_vel > 0.f))
					w_vel = 0.f;
			}
			into_gearbox = w_vel;
		}
	}

	if (RearWheelDrive())
	{
		float w_vel = (mSuspension->GetWheelAngularVelocity(2)
					+ mSuspension->GetWheelAngularVelocity(3)) 
					* 0.5f;
		if (!locked)
		{
			if (UMath::Abs(w_vel) > UMath::Abs(into_gearbox))
				into_gearbox = w_vel;
		}
		else
		{
			if (in_reverse)
			{
				if (!(w_vel < into_gearbox))
					w_vel = into_gearbox;
			}
			else
			{
				if (!(w_vel > into_gearbox))
					w_vel = into_gearbox;
			}
			into_gearbox = w_vel;
		}
	}

	return into_gearbox;
}

// MATCHING
void EngineRacer::SetDifferentialAngularVelocity(float w)
{
	float current = GetDifferentialAngularVelocity(0);
	float diff = w - current;
	IVehicle* vehicle = (IVehicle*)(pad[0x38 / 0x4]);
	float speed = MPS2MPH(vehicle->GetAbsoluteSpeed());
	int lockdiff = speed < 40.f;
	if (RearWheelDrive())
	{
		if (!mSuspension->IsWheelOnGround(2) && !mSuspension->IsWheelOnGround(3))
			lockdiff = 1;

		float w1 = mSuspension->GetWheelAngularVelocity(2);
		float w2 = mSuspension->GetWheelAngularVelocity(3);
		if (lockdiff)
			w2 = w1 = (w1 + w2) * 0.5f;

		mSuspension->SetWheelAngularVelocity(2, w1 + diff);
		mSuspension->SetWheelAngularVelocity(3, w2 + diff);
	}

	lockdiff = speed < 40.f;
	if (FrontWheelDrive())
	{
		if (!mSuspension->IsWheelOnGround(0) && !mSuspension->IsWheelOnGround(1))
			lockdiff = 1;

		float w1 = mSuspension->GetWheelAngularVelocity(0);
		float w2 = mSuspension->GetWheelAngularVelocity(1);
		if (lockdiff)
			w2 = w1 = (w1 + w2) * 0.5f;

		mSuspension->SetWheelAngularVelocity(0, w1 + diff);
		mSuspension->SetWheelAngularVelocity(1, w2 + diff);
	}
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
			return 1.f - UMath::Ramp(mTime, 0.f, mEngageTime) * 0.75f;
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

float SmoothRPMDecel[] = { 2.5f, 15.f };
// MATCHING
float Engine_SmoothRPM(bool is_shifting, GearID gear, float dT, float old_rpm, float new_rpm, float engine_inertia)
{
	bool fast_shifting = is_shifting && gear > G_FIRST || gear == G_NEUTRAL;
	// this ternary is dumb but that's what makes it match
	float max_rpm_decel = -SmoothRPMDecel[fast_shifting ? 1 : 0];
	float rpm = new_rpm;
	float rpm_decel = max_rpm_decel / engine_inertia * 1000.f;
	if (dT > 0.f && (new_rpm - old_rpm) / dT < rpm_decel)
	{
		float newrpm = rpm_decel * dT + old_rpm;
		if (!(newrpm < new_rpm))
			rpm = newrpm;
	}
	return rpm * 0.55f + old_rpm * 0.45f;
}

// NOT MATCHING
// will match with proper inheritance though so this code is correct
// <@>PRINT_ASM
float EngineRacer::GetMaxSpeedometer()
{
	uint32_t num_ratios = mTrannyInfo.Num_GEAR_RATIO();
	if (num_ratios > 0)
	{
		float limiter = MPH2MPS(mEngineInfo.SPEED_LIMITER(0));
		float rpm_max = mEngineInfo.RED_LINE();
		Physics::Tunings* tunings = GetTunings();
		float max_speedometer = Physics::Info::Speedometer(
								mTrannyInfo, 
								mEngineInfo, 
								mTireInfo, 
								rpm_max,
								(GearID)(num_ratios - 1),
								tunings);
		if (limiter > 0.f && !(max_speedometer < limiter))
			return limiter;
		else
			return max_speedometer;
	}
	else
		return 0.f;
}
