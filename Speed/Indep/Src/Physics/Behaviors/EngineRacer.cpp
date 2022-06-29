#include "Physics/Behaviors/EngineRacer.h"
#include "Physics/Behaviors/PInput.h"
#include "Physics/PhysicsInfo.hpp"

// Math
#include "Math/bMath.h"
#include "Math/mathcommon.h"

// Interfaces
#include "Interfaces/Simables/ICheater.h"
#include "Interfaces/Simables/ISimable.h"
#include "Interfaces/Simables/ISuspension.h"
#include "Interfaces/Simables/IVehicle.h"
#include "Interfaces/SimEntities/IPlayer.h"

// Events
#include "Speed/Indep/Src/Generated/Events/EPlayerShift.h"


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::IsNOSEngaged()
{
	return mNOSEngaged >= 1.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::HasNOS()
{
	return mNOSInfo.NOS_CAPACITY() > 0.f && mNOSInfo.TORQUE_BOOST() > 0.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetNOSFlowRate()
{
	return mNOSInfo.FLOW_RATE();
}


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::ChargeNOS(float charge)
{
	if (HasNOS())
		mNOSCapacity = UMath::Clamp_(mNOSCapacity + charge, 0.f, 1.f);
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::IsEngineBraking()
{
	return mEngineBraking;
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::IsShiftingGear()
{
	return mGearShiftTimer > 0.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::UseRevLimiter()
{
	return true;
}


//-------------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------------
// MATCHING
EngineRacer::Clutch::Clutch()
{
    mState = ENGAGED;
    mTime = 0.f;
	mEngageTime = 0.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::Clutch::Disengage()
{
	if (mState == ENGAGED)
		mState = DISENGAGED;
}


//-------------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::Clutch::Reset()
{
	mState = ENGAGED;
	mTime = 0.f;
}


//-------------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------------
// MATCHING
EngineRacer::Clutch::State EngineRacer::Clutch::GetState()
{
	return mState;
}


//-------------------------------------------------------------------------------------
// MATCHING
uint32_t EngineRacer::GetNumGearRatios()
{
	return mTrannyInfo.Num_GEAR_RATIO();
}


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetGearRatio(uint32_t idx)
{
	return mTrannyInfo.GEAR_RATIO(idx);
}


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetGearEfficiency(uint32_t idx)
{
	return mTrannyInfo.GEAR_EFFICIENCY(idx);
}


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetFinalGear()
{
	return mTrannyInfo.FINAL_GEAR();
}


//-------------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetShiftDelay(uint32_t gear)
{
	return mTrannyInfo.SHIFT_SPEED() * mTrannyInfo.GEAR_RATIO(gear);
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::RearWheelDrive()
{
	return mTrannyInfo.TORQUE_SPLIT() < 1.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::FrontWheelDrive()
{
	return mTrannyInfo.TORQUE_SPLIT() > 0.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
ShiftPotential EngineRacer::FindShiftPotential(GearID gear, float rpm)
{
	if (gear <= G_NEUTRAL)  return SHIFT_POTENTIAL_NONE;

	float shift_up_rpm = mShiftUpRPM[gear];
	float shift_down_rpm = mShiftDownRPM[gear];
	float lower_gear_ratio = mTrannyInfo.GEAR_RATIO(gear - 1);

	// is able to shift down
	if (gear != G_FIRST && lower_gear_ratio > 0.f)
	{
		float lower_gear_shift_up_rpm = mShiftUpRPM[gear - 1];
		lower_gear_shift_up_rpm = GetGearRatio(gear) * (lower_gear_shift_up_rpm / lower_gear_ratio) - 200.f;
		// the RPM to shift down is lowered when the throttle isn't pressed
		float off_throttle_rpm = UMath::Min(UMath::Lerp(UMath::Lerp(mEngineInfo.IDLE(), shift_down_rpm, 0.65f), shift_down_rpm, mThrottle), lower_gear_shift_up_rpm); 
		shift_down_rpm = off_throttle_rpm;
	}

	// shifting up
	if (rpm >= shift_up_rpm && gear < GetTopGear()) return SHIFT_POTENTIAL_UP;
	// shifting down or not shifting at all
	else return static_cast<ShiftPotential>(rpm <= shift_down_rpm && gear > G_FIRST);
}


//-------------------------------------------------------------------------------------
// MATCHING
ShiftStatus EngineRacer::OnGearChange(GearID gear)
{
	if (gear >= mTrannyInfo.Num_GEAR_RATIO())
		return SHIFT_STATUS_NONE;
	// new gear can't be the same as the old one
	if (gear == mGear || gear < G_REVERSE)
		return SHIFT_STATUS_NONE;

	mGearShiftTimer = gear < mGear ? (GetShiftDelay(gear) * 0.25f) : GetShiftDelay(gear);
	mGear = gear;
	mClutch.Disengage();
	return SHIFT_STATUS_NORMAL;
}


//-------------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetDriveWheelSlippage()
{
	float retval = 0.f;
	int drivewheels = 0;
	if (RearWheelDrive())
	{
		drivewheels += 2;
		retval += mSuspension->GetWheelSlip(TIRE_RR) + mSuspension->GetWheelSlip(TIRE_RL);
	}
	if (FrontWheelDrive())
	{
		drivewheels += 2;
		retval += mSuspension->GetWheelSlip(TIRE_FR) + mSuspension->GetWheelSlip(TIRE_FL);
	}

	return retval / drivewheels;
}


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::SetDifferentialAngularVelocity(float w)
{
	float current = GetDifferentialAngularVelocity(0);
	float diff = w - current;
	IVehicle* vehicle = GetVehicle();
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


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::CalcSpeedometer(float rpm, uint32_t gear)
{
	Physics::Tunings* tunings = GetVehicle()->GetTunings();
	return Physics::Info::Speedometer(
		   mTrannyInfo, 
		   mEngineInfo, 
		   mTireInfo, 
		   rpm,
		   (GearID)gear,
		   tunings);
}


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetMaxSpeedometer()
{
	uint32_t num_ratios = mTrannyInfo.Num_GEAR_RATIO();
	if (num_ratios > 0)
	{
		float limiter = MPH2MPS(mEngineInfo.SPEED_LIMITER(0));
		float rpm_max = mEngineInfo.RED_LINE();
		Physics::Tunings* tunings = GetVehicle()->GetTunings();
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


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetSpeedometer()
{
	return CalcSpeedometer(RPS2RPM(mTransmissionVelocity), mGear);
}


//-------------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------------
float SmoothRPMDecel[] = { 2.5f, 15.f };


//-------------------------------------------------------------------------------------
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


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::DoECU()
{
	if (GetGear() > G_NEUTRAL)
	{
		// the speed at which the limiter starts to kick in
		float limiter = MPH2MPS(mEngineInfo.SPEED_LIMITER(0));
		if (limiter > 0.f)
		{
			// the speed for the limiter to take full effect
			float cutoff = MPH2MPS(mEngineInfo.SPEED_LIMITER(1));
			if (cutoff > 0.f)
			{
				float speedometer = GetSpeedometer();
				if (speedometer > limiter)
				{
					float limiter_range = speedometer - limiter;
					mThrottle *= (1.f - UMath::Clamp(limiter_range / cutoff, 0.f, 1.f));
				}
			}
		}
	}
}


bool Tweak_InfiniteNOS = false;
//-------------------------------------------------------------------------------------
float EngineRacer::DoNos(const Physics::Tunings* tunings, float dT, bool engaged)
{
	if (!HasNOS())  return 1.f;

	float speed_mph = MPS2MPH(GetVehicle()->GetAbsoluteSpeed());
	float recharge_rate = 0.f;
	IPlayer* player = GetOwner()->GetPlayer();

	if  (!player || player->CanRechargeNOS())
	{
		float min_speed = mNOSInfo.RECHARGE_MIN_SPEED();
		float max_speed = mNOSInfo.RECHARGE_MAX_SPEED();
		if (speed_mph >= min_speed && mGear >= G_FIRST)
		{
			float t = UMath::Ramp(speed_mph, min_speed, max_speed);
			recharge_rate = UMath::Lerp(mNOSInfo.RECHARGE_MIN(), mNOSInfo.RECHARGE_MAX(), t);
		}
	}

	if (mGear < G_FIRST || mThrottle <= 0.f || IsBlown())
		engaged = false;
	if (speed_mph < 10.f && !IsNOSEngaged() || speed_mph < 5.f && IsNOSEngaged())
		engaged = false;

	float nos_discharge = Physics::Info::NosCapacity(mNOSInfo, tunings);
	float nos_torque_scale = 1.f;
	if (nos_discharge > 0.f)
	{
		float nos_disengage = mNOSInfo.NOS_DISENGAGE();
		if (engaged && mNOSCapacity > 0.f)
		{
			float discharge = dT / nos_discharge;
			// don't deplete nitrous
			if (Tweak_InfiniteNOS || GetVehicle()->GetDriverClass() == DRIVER_REMOTE)
				discharge = 0.f;
			// GetCatchupCheat returns 0.0 for human racers, but AI racers get hax
			if (mCheater)
				discharge *= UMath::Lerp(1.f, 0.5f, mCheater->GetCatchupCheat());
			mNOSCapacity -= discharge;
			nos_torque_scale = Physics::Info::NosBoost(mNOSInfo, tunings);
			mNOSEngaged = 1.f;
			// mNOSCapacity = UMath::Max(mNOSCapacity - discharge, 0.f);
			mNOSCapacity = UMath::Max(mNOSCapacity, 0.f);
		}
		else if (mNOSEngaged > 0.f && nos_disengage > 0.f)
		{
			// nitrous can't start recharging until the disengage timer runs out
			// it takes [NOS_DISENGAGE] seconds for it to reach zero

			mNOSEngaged -= dT / nos_disengage;
			mNOSEngaged = UMath::Max(mNOSEngaged, 0.f);
		}
		else
		{
			if (mNOSCapacity < 1.f && recharge_rate > 0.f)
			{
				float recharge = dT / recharge_rate;
				// GetCatchupCheat returns 0.0 for human racers, but AI racers get hax
				if (mCheater)
					recharge *= UMath::Lerp(1.f, 2.f, mCheater->GetCatchupCheat());
				mNOSCapacity = UMath::Min(recharge + mNOSCapacity, 1.f);
			}
			mNOSEngaged = 0.f;
		}
		
	}
	else
	{
		// fallback in case someone sets the nitrous capacity <= 0.0 by uncapping tuning limits

		mNOSCapacity = 0.f;
		mNOSEngaged = 0.f;
	}
	return nos_torque_scale;
}


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::DoInduction(const Physics::Tunings* tunings, float dT)
{
	eInductionType type = Physics::Info::InductionType(mInductionInfo);
	if (type == INDUCTION_NONE)
	{
		mSpool = 0.f;
		mInductionBoost = 0.f;
		mPSI = 0.f;
		return;
	}

	float desired_spool = UMath::Ramp(mThrottle, 0.f, 0.5f);
	float rpm = RPS2RPM(mAngularVelocity);

	if (IsGearChanging())
		desired_spool = 0.f;
	// turbocharger can't start spooling up until the engine rpm is >= the boost threshold
	if (type == INDUCTION_TURBO_CHARGER
	&& Physics::Info::InductionRPM(mEngineInfo, mInductionInfo, tunings) > rpm)
		desired_spool = 0.f;
	
	if (mSpool > desired_spool)
	{
		float spool_time = mInductionInfo.SPOOL_TIME_DOWN();
		if (spool_time > FLT_EPSILON)
		{
			mSpool -= dT / spool_time;
			mSpool = UMath::Max(mSpool, desired_spool);
		}
		else
		{
			mSpool = desired_spool;
		}
	}
	else if (mSpool < desired_spool)
	{
		float spool_time = mInductionInfo.SPOOL_TIME_UP();
		if (spool_time > FLT_EPSILON)
		{
			mSpool += dT / spool_time;
			mSpool = UMath::Min(mSpool, desired_spool);
		}
		else
		{
			mSpool = desired_spool;
		}
	}

	float target_psi;
	mSpool = UMath::Clamp_(mSpool, 0.f, 1.f);
	mInductionBoost = Physics::Info::InductionBoost(mEngineInfo, mInductionInfo, rpm, mSpool, tunings, &target_psi);
	if (mPSI > target_psi)
		mPSI = UMath::Max(mPSI - dT * 20.f, target_psi);
	else if (mPSI < target_psi)
		mPSI = UMath::Min(mPSI + dT * 20.f, target_psi);
}


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::DoThrottle()
{
	if (!IsBlown())
	{
		if (!mIInput)  return 0.f;
		else  return mIInput->GetControls().fGas;
	}
	else
	{
		// cut the throttle when the engine is blown
		return 0.f;
	}
}


//-------------------------------------------------------------------------------------
// MATCHING
float EngineRacer::GetShiftPoint(GearID from_gear, GearID to_gear)
{
	if (from_gear < G_NEUTRAL || to_gear <= G_NEUTRAL)
		return 0.f;
	if (to_gear > from_gear)
		return mShiftUpRPM[from_gear];
	if (to_gear >= from_gear)
		return 0.f;
	else
		return mShiftDownRPM[from_gear];
}


//-------------------------------------------------------------------------------------
// MATCHING
// <@>PRINT_ASM
float EngineRacer::GetPerfectLaunchRange(float& range)
{
	// perfect launch only applies to first gear
	if (mGear != G_FIRST)
	{
		range = 0.f;
		return 0.f;
	}
	else
	{
		range = (mEngineInfo.RED_LINE() - mEngineInfo.IDLE()) * 0.25f;
		float upper_limit = mEngineInfo.RED_LINE() - 500.f;
		return UMath::Min(range + mPeakTorqueRPM, upper_limit) - range;
	}
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::DoGearChange(GearID gear, bool automatic)
{
	// can't shift past top gear
	if (gear > GetTopGear())  return false;
	// can't shift below reverse gear
	if (gear < G_REVERSE)  return false;

	GearID previous = mGear;
	ShiftStatus status = OnGearChange(gear);
	// has shifted
	if (status != SHIFT_STATUS_NONE)
	{
		mShiftStatus = status;
		mShiftPotential = SHIFT_POTENTIAL_NONE;
		ISimable* owner = GetOwner();
		// AI shifted
		if (!owner->IsPlayer())  return true;

		// dispatch shift event
		new EPlayerShift(owner->GetInstanceHandle(), status, automatic, previous, gear);
		// player shifted
		return true;
	}
	// didn't shift
	return false;
}


//-------------------------------------------------------------------------------------
// MATCHING
bool EngineRacer::Shift(GearID gear)
{
	return DoGearChange(gear, false);
}


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::AutoShift()
{
	if (mGear == G_REVERSE 
	|| mGearShiftTimer > 0.f 
	|| GetVehicle()->IsStaging() 
	|| mSportShifting > 0.f)
		return;

	// skip neutral when using auto transmission
	if (mGear == G_NEUTRAL)
	{
		mGear = G_FIRST;
		return;
	}

	switch (mShiftPotential)
	{
		case SHIFT_POTENTIAL_DOWN:
		{
			int next_gear = mGear - 1;
			if (next_gear > G_FIRST)
			{
				float current_rpm = RPS2RPM(mTransmissionVelocity);
				float rpm = GetRatioChange(next_gear, mGear) * current_rpm;
				do
				{
					if (FindShiftPotential((GearID)next_gear, rpm) != SHIFT_POTENTIAL_DOWN)  break;
					rpm = GetRatioChange(--next_gear, mGear) * current_rpm;
				} while (next_gear > G_FIRST);
			}
			DoGearChange((GearID)next_gear, true);
			break;
		}

		case SHIFT_POTENTIAL_UP:
		case SHIFT_POTENTIAL_PERFECT:
		case SHIFT_POTENTIAL_MISS:
		{
			int have_traction = 1;
			for (int i = 0; i < 4; ++i)
			{
				have_traction &= mSuspension->IsWheelOnGround(i) && mSuspension->GetWheelSlip(i) < 4.f;
			}
			if (have_traction)  DoGearChange((GearID)(mGear + 1), true);
			break;
		}
		
		default:
			return;
	}
}


//-------------------------------------------------------------------------------------
// MATCHING
void EngineRacer::DoShifting(float dT)
{
	if (mIInput && mIInput->IsAutomaticShift())  AutoShift();

	if (mGearShiftTimer > 0.f)
	{
		mGearShiftTimer -= dT;
		if (mGearShiftTimer <= 0.f)  mGearShiftTimer = 0.f;
	}

	if (mSportShifting > 0.f && mShiftPotential)
	{
		if (mIInput)
		{
			float gas = mIInput->GetControls().fGas;
			mSportShifting = UMath::Max(mSportShifting - (2.f - gas) * dT, 0.f);
		}
		else
		{
			mSportShifting = 0.f;
		}
	}
}