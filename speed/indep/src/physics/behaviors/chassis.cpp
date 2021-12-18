#include "physics/behaviors/chassis.h"
#include "math/mathcommon.h"

extern float Sim_GetTime();
extern void ScaleVector(UMath::Vector3* in, const float scale, UMath::Vector3& dest);
extern UMath::Vector3* _cdecl TransformVector(const UMath::Vector3& v, const UMath::Matrix4& m, UMath::Vector3& dest);
extern float VU0_Atan2(float opposite, float adjacent);

/* SuspensionRacer::SuspensionRacer()
{
    mRB = NULL;
    mCollisionBody = NULL;
    mGameBreaker = 0.f;
    mNumWheelsOnGround = 0;
    mLastGroundCollision = 0.f;
    
    mDrift.State = Drift::eState::D_OUT;
    mDrift.Value = 0.f;

    mBurnOut.mState = 0;
    mBurnOut.mBurnOutTime = 0.f;
    mBurnOut.mTraction = 1.f;
    mBurnOut.mBurnOutAllow = 0.f;

    mTires[0] = NULL;
    mTires[1] = NULL;
    mTires[2] = NULL;
    mTires[3] = NULL;
} */

// MATCHING
/* void SuspensionRacer::ComputeAckerman(const float steering, const Chassis::State& state, UMath::Vector4& left, UMath::Vector4& right)
{
	UMath::Vector3 steer_vec;
	int going_right = true;
	float wheel_base = mChassisInfo.data->WHEEL_BASE;
	float track_width_front = mChassisInfo.data->TRACK_WIDTH[0];
	float steering_angle_radians = steering * TWO_PI;

	if (steering_angle_radians > PI)
		steering_angle_radians -= TWO_PI;

	if (steering_angle_radians < 0.f)
	{
		going_right = false;
		steering_angle_radians = -steering_angle_radians;
	}

	// Ackermann steering geometry causes the outside wheel to have a smaller turning angle than the inside wheel
	// this is determined by the distance of the wheel to the center of the rear axle
	// this equation is a modified version of 1/tan(L/(R+T/2)), where L is the wheelbase, R is the steering radius, and T is the track width
	float steer_left;
	float steer_right;
	float steer_outside = (steering_angle_radians * wheel_base)
					    / (steering_angle_radians * track_width_front + wheel_base);
	if (going_right)
	{
		steer_left = steer_outside;
		steer_right = steering_angle_radians;
	}
	else
	{
		steer_left = -steering_angle_radians;
		steer_right = -steer_outside;
	}

	// calculate forward vector for front wheels
	steer_vec.y = 0.f;
	steer_vec.z = cosf(steer_right);
	steer_vec.x = sinf(steer_right);
	TransformVector(steer_vec, state.matrix, steer_vec);
	right = UMath::Vector4(steer_vec, steer_right);

	steer_vec.y = 0.f;
	steer_vec.z = cosf(steer_left);
	steer_vec.x = sinf(steer_left);
	TransformVector(steer_vec, state.matrix, steer_vec);
	left = UMath::Vector4(steer_vec, steer_left);
} */

// NOT MATCHING
// there are very slight differences in the instruction order detailed below
// everything else matches though and these differences shouldn't be an issue
/* void SuspensionRacer::Burnout::Update(const float dT, const float speed_mph, const float wheel_slip, const int wheel_ind, const float yaw)
{
	if (mState)
	{
		if (speed_mph > 5.f)
		{
			float burnoutYaw = yaw * TWO_PI;
			ABS(burnoutYaw);
			if (burnoutYaw > BurnOutYawCancel)
			{
				mState = 0;
				mBurnOutTime = 0.f;
				mBurnOutAllow = 0.f;
				mTraction = 1.f;
				return;
			}
		}
		if (wheel_slip < BurnOutCancelSlipValue)
		{
			mBurnOutAllow += dT;
			if (mBurnOutAllow > BurnOutAllowTime);
			else return;
			mState = 0;
			mBurnOutTime = 0.f;
			mBurnOutAllow = 0.f;
			mTraction = 1.f;
			return;
		}
		mBurnOutAllow = 0.f;
		mBurnOutTime -= dT;
		if (mBurnOutTime < 0.f)
		{
			--mState;
			// this gets compiled before the decrement but in the original it comes after
			mBurnOutTime = BurnOutFishTailTime;
		}
	}
	else if (speed_mph < BurnOutMaxSpeed && wheel_slip > 0.5f)
	{
		float friction;
		BurnoutFrictionTable.GetValue(friction, wheel_slip);
		mTraction = friction / 1.4f;
		// burnout state changes according to what side of the axle the wheel is on
		mState = (int)((1.5f - friction) * BurnOutFishTails + (wheel_ind & 1));
		// this gets compiled before mState is assigned instead of after
		// seems like it might have something to do with BurnOutFishTailTime since it's in both differing instances
		mBurnOutTime = BurnOutFishTailTime;
		mBurnOutAllow = 0.f;
	}
} */

// NOT MATCHING
// see comments for explanation
/* void SuspensionRacer::DoDrifting(const Chassis::State& state)
{
	if (mDrift.State && ((state.flags & 1) || state.driver_style == 1))
	{
		mDrift.State = SuspensionRacer::Drift::eState::D_OUT;
		mDrift.Value = 0.f;
		return;
	}

	float drift_change = 0.f;
	switch (mDrift.State)
	{
		case SuspensionRacer::Drift::eState::D_OUT:
		case SuspensionRacer::Drift::eState::D_EXIT:
			drift_change = -2.f;
			break;
		case SuspensionRacer::Drift::eState::D_ENTER:
		case SuspensionRacer::Drift::eState::D_IN:
			drift_change = 8.f;
			break;
		default:
			break;
	}

	mDrift.Value += drift_change * state.time;
	// clamp the drift value between 0 and 1
	if (mDrift.Value <= 0.f)
	{
		mDrift.State = SuspensionRacer::Drift::eState::D_OUT;
		mDrift.Value = 0.f;
	}
	else if (mDrift.Value >= 1.f)
	{
		mDrift.State = SuspensionRacer::Drift::eState::D_IN;
		mDrift.Value = 1.f;
	}

	if (mDrift.State > SuspensionRacer::Drift::eState::D_ENTER)
	{
		float steering_avg = mSteering.InputAverage.fAverage;
		if ((state.local_angular_vel.y * state.slipangle) < 0.f)
		{
			float abs_slipangle = state.slipangle;
			ABS(abs_slipangle);
			if (abs_slipangle <= 0.25f
			&& state.speed > MPH_TO_MPS(30.00005f)
			&& (steering_avg * state.slipangle) <= 0.f
			&& fabsf(state.slipangle) > NORMALIZE_ANGLE_DEGREES(12.f))
			{
				mDrift.State = SuspensionRacer::Drift::eState::D_IN;
				goto InDrift;
			}
		}
		if ((state.steer_input * state.slipangle * state.gas_input) > NORMALIZE_ANGLE_DEGREES(12.f) 
		&& state.speed > MPH_TO_MPS(30.00005f))
		{
			mDrift.State = SuspensionRacer::Drift::eState::D_IN;
			goto InDrift;
		}
		float abs_slipangle = state.slipangle;
		ABS(abs_slipangle);
		if ((abs_slipangle * state.gas_input) <= NORMALIZE_ANGLE_DEGREES(12.f))
		{
			mDrift.State = SuspensionRacer::Drift::eState::D_EXIT;
			goto InDrift;
		}
		mDrift.State = SuspensionRacer::Drift::eState::D_ENTER;
	}

	else if (state.speed > MPH_TO_MPS(30.00005f) 
	&& (state.ebrake_input > 0.5f || fabsf(state.slipangle) > NORMALIZE_ANGLE_DEGREES(12.f)))
		mDrift.State = SuspensionRacer::Drift::eState::D_ENTER;

	InDrift:
	if (!(mDrift.Value <= 0.f))
	{
		float yaw = state.local_angular_vel.y;
		// chassis slip angle is stored as a value in the range [-1,1]
		// so multiplying by 2pi gives the entire possible angle range in radians
		float slipangle_radians = state.slipangle * TWO_PI;

		// charge speedbreaker if not in use and drifting is detected
		if (mGameBreaker <= 0.f 
		&& state.speed > MPH_TO_MPS(35.000058f) 
		&& fabsf(slipangle_radians) > 0.52358997f)
		{
			// ugly ass function pointers because I don't wanna deal with this class's inheritance and virtual functions right now
			// you get the point anyway

			// LocalPlayer::IPlayer* PhysicsObject::GetPlayer()
			int* player = (*(int* (**)())((*(int*)pad[12]) + 0x20))();
			if (player)
			{
				float charge = mDrift.Value * state.time * 0.5f;
				// void LocalPlayer::ChargeGameBreaker(float amount)
				(*(void (__fastcall **)(int*, float))((*player) + 0x4C))(player, charge);
			}
		}

		// apply yaw damping torque
		if ((yaw * slipangle_radians) < 0.f && mNumWheelsOnGround >= 2)
		{
			float driftStabilization;
			DriftStabilizerTable.GetValue(driftStabilization, fabsf(slipangle_radians));
			float moment = state.inertia.y * mDrift.Value * driftStabilization * yaw * -4.f;
			UMath::Vector3 yaw_damping;
			// multiply up vector by yaw moment to get the final amount of damping to apply
			ScaleVector((UMath::Vector3*)&state.matrix.v1, moment, yaw_damping);

			// void RigidBody::ResolveTorque(UMath::Vector3* torque)
			// the registers flipped here because I had to force the code to line up by making this a stdcall
			// in reality it would be a thiscall but that isn't an assignable keyword in MSVC7.1
			// with proper vfunctions instead of this abomination it would fix itself anyway
			(*(void (__stdcall **)(UMath::Vector3*))(*(int*)mRB + 0x94))(&yaw_damping);
		}

		// set steering multiplier when counter steering is detected
		float abs_steering = 0.f;
		if ((slipangle_radians * state.steer_input) > 0.f)
		{
			abs_steering = state.steer_input;
			ABS(abs_steering);
		}
		float abs_slipangle = slipangle_radians < 0.f ? -slipangle_radians : slipangle_radians;
		float abs_yaw = yaw < 0.f ? -yaw : yaw;
		float driftmult_rear = DriftRearFrictionTable.GetValue(((abs_yaw + abs_slipangle) * 0.5f + abs_steering * 4.f) * mDrift.Value);
		mTires[2]->mDriftFriction = driftmult_rear;
		mTires[3]->mDriftFriction = driftmult_rear;
	}
} */

// MATCHING
/* float SuspensionRacer::DoHumanSteering(const Chassis::State& state)
{
	float input = state.steer_input;
	float prev_steering = mSteering.Previous;

	if (prev_steering >= 180.f)
		prev_steering -= 360.f;

	float steering_coeff = mTireInfo.data->STEERING;
	ISteeringWheel::SteeringType steer_type = ISteeringWheel::SteeringType::kGamePad;

	// LocalPlayer::IPlayer* PhysicsObject::GetPlayer()
	int* player = (*(int* (**)())((*(int*)pad[12]) + 0x20))();
	if (player)
	{
		// SteeringWheelDevice* LocalPlayer::GetSteeringDevice()
		int* steering_device = (*(int* (__fastcall **)(int*))((*player) + 0x3C))(player);

		if (steering_device)
		{
			// bool SteeringWheelDevice::IsConnected()
			if ( (*(bool (__fastcall **)(int*))((*steering_device) + 0xC))(steering_device) )
			{
				// ISteeringWheel::SteeringType SteeringWheelDevice::GetSteeringType()
				steer_type = (*(ISteeringWheel::SteeringType (__fastcall **)(int*))((*steering_device) + 0x10))(steering_device);
			}
			
		}
	}

	float max_steering = CalculateMaxSteering(state, steer_type) * steering_coeff * input;
	float max_steer_range = UMath::Clamp(max_steering, -45.f, 45.f);
	float new_steer = max_steer_range;

	if (steer_type == ISteeringWheel::SteeringType::kGamePad)
	{
		input = SteerInputRemapTables->GetValue(input);
		float steer_speed = (CalculateSteeringSpeed(state) * steering_coeff) * state.time;
		float inc_steer = prev_steering + steer_speed;
		float dec_steer = prev_steering - steer_speed;
		if (max_steer_range > dec_steer)
			dec_steer = max_steer_range;
		if (inc_steer < dec_steer)
			dec_steer = inc_steer;
		
		new_steer = dec_steer;
		// this is absolutely pointless but it's part of the steering calculation for whatever reason
		if (fabsf(new_steer) < 0.f)
			new_steer = 0.f;
	}
	mSteering.LastInput = input;
	mSteering.Previous = new_steer;

	// if in speedbreaker, increase the current steering angle beyond the normal maximum
	// this change is instant, so the visual steering angle while in speedbreaker doesn't accurately represent this
	// instead it interpolates to this value so it looks nicer
	if (mGameBreaker > 0.f)
		new_steer += (state.steer_input * 60.f - new_steer) * mGameBreaker;

	mSteering.InputAverage.Record(mSteering.LastInput, Sim_GetTime());
	return new_steer / 360.f;
} */

static float ZeroDegreeTable[6]  = { 0.f };
static float TwoDegreeTable[]    = { 0.f, 1.2f, 2.3f, 3.f, 3.f, 2.8f };
static float FourDegreeTable[]   = { 0.f, 1.7f, 3.2f, 4.3f, 5.1f, 5.2f };
static float SixDegreeTable[]    = { 0.f, 1.8f, 3.5f, 4.9f, 5.8f, 6.1f };
static float EightDegreeTable[]  = { 0.f, 1.83f, 3.6f, 5.f, 5.96f, 6.4f };
static float TenDegreeTable[]    = { 0.f, 1.86f, 3.7f, 5.1f, 6.13f, 6.7f };
static float TwelveDegreeTable[] = { 0.f, 1.9f, 3.8f, 5.2f, 6.3f, 7.1f };
static Table ZeroDegree   = Table(6, 0.f, 10.f, 0.5f, ZeroDegreeTable);
static Table TwoDegree    = Table(6, 0.f, 10.f, 0.5f, TwoDegreeTable);
static Table FourDegree   = Table(6, 0.f, 10.f, 0.5f, FourDegreeTable);
static Table SixDegree    = Table(6, 0.f, 10.f, 0.5f, SixDegreeTable);
static Table EightDegree  = Table(6, 0.f, 10.f, 0.5f, EightDegreeTable);
static Table TenDegree    = Table(6, 0.f, 10.f, 0.5f, TenDegreeTable);
static Table TwelveDegree = Table(6, 0.f, 10.f, 0.5f, TwelveDegreeTable);
static Table* LoadSensitivityTable[] = 
{
	&ZeroDegree, &TwoDegree, &FourDegree, &SixDegree, &EightDegree, &TenDegree, &TwelveDegree
};
static int LatForceMultipliers[] = { -2, -2, -2, -2 };
static int* pLatForceMultipliers = LatForceMultipliers;
float SuspensionRacer::Tire::ComputeLateralForce(float load, float slip_angle)
{
	float angle = (slip_angle * 360.f) * 0.5f;
	uint32_t slip_angle_table = (int)angle;
	float load_sensitivity_in = (load * 0.001f);
	load_sensitivity_in *= 0.8f;
	float extra = angle - slip_angle_table;
	bool use_max_table = slip_angle_table < 6;
	
	if (use_max_table)
	{
		float grip_scale = mSpecs->data->GRIP_SCALE.Pair[mAxleIndex];
		return (((LoadSensitivityTable[6]->GetValue(load_sensitivity_in)
			 * pLatForceMultipliers[2]) * mGripBoost) * grip_scale) * 2500.f;
	}
	else
	{
		float low  = LoadSensitivityTable[slip_angle_table]->GetValue(load_sensitivity_in);
		float high = LoadSensitivityTable[slip_angle_table + 1]->GetValue(load_sensitivity_in);
		//float load_sensitivity = ;
		return (extra * (high - low) + low)
			 * mSpecs->data->GRIP_SCALE.Pair[mAxleIndex] * pLatForceMultipliers[2]
			 * mGripBoost * 2500.f;
	}
}

// MATCHING
float SuspensionRacer::Tire::GetPilotFactor(const float speed)
{
	// these if statements look so stupid but it's the only way I was able to influence the asm to line up
	// it should just be a single statement with ORs but that didn't match for whatever reason
	if (mBrakeLocked)
		return 1.f;
	if (mAV < 0.f)
		return 1.f;
	if (mWheelIndex < 2)
		return 1.f;
	
	float speed_factor = (speed - MPH_TO_MPS(30.00005f)) * 0.111849315f;
	float factor_clamped;
	if (speed_factor < 1.f)
	{
		factor_clamped = speed_factor;
		if (0.f > speed_factor)
			factor_clamped = 0.f;
	}
	else 
		factor_clamped = 1.f;
	return factor_clamped * 0.15f + 0.85f;
}

static float BrakingTorque = 4.f;
static float EBrakingTorque = 10.f;
static float BrakeLockAngularVelocityFactor = 100.f;
static float StaticToDynamicBrakeForceRatio = 1.2f;
// MATCHING
void SuspensionRacer::Tire::CheckForBrakeLock(float ground_force)
{
	Attrib::Gen::brakes::LayoutStruct* brakes_data = mBrakes->data;
	float brake_force = (((brakes_data->BRAKES[mAxleIndex] * 1.3558f) * brakes_data->BRAKE_LOCK[mAxleIndex]) * BrakingTorque) * mBrake;
	brake_force += ((brakes_data->EBRAKE * 1.3558f) * EBrakingTorque) * mEBrake;
	brake_force *= StaticToDynamicBrakeForceRatio;

	float radius = mRadius;
	float abs_av = mAV;
	ABS(abs_av);
	if (brake_force > BrakeLockAngularVelocityFactor * abs_av + radius * ground_force)
	{
		// this could be simplified to mBrakeLocked = brake_force > 1.f;
		// but it has to be done like this for the asm to match
		// it's clearer this way anyway
		if (brake_force > 1.f)
			mBrakeLocked = true;
		else
			mBrakeLocked = false;
		mAV = 0.f;
	}
	else
		mBrakeLocked = false;
}

// MATCHING
void SuspensionRacer::Tire::CheckSign()
{
	// this function is called but doesn't actually seem to be used for anything other than debugging
	// mLastSign isn't used anywhere outside of this function as far as I know

	if (mLastSign == WAS_POSITIVE)
	{
		if (mAV < 0.f)
			mAV = 0.f;
	}
	else if (mLastSign == WAS_NEGATIVE && mAV > 0.f)
		mAV = 0.f;
	
	if (mAV > FLT_EPSILON)
		mLastSign = WAS_POSITIVE;
	else if (mAV < -FLT_EPSILON)
		mLastSign = WAS_NEGATIVE;
	else
		mLastSign = WAS_ZERO;
}

static float WheelMomentOfInertia = 10.f;
// MATCHING
// NOTE: Only matches when the functions that it calls are uncommented, since it needs to know the calling convention
// Updates forces for an unloaded/airborne tire
void SuspensionRacer::Tire::UpdateFree(float dT)
{
	mLoad = 0.f;
	mSlip = 0.f;
	mTraction = 0.f;
	mSlipAngle = 0.f;
	CheckForBrakeLock(0.f);
	
	if (mBrakeLocked)
	{
		mAngularAcc = 0.f;
		mAV = 0.f;
	}
	else
	{
		float brake_torque = ((mBrakes->data->BRAKES[mAxleIndex] * 1.3558f) * BrakingTorque) * mBrake;
		float ebrake_torque = ((mBrakes->data->EBRAKE * 1.3558f) * EBrakingTorque) * mEBrake;
		mBrakeTorque += mAV > 0.f ? -brake_torque  : brake_torque;
		mBrakeTorque += mAV > 0.f ? -ebrake_torque : ebrake_torque;

		float accel = (mBrakeTorque + mDriveTorque) / WheelMomentOfInertia;
		mAngularAcc = accel;
		mAV += accel * dT;
	}
	CheckSign();
	mLateralForce = 0.f;
	mLongitudeForce = 0.f;
}

static float RollingFriction = 2.f;
float SuspensionRacer::Tire::UpdateLoaded(float lat_vel, float fwd_vel, float body_speed, float load, float dT)
{
	float bt = (mBrakes->data->BRAKES[mAxleIndex] * 1.3558f) * BrakingTorque;
	float ebt = (mBrakes->data->EBRAKE * 1.3558f) * EBrakingTorque;
	float dynamic_grip = mSpecs->data->DYNAMIC_GRIP.Pair[mAxleIndex];
	float static_grip = mSpecs->data->STATIC_GRIP.Pair[mAxleIndex];

	if (mLoad <= 0.f && !mBrakeLocked)
		mAV = fwd_vel / mRadius;
	float speed_delta = fwd_vel - mRoadSpeed;
	mRoadSpeed = fwd_vel;
	float fwd_acc = speed_delta / dT;
	mLoad = load > 0.f ? load : 0.f;
	mLateralSpeed = lat_vel;
	bt *= mBrake;
	float ebt_add = ebt * mEBrake;
	//ebt *= mEBrake;
	float abs_fwd_vel = fwd_vel < 0.f ? -fwd_vel : fwd_vel;
	if (abs_fwd_vel < 1.f)
	{
		// when car is nearly stopped, apply brake torque using forward velocity and wheel load
		float inv_radius = 1.f / mRadius;
		float stopping_bt  = -(inv_radius * mBrake  * fwd_vel * load);
		float stopping_ebt = -(inv_radius * mEBrake * fwd_vel * load);
		if (!mBrakeLocked)
		{
			float r_drive_torque = -(mEBrake * mDriveTorque);
			mDriveTorque += (-(mEBrake * mDriveTorque));
			mBrakeTorque += stopping_bt;
			mBrakeTorque = stopping_ebt + mBrakeTorque;
		}
	}
	else
	{
		float opp_bt = mAV > 0.f ? -bt : bt;
		if (!mBrakeLocked)
			mBrakeTorque += opp_bt;
		float opp_ebt = mAV > 0.f ? -ebt_add : ebt_add;
		if (!mBrakeLocked)
			mBrakeTorque += opp_ebt;
	}
	mSlipAngle = VU0_Atan2(lat_vel, abs_fwd_vel);
	float slip_speed = mRadius * mAV - fwd_vel;
	float slip_ground_friction = 0.f;
	float dynamic_friction = 1.f;
	mSlip = slip_speed;
	float lat_vel2 = lat_vel * lat_vel;
	float skid_speed = fsqrt(slip_speed * slip_speed + lat_vel2);
	float pilot_factor = GetPilotFactor(body_speed);
	if (skid_speed > FLT_EPSILON && (lat_vel != 0.f || fwd_vel != 0.f))
	{
		dynamic_friction = dynamic_grip * mTractionBoost * pilot_factor;
		slip_ground_friction = (dynamic_friction / skid_speed) * mLoad;
		float wheel_speed = fsqrt(fwd_vel * fwd_vel + lat_vel2);
		//float ground_force = mLoad / fsqrt(fwd_vel * fwd_vel + lat_vel2) * dynamic_friction * abs_fwd_vel;
		//volatile float wheel_load = mLoad;
		CheckForBrakeLock(mLoad / wheel_speed * abs_fwd_vel * dynamic_friction);
	}
	if (mTraction < 1.f || mBrakeLocked)
	{
		float long_force = slip_speed * slip_ground_friction;
		mLongitudeForce = long_force;
		mLateralForce = -(slip_ground_friction * lat_vel);
		// 0.44703 mps = 1 mph
		const float one_mph = 0.44703f;
		if (body_speed < one_mph && dynamic_friction > 0.1f)
		{
			float inv_friction = 1.f / dynamic_friction;
			mLateralForce *= inv_friction;
			mLongitudeForce = long_force * inv_friction;
		}
		long_force = (mBrakeTorque + mDriveTorque) / mRadius;
		float long_force_clamped = mLongitudeForce;
		if (long_force * long_force_clamped > 0.f)
		{
			if (long_force_clamped > 0.f)
			{
				if (!(long_force_clamped < long_force))
					long_force_clamped = long_force;
			}
			else if (!(long_force_clamped > long_force))
				long_force_clamped = long_force;
		}
		mLongitudeForce = long_force_clamped;
	}
	else
	{
		mBrakeLocked = false;
		mLongitudeForce = (mDriveTorque + mBrakeTorque) / mRadius;
		float slip_ang = mSlipAngle;
		float lat_force = ComputeLateralForce(mLoad, slip_ang < 0.f ? -slip_ang : slip_ang);
		mLateralForce = lat_force;
		if (lat_vel > 0.f)
			mLateralForce = -lat_force;
	}

	mLateralForce *= mLateralBoost;
	if (mTraction >= 1.f && !mBrakeLocked)
		mLongitudeForce += (mAngularAcc * mRadius - fwd_acc) / mRadius * WheelMomentOfInertia;
	bool use_ellipse = false;
	if ((mBrakeTorque + mDriveTorque) * fwd_vel > 0.f && !mBrakeLocked)
	{
		use_ellipse = true;
		mLongitudeForce *= 1.5f;
	}

	float ellipse_x = mLateralForce * mTractionCircle.x;
	mLateralForce = ellipse_x;
	float ellipse_y = mLongitudeForce * mTractionCircle.y;
	mLongitudeForce = ellipse_y;
	float wheel_force_length = ellipse_y * ellipse_y + ellipse_x * ellipse_x;
	wheel_force_length = fsqrt(wheel_force_length);
	mTraction = 1.f;
	float traction_scale = ((mDriftFriction * mTractionBoost) * static_grip) * mLoad * pilot_factor;
	float tolerated_slip = mMaxSlip;
	if (!(wheel_force_length > traction_scale) || !(wheel_force_length > 0.001f))
	{
		if (use_ellipse)
			mLongitudeForce *= 0.66666669f;
	}
	else
	{
		float ratio = traction_scale / wheel_force_length;
		mTraction = ratio;
		mLateralForce *= ratio;
		mLongitudeForce *= ratio;
		tolerated_slip *= ratio * ratio;
	}
	float abs_slip_speed = slip_speed < 0.f ? -slip_speed : slip_speed;
	if (abs_slip_speed > tolerated_slip)
	{
		//float abs_slip_speed = slip_speed < 0.f ? -slip_speed : slip_speed;
		mTraction /= slip_speed < 0.f ? -slip_speed : slip_speed;
		mTraction *= tolerated_slip;
	}

	Attrib::Gen::simsurface::LayoutStruct* simsurface_data = mSurface.data;
	mLateralForce *= mSurface.data->LATERAL_GRIP;
	mLongitudeForce *= mSurface.data->DRIVE_GRIP;
	if (fwd_vel <= 1.f)
	{
		float abs_lat_slip = lat_vel;
		ABS(abs_lat_slip);
		if (abs_lat_slip >= 1.f)
			abs_lat_slip = 1.f;
		mLateralForce *= abs_lat_slip;
	}
	else
	{
		// inline asm to force the intrinsic since the compiler keeps calling sinf instead of using the intrinsic
		mLongitudeForce -= sinf(mSlipAngle * TWO_PI) * (mDragReduction / mSpecs->data->GRIP_SCALE.Pair[mAxleIndex]) * mLateralForce;
	}

	if (mBrakeLocked)
		mAngularAcc = 0.f;
	else
	{
		if (mTraction < 1.f)
		{
			float last_torque = (mBrakeTorque + mDriveTorque - mLongitudeForce * mRadius + mLastTorque) * 0.5f;
			mLastTorque = last_torque;
			mAngularAcc = (last_torque - RollingFriction * mSurface.data->ROLLING_RESISTANCE * mAV)
						/ WheelMomentOfInertia - mTraction * mSlip / (dT * mRadius);
		}
		mAngularAcc = (fwd_acc / mRadius - mAngularAcc) * mTraction + mAngularAcc;
	}
	mAV += dT * mAngularAcc;
	CheckSign();
	return mLateralForce;
}

static float LowSpeedSpeed = 0.f;
static float HighSpeedSpeed = 30.f;
static float MaxYawBonus = 0.35f;
static float LowSpeedYawBoost = 0.f;
static float HighSpeedYawBoost = 1.f;
static float YawEBrakeThreshold = 0.5f;
static float YawAngleThreshold = 20.f;
// MATCHING
/* float YawFrictionBoost(float yaw, float ebrake, float speed, float yawcontrol, float grade)
{
	float abs_grade = fabsf(grade) + 1.f;
	float abs_yaw = fabsf(yaw);
	if (ebrake > YawEBrakeThreshold && abs_yaw < YawAngleThreshold * DEG_TO_RAD)
		return abs_grade;
	
	float bonus = abs_yaw 
				* ((speed - LowSpeedSpeed) / (HighSpeedSpeed - LowSpeedSpeed)
				* (HighSpeedYawBoost - LowSpeedYawBoost) + LowSpeedYawBoost) * yawcontrol;
	if (bonus > MaxYawBonus)
		bonus = MaxYawBonus;
	return abs_grade + bonus;
} */
