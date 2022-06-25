#include "physics/behaviors/SuspensionRacer.h"

// interfaces
#include "interfaces/Simables/IRigidBody.h"
#include "interfaces/simables/isimable.h"
#include "interfaces/simables/itransmission.h"
#include "interfaces/simables/ivehicle.h"
#include "interfaces/simentities/iplayer.h"

#include "math/bmath.h"
#include "math/mathcommon.h"
#include "physics/physicsinfo.hpp"
#include "Sim/Simulation.h"

float _SimTime;
SPEED_NO_INLINE float Sim_GetTime() { return _SimTime; }
void ScaleVector(const UMath::Vector3& in, const float scale, UMath::Vector3& dest);

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

float ZeroDegreeTable[6]  = { 0.f };
float TwoDegreeTable[]    = { 0.f, 1.2f, 2.3f, 3.f, 3.f, 2.8f };
float FourDegreeTable[]   = { 0.f, 1.7f, 3.2f, 4.3f, 5.1f, 5.2f };
float SixDegreeTable[]    = { 0.f, 1.8f, 3.5f, 4.9f, 5.8f, 6.1f };
float EightDegreeTable[]  = { 0.f, 1.83f, 3.6f, 5.f, 5.96f, 6.4f };
float TenDegreeTable[]    = { 0.f, 1.86f, 3.7f, 5.1f, 6.13f, 6.7f };
float TwelveDegreeTable[] = { 0.f, 1.9f, 3.8f, 5.2f, 6.3f, 7.1f };
Table ZeroDegree   = Table(6, 0.f, 10.f, 0.5f, ZeroDegreeTable);
Table TwoDegree    = Table(6, 0.f, 10.f, 0.5f, TwoDegreeTable);
Table FourDegree   = Table(6, 0.f, 10.f, 0.5f, FourDegreeTable);
Table SixDegree    = Table(6, 0.f, 10.f, 0.5f, SixDegreeTable);
Table EightDegree  = Table(6, 0.f, 10.f, 0.5f, EightDegreeTable);
Table TenDegree    = Table(6, 0.f, 10.f, 0.5f, TenDegreeTable);
Table TwelveDegree = Table(6, 0.f, 10.f, 0.5f, TwelveDegreeTable);
Table* LoadSensitivityTable[] = 
{
	&ZeroDegree, &TwoDegree, &FourDegree, &SixDegree, &EightDegree, &TenDegree, &TwelveDegree
};
// Mystery solved, this and some other physics params are set at runtime by the game's DRM. Thanks for the info rx!
int* pLatForceMultipliers = NULL;
float SuspensionRacer::Tire::ComputeLateralForce(float load, float slip_angle)
{
	float angle = ANGLE2DEG(slip_angle);
	float norm_angle = angle * 0.5f;
	// there are 3 instructions here that are out of order but everything else matches
	// Black Box what did you do???????
	// why??????
	int slip_angle_table = (int)norm_angle;
	load *= 0.001f;
	load *= 0.8f;
	
	if (slip_angle_table >= 6)
	{
		float grip_scale = mSpecs->GRIP_SCALE().At(mAxleIndex);
		return (((LoadSensitivityTable[6]->GetValue(load) * pLatForceMultipliers[2]) * mGripBoost) * grip_scale) * 2500.f;
	}
	else
	{
		float low  = LoadSensitivityTable[slip_angle_table]->GetValue(load);
		float high = LoadSensitivityTable[slip_angle_table + 1]->GetValue(load);
		float grip_scale = mSpecs->GRIP_SCALE().At(mAxleIndex);
		float delta = norm_angle - slip_angle_table;
		return ((((delta * (high - low) + low)
			 * grip_scale) * pLatForceMultipliers[2])
			 * mGripBoost) * 2500.f;
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
	
	float speed_factor = (speed - MPH2MPS(30.00005f)) * 0.111849315f;
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

float BrakingTorque = 4.f;
float EBrakingTorque = 10.f;
float BrakeLockAngularVelocityFactor = 100.f;
float StaticToDynamicBrakeForceRatio = 1.2f;
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

float WheelMomentOfInertia = 10.f;
// MATCHING
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

// MATCHING
float SuspensionRacer::DoHumanSteering(Chassis::State& state)
{
	float input = state.steer_input;
	float prev_steering = mSteering.Previous;

	if (prev_steering >= 180.f)
		prev_steering -= 360.f;

	float steering_coeff = mTireInfo.STEERING();
	ISteeringWheel::SteeringType steer_type = ISteeringWheel::SteeringType::kGamePad;

	IPlayer* player = GetOwner()->GetPlayer();
	if (player)
	{
		int* steering_device = reinterpret_cast<int*>(player->GetSteeringDevice());

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
}

float BrakeSteeringRangeMultiplier = 1.45f;
float SteeringRangeData[]          = { 40.f, 20.f, 10.f, 5.5f, 4.5f, 3.25f, 2.9f, 2.9f, 2.9f, 2.9f };
float SteeringRangeCoeffData[]     = { 1.f, 1.f, 1.1f, 1.2f, 1.25f, 1.35f };
float SteeringSpeedData[]          = { 1.f, 1.f, 1.f, 0.56f, 0.5f, 0.35f, 0.3f, 0.3f, 0.3f, 0.3f };
float SteeringWheelRangeData[]     = { 45.f, 15.f, 11.f, 8.f, 7.f, 7.f, 7.f, 7.f, 7.f, 7.f };
Table SteeringRangeTable      = Table(10, 0.f, 160.f, 0.5625f, SteeringRangeData);
Table SteeringWheelRangeTable = Table(10, 0.f, 160.f, 0.5625f, SteeringWheelRangeData);
Table SteeringRangeCoeffTable = Table(6, 0.f, 1.f, 5.f, SteeringRangeCoeffData);
Table SteeringSpeedTable      = Table(10, 0.f, 160.f, 0.5625f, SteeringSpeedData);
UMath::Vector2 PostCollisionSteerReductionData[] = 
{
	UMath::Vector2(0.f, 0.2f),
	UMath::Vector2(0.2f, 0.5f),
	UMath::Vector2(0.5f, 0.7f),
	UMath::Vector2(0.7f, 1.f)
};
Graph PostCollisionSteerReductionTable(PostCollisionSteerReductionData, 4);
// NOT MATCHING
// the last part of the function is very stubborn, but it matches functionally at least
float SuspensionRacer::CalculateMaxSteering(Chassis::State& state, ISteeringWheel::SteeringType steer_type)
{
	const float MAX_STEERING_ANGLE = 45.f;
	const float steer_input = state.steer_input;

	// max possible steering output scales with the car's forward speed
	float max_steering = SteeringRangeTable.GetValue(state.local_vel.z);
	// there are 2 racing wheel input types, one scales with speed and the other doesn't
	switch (steer_type)
	{
		case ISteeringWheel::SteeringType::kWheelSpeedSensitive:
			max_steering = SteeringWheelRangeTable.GetValue(state.local_vel.z);
			break;
		case ISteeringWheel::SteeringType::kWheelSpeedInsensitive:
			return MAX_STEERING_ANGLE;
		default:
			break;
	}

	/* 
	 *  manipulate the max steering range based on the throttle, brake, and handbrake inputs
	 *  pressing only the throttle will give the regular steering range
	 *  throttle + a brake input will boost the steering range by 25% of BrakeSteeringRangeMultiplier
	 *  throttle + both brake inputs will boost the steering range by 50% of BrakeSteeringRangeMultiplier
	 *  a single brake input will boost the steering range by 75% of BrakeSteeringRangeMultiplier
	 *  both brake inputs will boost the steering range by 100% of BrakeSteeringRangeMultiplier
	 */
	float tbcoeff = 1.f - (state.gas_input + 1.f - (state.ebrake_input + state.brake_input) * 0.5f) * 0.5f;
	max_steering *= SteeringSpeedTable.GetValue(state.local_vel.z) * BrakeSteeringRangeMultiplier * tbcoeff + 1.f;
	max_steering *= SteeringRangeCoeffTable.GetValue(fabsf(mSteering.InputAverage.GetValue()));

	const Physics::Tunings* tunings = GetVehicle()->GetTunings();
	// steering tuning allows for up to a 20% increase or decrease to the max steering range
	if (tunings)
		max_steering *= tunings->steeringTuning * 0.2f + 1.f;

	// reduce steering range after a collision
	if (mSteering.CollisionTimer > 0.f)
	{
		// steering range reduction lasts for one second
		float secs = 1.f - mSteering.CollisionTimer;
		if (secs < 1.f && secs > 0.f)
		{
			float speed_coeff = UMath::Min(1.f, state.local_vel.z / (MPH2MPS(170.f) * 0.7f));
			float collision_coeff = PostCollisionSteerReductionTable.GetValue(secs);
			speed_coeff = (1.f - speed_coeff); // only matches with parentheses
			max_steering *= speed_coeff * (1.f - collision_coeff) + collision_coeff;
		}
	}

	// when steering, the max possible steering range can't go below the slip angle of the rear wheel on the inside of the turn
	float yaw_left  = ANGLE2DEG(GetWheel(2).mSlipAngle);
	float yaw_right = ANGLE2DEG(GetWheel(3).mSlipAngle);
	// steering right
	if (steer_input > 0.f && yaw_right > 0.f)
	{
		max_steering = bClamp(max_steering, yaw_right, MAX_STEERING_ANGLE);
	}
	// steering left
	else if (steer_input < 0.f && yaw_left < 0.f)
	{
		max_steering = bClamp(max_steering, -yaw_left, MAX_STEERING_ANGLE);
	}
	else if (bAbs(mSteering.InputAverage.GetValue()) >= 0.5f)
	{
		// NOTE: multiplying parts of equations by zero to nullify them is a recurring Black Box coding style quirk
		// I have no idea why they did it that way instead of just commenting them out

		// don't let the max steering range go below the previous frame's value
		max_steering = bMax(max_steering, mSteering.LastMaximum - state.time * 0.f);
	}

	// this is kinda pointless for the first 2 checks since they already clamp to MAX_STEERING_ANGLE
	max_steering = bMin(max_steering, MAX_STEERING_ANGLE);
	mSteering.LastMaximum = max_steering;
	return max_steering;
}

float SteeringInputSpeedData[] = { 1.f, 1.05f, 1.1f, 1.5f, 2.2f, 3.1f };
float SteeringInputData[]      = { 1.f, 1.05f, 1.1f, 1.2f, 1.3f, 1.4f };
Table SteeringInputSpeedCoeffTable = Table(6, 0.f, 10.f, 0.5f, SteeringInputSpeedData);
Table SteeringInputCoeffTable      = Table(6, 0.f, 1.f, 5.f, SteeringInputData);
// NOT MATCHING
// seems like it really wants that volatile in order to fully match (it's still functionally matching though)
// <@>PRINT_ASM
float SuspensionRacer::CalculateSteeringSpeed(Chassis::State& state)
{
	// get a rough approximation of how fast the player is steering
	// this ends up creating a bit of a difference in how fast you can actually steer on a controller under normal circumstances
	// using a keyboard will always give you the fastest steering possible
	volatile float steer_input_speed = (state.steer_input - mSteering.LastInput) / state.time;

	mSteering.InputSpeedCoeffAverage.Record(SteeringInputSpeedCoeffTable.GetValue(bAbs(steer_input_speed)), Sim::GetTime());

	// steering speed scales with vehicle forward speed
	float steer_speed = (SteeringSpeedTable.GetValue(state.local_vel.z) * 180.f) * mSteering.InputSpeedCoeffAverage.fAverage;
	float steer_input = bAbs(mSteering.InputAverage.GetValue());
	return SteeringInputCoeffTable.GetValue(steer_input) * steer_speed;
}

// MATCHING
float SuspensionRacer::DoAISteering(Chassis::State& state)
{
	mSteering.Maximum = 45.f;
	if (state.driver_style != STYLE_DRAG)
		mSteering.Maximum = mTireInfo.STEERING() * 45.f;

	return DEG2ANGLE(state.steer_input * mSteering.Maximum);
}

// MATCHING
void SuspensionRacer::DoSteering(Chassis::State& state, UMath::Vector3& right, UMath::Vector3& left)
{
	float truesteer;
	UMath::Vector4 r4;
	UMath::Vector4 l4;

	// I don't know exactly what this vfunction call is,
	// but it's safe to assume it's checking to see if the current vehicle is player controlled
	if (mHumanAI && (*(bool (__fastcall **)(int*))((*(int*)mHumanAI) + 0x8))((int*)mHumanAI))
		truesteer = DoHumanSteering(state);
	else
		truesteer = DoAISteering(state);
	
	ComputeAckerman(truesteer, state, l4, r4);
	right = Vector4To3(r4);
	left  = Vector4To3(l4);
	mSteering.Wheels[0] = l4.w;
	mSteering.Wheels[1] = r4.w;
	DoWallSteer(state);
}

float BurnOutCancelSlipValue = 0.5f;
float BurnOutYawCancel = 0.5f;
float BurnOutAllowTime = 1.f;
float BurnOutMaxSpeed = 20.f;
float BurnOutFishTailTime = 2.f;
int BurnOutFishTails = 6;
UMath::Vector2 BurnoutFrictionData[] = 
{
	UMath::Vector2(0.f, 1.f),
	UMath::Vector2(5.f, 0.8f),
	UMath::Vector2(9.f, 0.9f),
	UMath::Vector2(12.6f, 0.833f),
	UMath::Vector2(17.1f, 0.72f),
	UMath::Vector2(25.f, 0.65f)
};
tGraph<float> BurnoutFrictionTable(BurnoutFrictionData, 6);
// MATCHING
void SuspensionRacer::Burnout::Update(const float dT, const float speedmph, const float max_slip, const int max_slip_wheel, const float yaw)
{
	// continue burnout/fishtailing state and end when certain conditions are met
	if (GetState())
	{
		if (speedmph > 5.f && UMath::Abs(yaw * TWO_PI) > BurnOutYawCancel)
		{
			Reset();
		}
		else if (max_slip < BurnOutCancelSlipValue)
		{
			IncBurnOutAllow(dT);
			if (mBurnOutAllow > BurnOutAllowTime)
				Reset();
		}
		else
		{
			ClearBurnOutAllow();
			DecBurnOutTime(dT);
			if (mBurnOutTime < 0.f)
			{
				SetState(mState - 1);
				SetBurnOutTime(BurnOutFishTailTime);
			}
		}
	}
	// initialize burnout/fishtailing state
	else if (speedmph < BurnOutMaxSpeed && max_slip > 0.5f)
	{
		float burnout_coeff;
		BurnoutFrictionTable.GetValue(burnout_coeff, max_slip);
		SetTraction(burnout_coeff / 1.4f);
		// burnout state changes according to what side of the axle the wheel that's slipping the most is on
		SetState((int)((1.5f - burnout_coeff) * BurnOutFishTails + (max_slip_wheel & 1)));
		SetBurnOutTime(BurnOutFishTailTime);
		ClearBurnOutAllow();
	}
}

// MATCHING
// Calculates artificial steering for when the car is touching a wall
void SuspensionRacer::DoWallSteer(Chassis::State& state)
{
	float wall = mSteering.WallNoseTurn;
	// nose turn is applied when the car is perpendicular to the wall
	// allows the player to easily turn their car away from the wall after a head-on crash without reversing
	if (wall != 0.f && mNumWheelsOnGround > 2 && state.gas_input > 0.f)
	{
		float dW = state.steer_input * state.gas_input * 0.125f;
		if (wall * dW < 0.f)
			return;
		UMath::Vector3 chg(0.f, UMath::Abs(wall) * dW, 0.f);
		UMath::Rotate(chg, state.matrix, chg);
		chg += state.angular_vel;
		mRB->SetAngularVelocity(chg);
	}

	wall = mSteering.WallSideTurn;
	float dW = state.steer_input * state.gas_input;
	// side turn is only applied when in reverse and if touching a wall parallel to the car
	// it helps the player move their car away from the wall when backing up
	if (dW * wall > 0.f && mNumWheelsOnGround > 2 && !state.gear)
	{
		dW *= -0.125f;
		UMath::Vector3 chg(0.f, UMath::Abs(wall) * dW, 0.f);
		UMath::Rotate(chg, state.matrix, chg);
		chg += state.angular_vel;
		mRB->SetAngularVelocity(chg);
	}
}

// NOT MATCHING
// stack frame is off in this function for some reason and I can't get it to match up
// it's also supposed to move ecx into esi but it's not doing that either
// everything else matches but it won't actually line up till I can get past the stack memes
void SuspensionRacer::Differential::CalcSplit(bool locked)
{
	if (has_traction[0] && has_traction[1] && !locked && !(factor <= 0.f))
	{
		float inv_bias = 1.f - bias;
		float av_0 = angular_vel[0] * (1.f - bias);
		float av_1 = angular_vel[1] * bias;
		float combined_av = UMath::Abs(av_0 + av_1);

		if (combined_av > FLT_EPSILON)
		{
			float inv_av = 1.f / combined_av;
			float inv_factor = 1.f - factor;
			torque_split[0] = ((UMath::Abs(av_1) * inv_av) * factor) + (inv_factor * bias);
			torque_split[1] = ((UMath::Abs(av_0) * inv_av) * factor) + (inv_factor * (1.f - bias));
		}
		else
		{
			torque_split[0] = bias;
			torque_split[1] = 1.f - bias;
		}

		torque_split[0] = UMath::Clamp(torque_split[0], 0.f, 1.f);
		torque_split[1] = UMath::Clamp(torque_split[1], 0.f, 1.f);
	}
	else
	{
		torque_split[0] = bias;
		torque_split[1] = 1.f - bias;
	}
}

// NOT MATCHING
void SuspensionRacer::DoDriveForces(Chassis::State& state)
{
	if (mTransmission)
	{
		float drive_torque = mTransmission->GetDriveTorque();
		if (drive_torque != 0.f)
		{
			SuspensionRacer::Differential center_diff;
			center_diff.factor = mTransInfo.DIFFERENTIAL(2);
			if (center_diff.factor > 0.f)
			{
				center_diff.bias = mTransInfo.TORQUE_SPLIT();
				center_diff.angular_vel[0] = mTires[0]->GetAngularVelocity() + mTires[1]->GetAngularVelocity();
				center_diff.angular_vel[1] = mTires[2]->GetAngularVelocity() + mTires[3]->GetAngularVelocity();

				if (mTires[0]->IsOnGround() || (center_diff.has_traction[0] = 0, mTires[1]->IsOnGround()))
					center_diff.has_traction[0] = 1;
				if (mTires[2]->IsOnGround() || (center_diff.has_traction[1] = 0, mTires[3]->IsOnGround()))
					center_diff.has_traction[1] = 1;
				center_diff.CalcSplit(false);
			}
			else
			{
				center_diff.torque_split[0] = mTransInfo.TORQUE_SPLIT();
				center_diff.torque_split[1] = 1.f - center_diff.torque_split[0];
			}

			uint32_t axle = 0;
			// loop through 2 wheels at a time so we can easily deal with both wheels on the axle
			for (SuspensionRacer::Tire** tire = mTires; axle < 2; tire += 2)
			{
				float axle_torque = drive_torque * center_diff.torque_split[axle];
				if (UMath::Abs(axle_torque) > FLT_EPSILON)
				{
					SuspensionRacer::Differential diff;
					diff.bias = 0.5f;
					float traction_control[2] = { 1.f, 1.f };
					float traction_boost[2]   = { 1.f, 1.f };
					diff.factor = mTransInfo.DIFFERENTIAL(axle);
					diff.angular_vel[0] = tire[0]->GetAngularVelocity();
					diff.has_traction[0] = tire[0]->IsOnGround();

					float fwd_slip = tire[0]->mSlip * center_diff.torque_split[axle] * 0.5f;
					float lat_slip = tire[0]->mLateralSpeed * center_diff.torque_split[axle];

					diff.angular_vel[1] = tire[1]->GetAngularVelocity();
					lat_slip *= 0.5f;
					diff.has_traction[1] = tire[1]->IsOnGround();
					
					bool locked_diff = false;
					fwd_slip += tire[1]->mSlip * center_diff.torque_split[axle] * 0.5f;
					lat_slip += tire[1]->mLateralSpeed * center_diff.torque_split[axle] * 0.5f;

					if ((mBurnOut.GetState() & 1) != 0)
					{
						traction_boost[1] = mBurnOut.GetTraction();
						diff.bias = mBurnOut.GetTraction() * 0.5f;
						locked_diff = true;
					}
					else if ((mBurnOut.GetState() & 2) != 0)
					{
						traction_boost[0] = mBurnOut.GetTraction();
						diff.bias = 1.f - mBurnOut.GetTraction() * 0.5f;
						locked_diff = true;
					}
					else
					{
						float delta_lat_slip = lat_slip - state.local_vel.x;
						if (delta_lat_slip * state.steer_input > 0.f && fwd_slip * axle_torque > 0.f)
						{
							float delta_fwd_slip = fwd_slip - state.local_vel.z;
							float traction_control_limit = UMath::Ramp(delta_fwd_slip, 1.f, 20.f);

							if (traction_control_limit > 0.f)
							{
								float traction_angle = Atan2d(delta_lat_slip, UMath::Abs(delta_fwd_slip)) * state.steer_input;
								traction_angle = UMath::Abs(traction_angle);
								traction_control_limit *= UMath::Ramp(traction_angle, 1.f, 16.f);
								if (traction_control_limit > 0.f)
								{
									if (delta_lat_slip > 0.f)
									{
										traction_control[1] = 1.f - traction_control_limit;
										traction_control[0] = 1.f - traction_control_limit * 0.5f;
										traction_boost[0] = traction_control_limit + 1.f;
									}
									else
									{
										traction_control[0] = 1.f - traction_control_limit;
										traction_control[1] = 1.f - traction_control_limit * 0.5f;
										traction_boost[1] = traction_control_limit + 1.f;
									}
								}
							}
						}
					}
					diff.CalcSplit(locked_diff);

					if (tire[0]->IsOnGround())
					{
						if (!tire[0]->mBrakeLocked)
							tire[0]->mDriveTorque += diff.torque_split[0] * traction_control[0] * axle_torque;
						tire[0]->mTractionBoost *= traction_boost[0];
					}
					if (tire[1]->IsOnGround())
					{
						if (!tire[1]->mBrakeLocked)
							tire[1]->mDriveTorque += diff.torque_split[1] * traction_control[1] * axle_torque;
						tire[1]->mTractionBoost *= traction_boost[1];
					}
				}
				++axle;
			}
		}
	}
}

// NOT MATCHING
float RollingFriction = 2.f;
float TireForceEllipseRatio = 1.5f;
float SuspensionRacer::Tire::UpdateLoaded(float lat_vel, float fwd_vel, float body_speed, float load, float dT)
{
	float bt = (mBrakes->data->BRAKES[mAxleIndex] * 1.3558f) * BrakingTorque;
	float ebt = (mBrakes->data->EBRAKE * 1.3558f) * EBrakingTorque;
	const float dynamicgrip_spec = mSpecs->DYNAMIC_GRIP().Pair[mAxleIndex];
	const float staticgrip_spec = mSpecs->STATIC_GRIP().Pair[mAxleIndex];

	if (mLoad <= 0.f && !mBrakeLocked)
		mAV = fwd_vel / mRadius;
	float fwd_acc = (fwd_vel - mRoadSpeed) / dT;
	mRoadSpeed = fwd_vel;

	//float temp_load = UMath::Max(load, 0.f);
	mLoad = UMath::Max(load, 0.f);
	mLateralSpeed = lat_vel;
	//bt *= mBrake;
	//ebt *= mEBrake;
	float bt_add  = bt * mBrake;
	float ebt_add = ebt * mEBrake;
	float abs_fwd = UMath::Abs(fwd_vel);
	if (abs_fwd < 1.f)
	{
		// when car is nearly stopped, apply brake torque using forward velocity and wheel load
		float inv_radius = 1.f / mRadius;
		float stopping_bt    = -((inv_radius * mBrake)  * fwd_vel * load);
		float stopping_ebt   = -((inv_radius * mEBrake) * fwd_vel * load);
		float r_drive_torque = mEBrake;
		r_drive_torque *= -mDriveTorque;
		if (!mBrakeLocked)
			mDriveTorque += r_drive_torque;
		if (!mBrakeLocked)
			mBrakeTorque += stopping_bt;
		if (!mBrakeLocked)
			mBrakeTorque += stopping_ebt;
	}
	else
	{
		float opp_bt = mAV > 0.f ? -bt_add : bt_add;
		if (!mBrakeLocked)
			mBrakeTorque += opp_bt;
		float opp_ebt = mAV > 0.f ? -ebt_add : ebt_add;
		if (!mBrakeLocked)
			mBrakeTorque += opp_ebt;
	}
	
	mSlipAngle = VU0_Atan2(lat_vel, abs_fwd);
	//float slip_speed = mAV;
	float slip_speed = mAV * mRadius - fwd_vel;
	float slip_ground_friction = 0.f;
	float dynamic_friction = 1.f;
	mSlip = slip_speed;
	float skid_speed = Sqrt(slip_speed * slip_speed + lat_vel * lat_vel);
	float pilot_factor = GetPilotFactor(body_speed);
	if (skid_speed > FLT_EPSILON && (lat_vel != 0.f || fwd_vel != 0.f))
	{
		dynamic_friction = (mTractionBoost * pilot_factor) * dynamicgrip_spec;
		slip_ground_friction = (dynamic_friction / skid_speed) * mLoad;
		float vel_len = Sqrt(fwd_vel * fwd_vel + lat_vel * lat_vel);
		//float wheel_speed = mLoad / Sqrt(fwd_vel * fwd_vel + lat_vel * lat_vel);
		//float ground_speed = ((mLoad / vel_len) * dynamic_friction) * abs_fwd;
		CheckForBrakeLock(((mLoad / vel_len) * dynamic_friction) * abs_fwd);
	}

	if (mTraction < 1.f || mBrakeLocked)
	{
		//float long_force;// = slip_speed * slip_ground_friction;
		mLongitudeForce = slip_ground_friction;
		mLongitudeForce *= slip_speed;
		mLateralForce = -(slip_ground_friction * lat_vel);

		// 0.44703 mps = 1 mph
		const float one_mph = 0.44703f;
		if (body_speed < one_mph && dynamic_friction > 0.1f)
		{
			mLateralForce /= dynamic_friction;
			mLongitudeForce /= dynamic_friction;
		}
		mLongitudeForce = UMath::Limit(mLongitudeForce, GetTotalTorque() / mRadius);
	}
	else
	{
		mBrakeLocked = false;
		mLongitudeForce = GetTotalTorque() / mRadius;
		float slip_ang = mSlipAngle;
		//float temp_load = mLoad;
		float lat_force = ComputeLateralForce(mLoad, UMath::Abs(slip_ang));// < 0.f ? -slip_ang : slip_ang);
		mLateralForce = lat_force;
		if (lat_vel > 0.f)
			mLateralForce = -lat_force;
	}

	mLateralForce *= mLateralBoost;
	if (mTraction >= 1.f && !mBrakeLocked)
		mLongitudeForce += (mAngularAcc * mRadius - fwd_acc) / mRadius * WheelMomentOfInertia;
	
	float combined_torque = GetTotalTorque();
	bool use_ellipse = false;
	//combined_torque *= fwd_vel;
	if (combined_torque * fwd_vel > 0.f && !mBrakeLocked)
	{
		use_ellipse = true;
		mLongitudeForce *= TireForceEllipseRatio;
	}

	//float ellipse_x = mLateralForce * mTractionCircle.x;
	mLateralForce *= mTractionCircle.x;
	//float ellipse_y = mLongitudeForce * mTractionCircle.y;
	mLongitudeForce *= mTractionCircle.y;
	//float len_force = Sqrt(mLongitudeForce * mLongitudeForce + mLateralForce * mLateralForce);
	float len_force = mLongitudeForce * mLongitudeForce + mLateralForce * mLateralForce;
	len_force = Sqrt(len_force);
	mTraction = 1.f;
	float traction_scale = ((mDriftFriction * mTractionBoost) * staticgrip_spec) * mLoad * pilot_factor;
	float tolerated_slip = mMaxSlip;
	//if (!(len_force > traction_scale) || !(len_force > 0.001f))
	if (len_force > traction_scale && len_force > 0.001f)
	{
		float ratio = traction_scale / len_force;
		mTraction = ratio;
		mLateralForce *= ratio;
		mLongitudeForce *= ratio;
		tolerated_slip = (ratio * ratio) * tolerated_slip;
	}
	else if (use_ellipse)
	{
		mLongitudeForce /= TireForceEllipseRatio;
	}

	if (UMath::Abs(slip_speed) > tolerated_slip)
	{
		mTraction /= UMath::Abs(slip_speed);
		mTraction *= tolerated_slip;
	}

	// factor surface friction into the tire force
	Attrib::Gen::simsurface::LayoutStruct* simsurface_data = mSurface.data;
	mLateralForce *= mSurface.data->LATERAL_GRIP;
	mLongitudeForce *= mSurface.data->DRIVE_GRIP;
	if (fwd_vel > 1.f)
		mLongitudeForce -= sinf(mSlipAngle * TWO_PI) * (mDragReduction / mSpecs->GRIP_SCALE().At(mAxleIndex)) * mLateralForce;
	else
	{
		float abs_lat_slip = UMath::Min(UMath::Abs(lat_vel), 1.f);
		mLateralForce *= abs_lat_slip;
	}

	if (mBrakeLocked)
		mAngularAcc = 0.f;
	else
	{
		if (mTraction < 1.f)
		{
			float long_force = mLongitudeForce;
			float last_torque = (GetTotalTorque() - mLongitudeForce * mRadius + mLastTorque) * 0.5f;
			mLastTorque = last_torque;
			mAngularAcc = (last_torque - (RollingFriction * mSurface.data->ROLLING_RESISTANCE) * mAV)
						/ WheelMomentOfInertia - (mTraction * mSlip) / (dT * mRadius);
		}
		// pointless temp var so that this can match
		float ang_acc = mAngularAcc;
		mAngularAcc = ((fwd_acc / mRadius) - ang_acc) * mTraction + ang_acc;
	}
	mAV += dT * mAngularAcc;
	CheckSign();
	return mLateralForce;
}

// MATCHING
float SuspensionRacer::CalcYawControlLimit(float speed)
{
	if (mTransmission)
	{
		float maxspeed = mTransmission->GetMaxSpeedometer();
		if (maxspeed <= 0.f)
			return 0.f;
		float percent = UMath::Min(UMath::Abs(speed) / maxspeed, 1.f);
		uint32_t numunits = mTireInfo.Num_YAW_CONTROL();
		if (numunits > 1)
		{
			float ratio = (numunits - 1) * percent;
			uint32_t index1 = ratio;
			float delta = ratio - index1;
			uint32_t index2 = UMath::Min(index1 + 1, numunits - 1);
			float a = mTireInfo.YAW_CONTROL(index1);
			float b = mTireInfo.YAW_CONTROL(index2);
			return (b - a) * delta + a;
		}
	}
	return mTireInfo.YAW_CONTROL(0);
}

float DriftRearFrictionData[] = { 1.1f, 0.95f, 0.87f, 0.77f, 0.67f, 0.6f, 0.51f, 0.43f, 0.37f, 0.34f };
Table DriftRearFrictionTable(10, 0.f, 1.f, 9.f, DriftRearFrictionData);
UMath::Vector2 DriftStabilizerData[] = 
{
	UMath::Vector2(0.f, 0.f),
	UMath::Vector2(0.2617994f, 0.1f),
	UMath::Vector2(0.52359879f, 0.45f),
	UMath::Vector2(0.78539819f, 0.85f),
	UMath::Vector2(1.0471976f, 0.95f),
	UMath::Vector2(1.5533431f, 1.15f),
	UMath::Vector2(1.5707964f, 0.f)
};
tGraph<float> DriftStabilizerTable(DriftStabilizerData, 7);
// NOT MATCHING
// fucking auto inlines man
void SuspensionRacer::DoDrifting(const Chassis::State& state)
{
	if (mDrift.State && ((state.flags & 1) || state.driver_style == STYLE_DRAG))
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
		// the drift value will decrement by (dT * 2) when not drifting or exiting a drift
			drift_change = -2.f;
			break;
		case SuspensionRacer::Drift::eState::D_ENTER:
		case SuspensionRacer::Drift::eState::D_IN:
		// the drift value will increment by (dT * 8) when entering and holding a drift
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
		float avg_steer = mSteering.InputAverage.fAverage;
		if ((state.local_angular_vel.y * state.slipangle) < 0.f 
		&& UMath::Abs(state.slipangle) <= 0.25f && state.speed > MPH2MPS(30.00005f)
		&& (avg_steer * state.slipangle) <= 0.f && UMath::Abs(state.slipangle) > DEG2ANGLE(12.f))
		{
			mDrift.State = SuspensionRacer::Drift::eState::D_IN;
		}
		else if ((state.steer_input * state.slipangle) * state.gas_input > DEG2ANGLE(12.f) && state.speed > MPH2MPS(30.00005f))
		{
			mDrift.State = SuspensionRacer::Drift::eState::D_IN;
		}
		else if (!((UMath::Abs(state.slipangle) * state.gas_input) > DEG2ANGLE(12.f)))
		{
			mDrift.State = SuspensionRacer::Drift::eState::D_EXIT;
		}
		else
		{
			mDrift.State = SuspensionRacer::Drift::eState::D_ENTER;
		}
	}
	else if (state.speed > MPH2MPS(30.00005f) && (state.ebrake_input > 0.5f || UMath::Abs(state.slipangle) > DEG2ANGLE(12.f)))
	{
		mDrift.State = SuspensionRacer::Drift::eState::D_ENTER;
	}

	if (!(mDrift.Value <= 0.f))
	{
		float yaw = state.local_angular_vel.y;
		// chassis slip angle is stored as a value in the range [-1,1]
		// so multiplying by 2pi gives the entire possible angle range in radians
		float slipangle_radians = state.slipangle * TWO_PI;

		// charge speedbreaker if not in use and drifting is detected
		if (mGameBreaker <= 0.f 
		&& state.speed > MPH2MPS(35.000058f) 
		&& UMath::Abs(slipangle_radians) > DEG2RAD(30.f))//0.52358997f)
		{
			IPlayer* player = GetOwner()->GetPlayer();
			if (player)
			{
				float charge = mDrift.Value * state.time * 0.5f;
				player->ChargeGameBreaker(charge);
			}
		}

		// apply yaw damping torque
		if ((yaw * slipangle_radians) < 0.f && mNumWheelsOnGround >= 2)
		{
			float damping;
			DriftStabilizerTable.GetValue(damping, UMath::Abs(slipangle_radians));
			float yaw_coef = state.inertia.y * mDrift.Value * damping * yaw * -4.f;
			UMath::Vector3 moment;
			// multiply up vector by yaw coefficient to get the final amount of damping to apply
			ScaleVector(UMath::Vector4To3(state.matrix.v1), yaw_coef, moment);
			mRB->ResolveTorque(moment);
		}

		// detect counter steering
		float countersteer = 0.f;
		if ((slipangle_radians * state.steer_input) > 0.f)
			countersteer = UMath::Abs(state.steer_input);
			
		float abs_slipangle = UMath::Abs(slipangle_radians);
		float abs_yaw = UMath::Abs(yaw);
		float driftmult_rear = DriftRearFrictionTable.GetValue(((abs_yaw + abs_slipangle) * 0.5f + countersteer * 4.f) * mDrift.Value);
		mTires[2]->mDriftFriction = driftmult_rear;
		mTires[3]->mDriftFriction = driftmult_rear;
	}
}

float EBrakeYawControlMin = 0.5f;
float EBrakeYawControlOnSpeed = 1.f;
float EBrakeYawControlOffSpeed = 20.f;
float EBrake180Yaw = 0.3f;
float EBrake180Speed = 80.f;
// NOT MATCHING
// just some stack memes but all the code matches
// there's probably an extra variable that needs to be added somewhere to resolve the stack differences
void SuspensionRacer::TuneWheelParams(Chassis::State& state)
{
	float ebrake = state.ebrake_input;
	float t = state.time;
	float speedmph = MPS2MPH(state.local_vel.z);
	float car_yaw = ANGLE2RAD(state.slipangle);
	float yawcontrol = mSteering.YawControl;
	
	// engaging the handbrake decreases steering yaw control 
	if (ebrake >= 0.5f)
	{
		yawcontrol -= t * EBrakeYawControlOffSpeed;
		if (yawcontrol < EBrakeYawControlMin)
			yawcontrol = EBrakeYawControlMin;
	}
	else
	{
		yawcontrol += t * EBrakeYawControlOnSpeed;
		if (yawcontrol > 1.f)
			yawcontrol = 1.f;
	}
	mSteering.YawControl = yawcontrol;

	float brake_biased[2] = { state.brake_input, state.brake_input };
	yawcontrol *= (1.f - mDrift.Value); // pointless parentheses for matching purposes
	const Physics::Tunings* tunings = GetVehicle()->GetTunings();
	if (tunings)
	{
		// brake tuning adjusts the brake bias
		brake_biased[0] *= tunings->brakesTuning * 0.5f + 1.f;
		brake_biased[1] -= brake_biased[1] * tunings->brakesTuning * 0.5f;
	}
	float suspension_yaw_control_limit = CalcYawControlLimit(state.speed);
	IPlayer* player = GetOwner()->GetPlayer();
	if (state.driver_style == STYLE_DRAG)
		suspension_yaw_control_limit = 0.1f;
	else if (player)
	{
		PlayerSettings* settings = player->GetSettings();
		if (settings)
		{
			// increase yaw control limit when stability control is off (unused by normal means)
			if (!settings->Handling)
				suspension_yaw_control_limit += 2.5f;
		}
	}

	float max_slip = 0.f;
	int max_slip_wheel = 0;
	for (int i = 0; i < 4; ++i)
	{
		float lateral_boost = 1.f;

		// at speeds below 10 mph, 5% of the current speed in mph is applied as the brake scale for driven wheels
		if (state.gas_input > 0.8f && state.brake_input > 0.5f && UMath::Abs(speedmph) < 10.f && IsDriveWheel(i))
			mTires[i]->mBrake = UMath::Abs(speedmph) * 0.05f;
		else
			mTires[i]->mBrake = brake_biased[i >> 1];

		// handbrake only applies to the rear wheels
		if (IsRear(i))
		{
			float ebrake_boost = ebrake;
			// increase handbrake multiplier when a hard handbrake turn is detected
			if (ebrake > 0.2f && car_yaw > EBrake180Yaw && speedmph < EBrake180Speed)
				ebrake_boost += 0.5f;
			mTires[i]->mEBrake = ebrake_boost;
		}
		else
			mTires[i]->mEBrake = 0.f;

		float friction_boost = 1.f;
		// rear wheels get extra boost according to the yaw control
		if (IsRear(i))
		{
			float grade = state.GetForwardVector().y;
			float boost = YawFrictionBoost(car_yaw, mTires[i]->mEBrake, state.speed, suspension_yaw_control_limit, grade) - 1.f;
			friction_boost = boost * yawcontrol + 1.f;
		}

		// speedbreaker increases front tire friction relative to the absolute steering input
		if (mGameBreaker > 0.f && IsFront(i))
		{
			float gamebreaker_friction_scale = UMath::Abs(state.steer_input) * mGameBreaker * 0.75f + 1.f;
			lateral_boost = gamebreaker_friction_scale;
			friction_boost *= gamebreaker_friction_scale;
		}
		mTires[i]->mTractionBoost *= friction_boost;
		mTires[i]->mLateralBoost = lateral_boost;

		if (tunings)
		{
			float ellipse_mul = tunings->handlingTuning * 0.2f;
			UMath::Vector2 circle(ellipse_mul + 1.f, 1.f - ellipse_mul);
			mTires[i]->SetTractionCircle(circle);
		}
		// traction is increased by perfect shifts in drag races and also by engaging the nitrous
		float over_boost = state.shift_boost; // pointless temp var just for matching
		mTires[i]->mTractionBoost *= (over_boost * state.nos_boost); // these pointless parentheses are seriously what makes it match

		// popped tires are permanently braking and have reduced traction
		if ((1 << i) & state.blown_tires)
		{
			mTires[i]->mEBrake = 0.f;
			mTires[i]->mBrake = 1.f;
			mTires[i]->mTractionBoost *= 0.3f;
		}

		// find the highest slip of all tires for the burnout/fishtailing state
		if (mTires[i]->mSlip > max_slip)
		{
			max_slip = mTires[i]->mSlip;
			max_slip_wheel = i;
		}
	}

	// burnout state only applies when in first gear and the throttle is fully pressed outside of drag events
	if (state.driver_style != STYLE_DRAG && state.gear == G_FIRST && state.gas_input >= 1.f)
		mBurnOut.Update(state.time, MPS2MPH(state.local_vel.z), max_slip, max_slip_wheel, state.slipangle);
	else
		mBurnOut.Reset();

	// lower traction for all wheels when staging
	if (state.flags & 1)
	{
		// this was likely a loop that MSVC unrolled in the original code; it generates the same asm as this would:
		// mTires[0]->ScaleTractionBoost(0.25f);
		// mTires[1]->ScaleTractionBoost(0.25f);
		// mTires[2]->ScaleTractionBoost(0.25f);
		// mTires[3]->ScaleTractionBoost(0.25f);
		for (int i = 0; i < 4; ++i)
		{
			mTires[i]->ScaleTractionBoost(0.25f);
		}
	}

	DoDrifting(state);
}

float ENABLE_ROLL_STOPS_THRESHOLD = 0.2f;
void SuspensionRacer::DoWheelForces(Chassis::State& state)
{
	const float dT = state.time;

	UMath::Vector3 steerR(0.f);
	UMath::Vector3 steerL(0.f);
	DoSteering(state, steerR, steerL);
	TuneWheelParams(state);

	UMath::Vector4 vUp;
	uint32_t wheelsOnGround = 0;
	float maxDelta = 0.f;
	const float mass = state.mass;
	float ride_extra = 0.f;

	const Physics::Tunings* tunings = GetVehicle()->GetTunings();
	if (tunings)
		ride_extra = tunings->rideHeightTuning;
	
	float time = Sim_GetTime();
	float shock_specs[2];
	float spring_specs[2];
	float sway_specs[2];
	float travel_specs[2];
	float rideheight_specs[2];
	float shock_ext_specs[2];
	float shock_valving[2];
	float shock_digression[2];
	float progression[2];
	// the compiler unrolls this loop
	// how fun
	for (int i = 0; i < 2; ++i)
	{
		shock_specs[i]      = LBIN2NM(mSuspensionInfo.SHOCK_STIFFNESS().At(i));
		shock_ext_specs[i]  = LBIN2NM(mSuspensionInfo.SHOCK_EXT_STIFFNESS().At(i));
		shock_valving[i]    = INCH2METERS(mSuspensionInfo.SHOCK_VALVING().At(i));
		shock_digression[i] = 1.f - mSuspensionInfo.SHOCK_DIGRESSION().At(i);
		spring_specs[i]     = LBIN2NM(mSuspensionInfo.SPRING_STIFFNESS().At(i));
		sway_specs[i]       = LBIN2NM(mSuspensionInfo.SWAYBAR_STIFFNESS().At(i));
		travel_specs[i]     = INCH2METERS(mSuspensionInfo.TRAVEL().At(i));
		rideheight_specs[i] = INCH2METERS(mSuspensionInfo.RIDE_HEIGHT().At(i)) + INCH2METERS(ride_extra);
		progression[i]      = mSuspensionInfo.SPRING_PROGRESSION().At(i);
	}
	
	UMath::Vector4 vFwd;
	float sway_stiffness[4];
	UMath::Vector4 steering_normals[4];
	sway_stiffness[0] = (mTires[0]->mCompression - mTires[1]->mCompression) * sway_specs[0];
	sway_stiffness[1] = -sway_stiffness[0];
	float delta = (mTires[2]->mCompression - mTires[3]->mCompression);
	vFwd = steerL;
	vFwd.w = 1.f;
	bool resolve = false;
	sway_stiffness[2] = delta * sway_specs[1];
	sway_stiffness[3] = -sway_stiffness[2];

	//steering_normals[0].w = 1.f;
	steering_normals[0] = vFwd;

	vFwd = steerR;
	vFwd.w = 1.f;
	steering_normals[1] = vFwd;

	vFwd = state.GetForwardVector();
	vFwd.w = 1.f;
	steering_normals[2] = vFwd;

	vFwd = state.GetForwardVector();
	vFwd.w = 1.f;
	steering_normals[3] = vFwd;

	for (uint32_t i = 0; i < 4; ++i)
	{
		int axle = i >> 1;
		Tire& wheel = GetWheel(i);
		wheel.UpdatePosition(state.angular_vel, state.linear_vel, state.matrix, state.world_cog, state.time, wheel.mRadius, true, state.collider, state.dimension.y * 2.f);
		const UMath::Vector3 groundNormal(UMath::Vector4To3(wheel.mNormal));
		const UMath::Vector3 forwardNormal(UMath::Vector4To3(steering_normals[i]));
		UMath::Vector3 lateralNormal;
		UMath::UnitCross(groundNormal, forwardNormal, lateralNormal);

		float penetration = wheel.mNormal.w;
		// how angled the wheel is relative to the ground
		float upness = UMath::Clamp_(UMath::Dot(groundNormal, state.GetUpVector()), 0.f, 1.f);
		const float oldCompression = wheel.mCompression;
		float newCompression = upness * rideheight_specs[axle] + penetration;
		float max_compression = travel_specs[axle];
		if (oldCompression == 0.f)
		{
			float delta = newCompression - max_compression;
			//maxDelta = UMath::Max(maxDelta, delta);
			if (maxDelta > delta)
				delta = maxDelta;
			maxDelta = delta;
		}
		float clamped_comp;
		//newCompression = UMath::Max(newCompression, 0.f);
		if (newCompression > 0.f)
			clamped_comp = newCompression;
		else
			clamped_comp = 0.f;
		newCompression = clamped_comp;

		// handle the suspension bottoming out
		if (newCompression > max_compression)
		{
			float delta = newCompression - max_compression;
			//maxDelta = UMath::Max(maxDelta, delta);
			if (maxDelta > delta)
				delta = maxDelta;
			maxDelta = delta;
			// suspension can't compress past the max travel length, so clamp it here
			newCompression = max_compression;
			wheel.mBottomOutTime = time;
		}

		if (newCompression > 0.f && upness > ENABLE_ROLL_STOPS_THRESHOLD)
		{
			++wheelsOnGround;
			const float diff = (newCompression - wheel.mCompression);
			float rise = diff / dT;
			float spring = (newCompression * progression[axle] + 1.f) * spring_specs[axle] * newCompression;
			if (shock_valving[axle] > FLT_EPSILON && shock_digression[axle] < 1.f)
			{
				float abs_rise = UMath::Abs(rise);
				float valving = shock_valving[axle];
				max_compression = abs_rise;
				if (abs_rise > valving)
				{
					float digression = powf(max_compression / valving, shock_digression[axle]) * valving;
					if (rise > 0.f)
						rise = digression;
					else
						rise = -digression;
				}
			}

			float damp = rise > 0.f ? shock_specs[axle] : shock_ext_specs[axle];
			damp *= rise;
			if (damp > mass * mSuspensionInfo.SHOCK_BLOWOUT() * 9.81f)
				damp = 0.f;
			float springForce = damp + sway_stiffness[i] + spring;
			springForce = UMath::Max(springForce, 0.f);

			UMath::Vector3 verticalForce;
			UMath::Vector3 driveForce;
			UMath::Vector3 lateralForce;
			UMath::Vector3 c;

			//UMath::Vector3 up = (UMath::Vector3&)state.GetUpVector() * springForce;
			verticalForce = state.GetUpVector();
			verticalForce = verticalForce * springForce;
			//UMath::Scale(state.GetUpVector(), springForce, verticalForce);
			UMath::Cross(c, forwardNormal, groundNormal);
			UMath::Cross(c, c, forwardNormal);

			float d2 = UMath::Dot(c, groundNormal);
			float load = springForce * UMath::Max(d2 * 4.f - 3.f, 0.3f);
			float xspeed = UMath::Dot(wheel.mVelocity, lateralNormal);
			float zspeed = UMath::Dot(wheel.mVelocity, forwardNormal);
			
			float traction_force = wheel.UpdateLoaded(xspeed, zspeed, state.speed, load, state.time);
			float max_traction = (xspeed / dT) * 0.25f * mass;
			traction_force = UMath::Bound(traction_force, UMath::Abs(max_traction));
			UMath::Scale(lateralNormal, traction_force, lateralForce);

			UMath::Vector3 force;
			UMath::Vector3 pointVelocity;
			UMath::UnitCross(lateralNormal, groundNormal, force);
			UMath::Scale(force, wheel.mLongitudeForce, driveForce);
			force = driveForce + lateralForce + verticalForce;
			
			wheel.mForce = force;
			resolve = true;
		}
		else
		{
			wheel.mForce = UMath::Vector3::kZero;
			wheel.UpdateFree(dT);
		}

		if (newCompression == 0.f)
			wheel.mAirTime += dT;
		else
			wheel.mAirTime = 0.f;
		wheel.mCompression = UMath::Max(newCompression, 0.f);
	}

	if (resolve)
	{
		UMath::Vector3 total_torque = UMath::Vector3::kZero;
		UMath::Vector3 total_force = UMath::Vector3::kZero;

		for (int i = 0; i < GetNumWheels(); ++i)
		{
			Tire& wheel = GetWheel(i);
			UMath::Vector3 p(wheel.mLocalArm.x, wheel.mLocalArm.y + wheel.mCompression - rideheight_specs[(uint32_t)i >> 1], wheel.mLocalArm.z);
			UMath::RotateTranslate(p, state.matrix, p);
			wheel.mPosition = p;
			UMath::Vector3 force = wheel.mForce;
			UMath::Vector3 torque;
			UMath::Cross(p - state.world_cog, force, torque);
			total_force += force;
			total_torque += torque;
		}
		
		float yaw = UMath::Dot(total_torque, state.GetUpVector());
		float counter_yaw = yaw * mTireInfo.YAW_SPEED();
		if (state.driver_style == STYLE_DRAG)
			counter_yaw *= 1.6f;
		counter_yaw -= yaw;
		UMath::ScaleAdd(state.GetUpVector(), counter_yaw, total_torque, total_torque);
		mRB->Resolve(total_force, total_torque);
	}

	if (maxDelta > 0.f)
	{
		/* if (GetNumWheels())
		{
			
		} */
		for (int i = 0; i < GetNumWheels(); ++i)
		{
			mTires[i]->mPosition.y += maxDelta;
		}
		mRB->ModifyYPos(maxDelta);
	}

	if (wheelsOnGround != 0 && !mNumWheelsOnGround)
	{
		state.local_angular_vel.z *= 0.5f;
		state.local_angular_vel.y *= 0.5f;
		UMath::Rotate(state.local_angular_vel, state.matrix, state.angular_vel);
		mRB->SetAngularVelocity(state.angular_vel);
	}
	mNumWheelsOnGround = wheelsOnGround;
}
