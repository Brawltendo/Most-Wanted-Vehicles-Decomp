#include "physics/behaviors/chassis.h"

#include "interfaces/simables/iplayer.h"
#include "interfaces/simables/isimable.h"
#include "interfaces/simables/itransmission.h"
#include "math/bmath.h"
#include "math/mathcommon.h"
#include "math/matrix.h"
#include "physics/physicsinfo.h"

extern float Sim_GetTime();
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

// MATCHING
float SuspensionRacer::ComputeMaxSlip(const Chassis::State& state)
{
	float ramp = UMath::Ramp(state.speed, 10.f, 71.f);
	float result = ramp + 0.5f;
	if (state.gear == G_REVERSE)
		result = 71.f;
	return result;
}

static float AeroDropOff = 0.5f;
static float AeroDropOffMin = 0.4f;
static float OffThrottleDragFactor = 2.f;
static float OffThrottleDragCenterHeight = -0.1f;
// MATCHING
void SuspensionRacer::DoAerodynamics(const Chassis::State& state, float drag_pct, float aero_pct, float aero_front_z, float aero_rear_z, const Physics::Tunings& tunings)
{
	// eventually I'll set up proper inheritance for this class...
	IRigidBody* irb = ((ISimable*)pad[0x30 / 0x4])->GetRigidBody();

	if (drag_pct > 0.f)
	{
		const float dragcoef_spec = mChassisInfo.DRAG_COEFFICIENT();
		// drag increases relative to the car's speed
		// letting off the throttle will increase drag by OffThrottleDragFactor
		float drag = (dragcoef_spec * state.speed * drag_pct)
				   * ((OffThrottleDragFactor - 1.f) * (1.f - state.gas_input) + 1.f);
		if (&tunings)
			drag *= tunings.aerodynamicsTuning * 0.25f + 1.f;
		
		UMath::Vector3 drag_vector(state.linear_vel);
		UMath::Scale(drag_vector, -drag, drag_vector);
		UMath::Vector3 drag_center(state.cog);

		// lower drag vertical pos based on off-throttle amount as long as 2 or more wheels are grounded
		if (state.ground_effect >= 0.5f)
			drag_center.y += (1.f - state.gas_input) * OffThrottleDragCenterHeight;

		UMath::RotateTranslate(drag_center, state.matrix, drag_center);
		irb->ResolveForceAtPoint(drag_vector, drag_center);
	}

	if (aero_pct > 0.f)
	{
		// this should really be an inlined UMath::Max but for some reason it doesn't wanna generate correctly
		// so instead I manually inlined it
		float upness_temp = state.GetUpVector().y;
		if (!(upness_temp > 0.f))
			upness_temp = 0.f;
		// scale downforce by the gradient when less than 2 wheels are grounded
		float upness = upness_temp;
		if (state.ground_effect >= 0.5f)
			upness = 1.f;
		
		// in reverse, the car's forward vector is used as the movement direction
		UMath::Vector3 movement_dir(state.GetForwardVector());
		if (state.speed > 0.0001f)
		{
			UMath::Vector3 lin_vel(state.linear_vel);
			UMath::Scale(lin_vel, 1.f / state.speed, movement_dir);
		}

		float forwardness = UMath::Max(UMath::Dot(movement_dir, state.GetForwardVector()), 0.f);
		float drop_off = UMath::Max(AeroDropOffMin, powf(forwardness, AeroDropOff));
		float downforce = Physics::Info::AerodynamicDownforce(mChassisInfo, state.speed) * drop_off * upness * aero_pct;
		// lower downforce when car is in air
		if (state.ground_effect == 0.f)
			downforce *= 0.8f;
		if (&tunings)
			downforce *= tunings.aerodynamicsTuning * 0.25f + 1.f;
		
		if (downforce > 0.f)
		{
			UMath::Vector3 aero_center(state.cog);
			// when at least 1 wheel is grounded, change the downforce forward position using the aero CG and axle positions
			if (state.ground_effect != 0.f)
				aero_center.z = (mChassisInfo.AERO_CG() * 0.01f) * (aero_front_z - aero_rear_z) + aero_rear_z;

			UMath::Vector3 force(0.f, -downforce, 0.f);
			UMath::RotateTranslate(aero_center, state.matrix, aero_center);
			UMath::Rotate(force, state.matrix, force);
			irb->ResolveForceAtPoint(force, aero_center);
		}
	}
}

float GripVsSpeed[] = { 0.833f, 0.958f, 1.008f, 1.0167f, 1.033f, 1.033f, 1.033f, 1.0167f, 1.f, 1.f };
Table GripRangeTable(10, 0.f, 1.f, 9.f, GripVsSpeed);
// MATCHING
float SuspensionRacer::ComputeLateralGripScale(const Chassis::State& state)
{
	// lateral grip is tripled when in a drag race
	if (state.driver_style == STYLE_DRAG)
		return 3.f;
	else
	{
		float ratio = UMath::Ramp(state.speed, 0.f, MPH2MPS(85.f));
		return GripRangeTable.GetValue(ratio) * 1.2f;
	}
}

float TractionVsSpeed[] = { 0.90899998f, 1.045f, 1.09f, 1.09f, 1.09f, 1.09f, 1.09f, 1.045f, 1.f, 1.f };
Table TractionRangeTable(10, 0.f, 1.f, 9.f, TractionVsSpeed);
// MATCHING
float SuspensionRacer::ComputeTractionScale(const Chassis::State& state)
{
	float result;
	if (state.driver_style == STYLE_DRAG)
		result = 1.1f;
	else
	{
		float ratio = UMath::Ramp(state.speed, 0.f, MPH2MPS(85.f));
		result = TractionRangeTable.GetValue(ratio) * 1.1f;
	}

	// traction is doubled when in reverse
	if (state.gear == G_REVERSE)
		result = 2.f;

	return result;
}

// MATCHING
void SuspensionRacer::ComputeAckerman(const float steering, const Chassis::State& state, UMath::Vector4& left, UMath::Vector4& right)
{
	UMath::Vector3 steer_vec;
	int going_right = true;
	float wheel_base = mChassisInfo.WHEEL_BASE();
	float track_width_front = mChassisInfo.TRACK_WIDTH().At(0);
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
	UMath::Rotate(steer_vec, state.matrix, steer_vec);
	right = UMath::Vector4(steer_vec, steer_right);

	steer_vec.y = 0.f;
	steer_vec.z = cosf(steer_left);
	steer_vec.x = sinf(steer_left);
	UMath::Rotate(steer_vec, state.matrix, steer_vec);
	left = UMath::Vector4(steer_vec, steer_left);
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
// <@>PRINT_ASM
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
			// ugly ass casting because I don't wanna deal with this class's inheritance and virtual functions right now
			// you get the point anyway

			IPlayer* player = (IPlayer*)((ISimable*)pad[12])->GetPlayer();
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

// MATCHING
float SuspensionRacer::DoHumanSteering(const Chassis::State& state)
{
	float input = state.steer_input;
	float prev_steering = mSteering.Previous;

	if (prev_steering >= 180.f)
		prev_steering -= 360.f;

	float steering_coeff = mTireInfo.STEERING();
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
}

// can't tell if this function is matching or not since it's inlined in the PC version and I haven't found the uninlined function
// it's generating the right code in DoSteering though so it probably matches
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
// I have no idea why these are set to -2 since when they're actually used in the function it acts like 1.0f
static int LatForceMultipliers[] = { -2, -2, -2, -2 };
static int* pLatForceMultipliers = LatForceMultipliers;
float SuspensionRacer::Tire::ComputeLateralForce(float load, float slip_angle)
{
	float angle = (slip_angle * 360.f) * 0.5f;
	// there are 3 instructions here that are out of order but everything else matches
	// Black Box what did you do???????
	// why??????
	int slip_angle_table = (int)angle;
	load = (load * 0.001f) * 0.8f;
	
	if (slip_angle_table >= 6)
	{
		float grip_scale = mSpecs->GRIP_SCALE().Pair[mAxleIndex];
		return (((LoadSensitivityTable[6]->GetValue(load)
			 * pLatForceMultipliers[2]) * mGripBoost) * grip_scale) * 2500.f;
	}
	else
	{
		float low  = LoadSensitivityTable[slip_angle_table]->GetValue(load);
		float high = LoadSensitivityTable[slip_angle_table + 1]->GetValue(load);
		float grip_scale = mSpecs->GRIP_SCALE().Pair[mAxleIndex];
		float delta = angle - slip_angle_table;
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
	float bt_add  = bt * mEBrake;
	float ebt_add = ebt * mEBrake;
	float abs_fwd = UMath::Abs(fwd_vel);
	if (abs_fwd < 1.f)
	{
		// when car is nearly stopped, apply brake torque using forward velocity and wheel load
		float inv_radius = 1.f / mRadius;
		float stopping_bt    = -(inv_radius * mBrake  * fwd_vel * load);
		float stopping_ebt   = -(inv_radius * mEBrake * fwd_vel * load);
		float r_drive_torque = -(mEBrake * mDriveTorque);
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
	float slip_speed = (mAV * mRadius) - fwd_vel;
	float slip_ground_friction = 0.f;
	float dynamic_friction = 1.f;
	mSlip = slip_speed;
	float skid_speed = fsqrt(slip_speed * slip_speed + lat_vel * lat_vel);
	float pilot_factor = GetPilotFactor(body_speed);
	if (skid_speed > FLT_EPSILON && (lat_vel != 0.f || fwd_vel != 0.f))
	{
		dynamic_friction = (mTractionBoost * pilot_factor) * dynamicgrip_spec;
		slip_ground_friction = (dynamic_friction / skid_speed) * mLoad;
		float vel_len = fsqrt(fwd_vel * fwd_vel + lat_vel * lat_vel);
		//float wheel_speed = mLoad / fsqrt(fwd_vel * fwd_vel + lat_vel * lat_vel);
		float ground_speed = ((mLoad / vel_len) * dynamic_friction) * abs_fwd;
		CheckForBrakeLock(ground_speed);
	}
	if (mTraction < 1.f || mBrakeLocked)
	{
		float long_force = slip_ground_friction * slip_speed;
		mLongitudeForce = long_force;
		mLateralForce = -(slip_ground_friction * lat_vel);
		// 0.44703 mps = 1 mph
		const float one_mph = 0.44703f;
		if (body_speed < one_mph && dynamic_friction > 0.1f)
		{
			mLateralForce /= dynamic_friction;
			mLongitudeForce /= dynamic_friction;
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
		//float temp_load = mLoad;
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
	mLateralForce = mLateralForce * mTractionCircle.x;
	float ellipse_y = mLongitudeForce * mTractionCircle.y;
	mLongitudeForce = mLongitudeForce * mTractionCircle.y;
	float wheel_force_length = fsqrt(ellipse_y * ellipse_y + ellipse_x * ellipse_x);
	//wheel_force_length = fsqrt(wheel_force_length);
	mTraction = 1.f;
	float traction_scale = ((mDriftFriction * mTractionBoost) * staticgrip_spec) * mLoad * pilot_factor;
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
		tolerated_slip = (ratio * ratio) * tolerated_slip;
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
		mLongitudeForce -= sinf(mSlipAngle * TWO_PI) * (mDragReduction / mSpecs->GRIP_SCALE().Pair[mAxleIndex]) * mLateralForce;
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
			float last_torque = (((mBrakeTorque + mDriveTorque) - (mLongitudeForce * mRadius)) + mLastTorque) * 0.5f;
			mLastTorque = last_torque;
			mAngularAcc = (last_torque - (RollingFriction * mSurface.data->ROLLING_RESISTANCE) * mAV)
						/ WheelMomentOfInertia - (mTraction * mSlip) / (dT * mRadius);
		}
		mAngularAcc += ((fwd_acc / mRadius) - mAngularAcc) * mTraction;
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
float YawFrictionBoost(float yaw, float ebrake, float speed, float yawcontrol, float grade)
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
}

// stack frame is off in this function for some reason and I can't get it it to match up
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
