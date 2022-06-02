#include "physics/behaviors/chassis.h"

// interfaces
#include "interfaces/Simables/IRigidBody.h"
#include "interfaces/simables/isimable.h"
#include "interfaces/simables/itransmission.h"
#include "interfaces/simables/ivehicle.h"
#include "interfaces/simentities/iplayer.h"

// math
#include "math/bmath.h"
#include "math/mathcommon.h"

#include "physics/physicsinfo.hpp"

// MATCHING
float Chassis::ComputeMaxSlip(const Chassis::State& state)
{
	float ramp = UMath::Ramp(state.speed, 10.f, 71.f);
	float result = ramp + 0.5f;
	if (state.gear == G_REVERSE)
		result = 71.f;
	return result;
}

float GripVsSpeed[] = { 0.833f, 0.958f, 1.008f, 1.0167f, 1.033f, 1.033f, 1.033f, 1.0167f, 1.f, 1.f };
Table GripRangeTable(10, 0.f, 1.f, 9.f, GripVsSpeed);
// MATCHING
float Chassis::ComputeLateralGripScale(const Chassis::State& state)
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
float Chassis::ComputeTractionScale(const Chassis::State& state)
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
void Chassis::ComputeAckerman(const float steering, const Chassis::State& state, UMath::Vector4& left, UMath::Vector4& right)
{
	UMath::Vector3 steer_vec;
	int going_right = true;
	float wheel_base = mAttributes.WHEEL_BASE();
	float track_width_front = mAttributes.TRACK_WIDTH().At(0);
	float steering_angle_radians = steering * TWO_PI;

	// clamp steering angle <= 180 degrees
	if (steering_angle_radians > PI)
		steering_angle_radians -= TWO_PI;

	// negative steering angle indicates a left turn
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

static float AeroDropOff = 0.5f;
static float AeroDropOffMin = 0.4f;
static float OffThrottleDragFactor = 2.f;
static float OffThrottleDragCenterHeight = -0.1f;
// MATCHING
void Chassis::DoAerodynamics(const Chassis::State& state, float drag_pct, float aero_pct, float aero_front_z, float aero_rear_z, const Physics::Tunings& tunings)
{
	// eventually I'll set up proper inheritance for this class...
	IRigidBody* irb = mIOwner->GetRigidBody();

	if (drag_pct > 0.f)
	{
		const float dragcoef_spec = mAttributes.DRAG_COEFFICIENT();
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
		float downforce = Physics::Info::AerodynamicDownforce(mAttributes, state.speed) * drop_off * upness * aero_pct;
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
				aero_center.z = (mAttributes.AERO_CG() * 0.01f) * (aero_front_z - aero_rear_z) + aero_rear_z;

			UMath::Vector3 force(0.f, -downforce, 0.f);
			UMath::RotateTranslate(aero_center, state.matrix, aero_center);
			UMath::Rotate(force, state.matrix, force);
			irb->ResolveForceAtPoint(force, aero_center);
		}
	}
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
	// for some reason whoever wrote this didn't use UMath::Abs in this function and instead used C's abs function
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
