#pragma once
// attribs
#include "generated/attribsys/classes/chassis.h"

// interfaces/inheritance
#include "physics/VehicleBehaviors.h"
#include "interfaces/simables/isuspension.h"

#include "input/isteeringwheel.h"
#include "math/vector.h"
#include "math/matrix.h"
#include "misc/graph.h"
#include "misc/table.hpp"
#include "physics/common/average.h"
#include "physics/physicstunings.h"

// Globals
bool IsFront(uint32_t i) { return i < 2; }
bool IsRear(uint32_t i)  { return i > 1; }

class Chassis : public VehicleBehavior, public ISuspension
{

public:

	struct State
	{
	    UMath::Matrix4 matrix;
	    UMath::Vector3 local_vel;
	    float gas_input;
	    UMath::Vector3 linear_vel;
	    float brake_input;
	    UMath::Vector3 angular_vel;
	    float ground_effect;
	    UMath::Vector3 cog;
	    float ebrake_input;
	    UMath::Vector3 dimension;
	    float steer_input;
	    UMath::Vector3 local_angular_vel;
	    float slipangle;
	    UMath::Vector3 inertia;
	    float mass;
	    UMath::Vector3 world_cog;
	    float speed;
	    float time;
	    int flags;
	    int16_t driver_style;
	    int16_t driver_class;
	    int16_t gear;
	    int16_t blown_tires;
	    float nos_boost;
	    float shift_boost;
	    struct WCollider *collider;

		enum Flags
		{
			IS_STAGING = 1,
			IS_DESTROYED
		};

		const UMath::Vector3& GetRightVector() const { return (UMath::Vector3&)matrix.v0; }
		const UMath::Vector3& GetUpVector() const { return (UMath::Vector3&)matrix.v1; }
		const UMath::Vector3& GetForwardVector() const { return (UMath::Vector3&)matrix.v2; }
		const UMath::Vector3& GetPosition() const { return (UMath::Vector3&)matrix.v3; }

	};

	float ComputeMaxSlip(const State& state);
	void OnTaskSimulate(float dT);
	float ComputeLateralGripScale(const Chassis::State& state);
	float ComputeTractionScale(const Chassis::State& state);
	void ComputeAckerman(const float steering, const State& state, UMath::Vector4& left, UMath::Vector4& right);
	void SetCOG(float extra_bias, float extra_ride);
	void ComputeState(State& state, float dT);
	void DoAerodynamics(const Chassis::State& state, float drag_pct, float aero_pct, float aero_front_z, float aero_rear_z, const Physics::Tunings& tunings);
	void DoJumpStabilizer(State& state);

private:
	class ICollisionBody* mRBComplex;
	class IInput* mInput;
	class IEngine* mEngine;
	class ITransmission* mTransmission;
	class IDragTransmission* mDragTrany;
	class IEngineDamage* mEngineDamage;
	class ISpikeable* mSpikeDamage;
	Attrib::Gen::chassis mAttributes;
    float mJumpTime;
    float mJumpAltitude;
    float mTireHeat;

};

float YawFrictionBoost(float yaw, float ebrake, float speed, float yawcontrol, float grade);