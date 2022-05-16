#include "generated/attribsys/classes/brakes.h"
#include "generated/attribsys/classes/chassis.h"
#include "generated/attribsys/classes/tires.h"
#include "generated/attribsys/classes/transmission.h"

#include "input/isteeringwheel.h"
#include "interfaces/simables/ichassis.h"
#include "math/vector.h"
#include "math/matrix.h"
#include "misc/graph.h"
#include "misc/table.h"
#include "physics/common/average.h"
#include "physics/physicstunings.h"
#include "physics/wheel.h"

// Globals
static float JoystickInputToSteerRemap1[] = 
{
	-1.f, -0.712f, -0.453f, -0.303f, -0.216f, -0.148f, -0.116f, -0.08f, -0.061f, -0.034f,
	0.f,
	0.034f, 0.061f, 0.08f, 0.116f, 0.148f, 0.216f, 0.303f, 0.453f, 0.712f, 1.f
};
static float JoystickInputToSteerRemap2[] = 
{
	-1.f, -0.736f, -0.542f, -0.4f, -0.292f, -0.214f, -0.16f, -0.123f, -0.078f, -0.036f,
	0.f,
	0.036f, 0.078f, 0.123f, 0.16f, 0.214f, 0.292f, 0.4f, 0.542f, 0.736f, 1.f
};
static float JoystickInputToSteerRemap3[] = 
{
	-1.f, -0.8f, -0.615f, -0.483f, -0.388f, -0.288f, -0.22f, -0.161f, -0.111f, -0.057f,
	0.f,
	0.057f, 0.111f, 0.161f, 0.22f, 0.288f, 0.388f, 0.483f, 0.615f, 0.8f, 1.f
};
static float JoystickInputToSteerRemapDrift[] = 
{
	-1.f, -1.f, -0.688f, -0.492f, -0.319f, -0.228f, -0.16f, -0.123f, -0.085f, -0.05f,
	0.f,
	0.05f, 0.085f, 0.123f, 0.16f, 0.228f, 0.319f, 0.492f, 0.688f, 1.f, 1.f
};
static Table SteerInputRemapTables[] = 
{
	Table(21, -1.f, 1.f, 10.f, JoystickInputToSteerRemap1),
	Table(21, -1.f, 1.f, 10.f, JoystickInputToSteerRemap2),
	Table(21, -1.f, 1.f, 10.f, JoystickInputToSteerRemap3),
	Table(21, -1.f, 1.f, 10.f, JoystickInputToSteerRemapDrift)
};

bool IsFront(uint32_t i) { return i < 2; }
bool IsRear(uint32_t i)  { return i > 1; }

namespace Chassis
{
    
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

} // namespace Chassis

class SuspensionRacer : ISuspension
{
public:
    SuspensionRacer();
    void OnTaskSimulate(float dT);
	float ComputeMaxSlip(const Chassis::State& state);
	void DoAerodynamics(const Chassis::State& state, float drag_pct, float aero_pct, float aero_front_z, float aero_rear_z, const Physics::Tunings& tunings);
	float ComputeLateralGripScale(const Chassis::State& state);
	float ComputeTractionScale(const Chassis::State& state);
	void ComputeAckerman(const float steering, const Chassis::State& state, UMath::Vector4& left, UMath::Vector4& right);
	void DoDrifting(const Chassis::State& state);
	void TuneWheelParams(Chassis::State& state);
	void DoWheelForces(Chassis::State& state);
	float CalculateMaxSteering(Chassis::State& state, ISteeringWheel::SteeringType steer_type);
	float CalculateSteeringSpeed(Chassis::State& state);
	void DoWallSteer(Chassis::State& state);
	void DoDriveForces(Chassis::State& state);
	float DoHumanSteering(Chassis::State& state);
	float DoAISteering(Chassis::State& state);
	void DoSteering(Chassis::State& state, UMath::Vector3& right, UMath::Vector3& left);
	float CalcYawControlLimit(float speed);

	int pad[0x6C / 0x4];
	Attrib::Gen::chassis mChassisInfo;
    float mJumpTime;
    float mJumpAltitude;
    float tireHeat;
	char pad_0044[8];
    Attrib::Gen::tires mTireInfo;
    Attrib::Gen::brakes mBrakeInfo;
    Attrib::Gen::chassis mSuspensionInfo;
    Attrib::Gen::transmission mTransInfo;
    struct IRigidBody *mRB;
    struct ICollisionBody *mCollisionBody;
    struct ITransmission *mTransmission;
    struct IHumanAI *mHumanAI;
    float mGameBreaker;
    uint32_t mNumWheelsOnGround;
    float mLastGroundCollision;

    struct Drift
    {
        enum eState
        {
            D_OUT,
            D_ENTER,
            D_IN,
            D_EXIT
        } State;
        float Value;
    } mDrift;

    struct Burnout 
    {
		void Update(const float dT, const float speedmph, const float max_slip, const int max_slip_wheel, const float yaw);
		int GetState() { return mState; }
		float GetTraction() { return mTraction; }
		void Reset()
		{
			mState = 0;
			mBurnOutTime = 0.f;
			mTraction = 1.f;
			mBurnOutAllow = 0.f;
		}
		void SetState(int s) { mState = s; }
		void SetBurnOutTime(float t) { mBurnOutTime = t; }
		void SetTraction(float t) { mTraction = t; }
		float GetBurnOutTime(float t) { return mBurnOutTime; }
		void DecBurnOutTime(float t) { mBurnOutTime -= t; }
		void ClearBurnOutAllow() { mBurnOutAllow = 0.f; }
		void IncBurnOutAllow(float t) { mBurnOutAllow += t; }
        
	private:
		int mState;
        float mBurnOutTime;
        float mTraction;
        float mBurnOutAllow;
    } mBurnOut;

    struct Steering 
    {
        float Previous;
        float Wheels[2];
        float Maximum;
        float LastMaximum;
        float LastInput;
        AverageWindow InputAverage;
        AverageWindow InputSpeedCoeffAverage;
        float CollisionTimer;
        float WallNoseTurn;
        float WallSideTurn;
        float YawControl;
    } mSteering;

    struct Tire : Wheel
    {
		bool IsOnGround() { return mCompression > 0.f; }
		float SetBrake(float brake) { mBrake = brake; }
		float SetEBrake(float ebrake) { mEBrake = ebrake; }
		float GetEBrake() { return mEBrake; }
		float GetRadius() { return mRadius; }
		float GetAngularVelocity() { return mAV; }
		float GetToleratedSlip() { return mSlip; }
		void SetLateralBoost(float f) { mLateralBoost = f; }
		void SetBottomOutTime(float time) { mBottomOutTime = time; }
		void ScaleTractionBoost(float scale) { mTractionBoost *= scale; }
		void SetDriftFriction(float scale) { mDriftFriction = scale; }
		void ApplyDriveTorque(float torque) { if (!mBrakeLocked) mDriveTorque += torque; }
		void ApplyBrakeTorque(float torque) { if (!mBrakeLocked) mBrakeTorque += torque; }
		float GetTotalTorque() { return mBrakeTorque + mDriveTorque; }
		float GetDriveTorque() { return mDriveTorque; }
		float GetLongitudeForce() { return mLongitudeForce; }
		bool IsBrakeLocked() { return mBrakeLocked; }
		bool IsSteeringWheel() { return mWheelIndex < 2; }
		void SetTractionCircle(const UMath::Vector2& circle) { mTractionCircle = circle; }
		float ComputeLateralForce(float load, float slip_angle);
		float GetPilotFactor(const float speed);
		void CheckForBrakeLock(float ground_force);
		void CheckSign();
		void UpdateFree(float dT);
		float UpdateLoaded(float lat_vel, float fwd_vel, float body_speed, float load, float dT);
        
		int pad[3];
		float mRadius;
        float mBrake;
        float mEBrake;
        float mAV;
        float mLoad;
        float mLateralForce;
        float mLongitudeForce;
        float mDriveTorque;
        float mBrakeTorque;
        float mLateralBoost;
        float mTractionBoost;
        float mSlip;
        float mLastTorque;
        int mWheelIndex;
        float mRoadSpeed;
        Attrib::Gen::tires *mSpecs;
        Attrib::Gen::brakes *mBrakes;
        float mAngularAcc;
        int mAxleIndex;
        float mTraction;
        float mBottomOutTime;
        float mSlipAngle;
        UMath::Vector2 mTractionCircle;
        float mMaxSlip;
        float mGripBoost;
        float mDriftFriction;
        float mLateralSpeed;
        bool mBrakeLocked;

        enum LastRotationSign
        {
            WAS_POSITIVE,
            WAS_ZERO,
            WAS_NEGATIVE
        } mLastSign;

        float mDragReduction;
    } *mTires[4];

	struct Differential
	{
		void CalcSplit(bool locked);

		float angular_vel[2];
		int has_traction[2];
		float bias;
		float factor;
		float torque_split[2];
	};

public:
	bool RearWheelDrive()  { return mTransInfo.TORQUE_SPLIT() < 1.f; }
	bool FrontWheelDrive() { return mTransInfo.TORQUE_SPLIT() > 0.f; }
	bool IsDriveWheel(uint32_t i)
	{
		if ((!IsRear(i)  || !RearWheelDrive()) 
		&& ( !IsFront(i) || !FrontWheelDrive()) )
			return false;
		else
			return true;
	}
	Tire& GetWheel(uint32_t i) { return *mTires[i]; }
	const Tire& GetWheel(uint32_t i) const { return *mTires[i]; }

};
//const int offset = offsetof(SuspensionRacer::Tire, mRoadSpeed);

float YawFrictionBoost(float yaw, float ebrake, float speed, float yawcontrol, float grade);