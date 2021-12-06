#include "generated/attribsys/classes/brakes.h"
#include "generated/attribsys/classes/chassis.h"
#include "generated/attribsys/classes/tires.h"
#include "generated/attribsys/classes/transmission.h"

#include "interfaces/simables/ichassis.h"
#include "math/vector.h"
#include "math/matrix.h"
#include "misc/graph.h"
#include "misc/table.h"
#include "physics/common/average.h"
#include "physics/wheel.h"

// Globals
static float BurnOutCancelSlipValue = 0.5f;
static float BurnOutYawCancel = 0.5f;
static float BurnOutAllowTime = 1.f;
static float BurnOutMaxSpeed = 20.f;
static float BurnOutFishTailTime = 2.f;
static int BurnOutFishTails = 6;
static UMath::Vector2 BurnoutFrictionData[] = 
{
	UMath::Vector2(0.f, 1.f),
	UMath::Vector2(5.f, 0.8f),
	UMath::Vector2(9.f, 0.9f),
	UMath::Vector2(12.6f, 0.833f),
	UMath::Vector2(17.1f, 0.72f),
	UMath::Vector2(25.f, 0.65f)
};
static tGraph<float> BurnoutFrictionTable = tGraph<float>(BurnoutFrictionData, 6);
static float DriftRearFrictionData[] = { 1.1f, 0.95f, 0.87f, 0.77f, 0.67f, 0.6f, 0.51f, 0.43f, 0.37f, 0.34f };
static Table DriftRearFrictionTable = Table(10, 0.f, 1.f, 9.f, DriftRearFrictionData);
static UMath::Vector2 DriftStabilizerData[] = 
{
	UMath::Vector2(0.f, 0.f),
	UMath::Vector2(0.2617994f, 0.1f),
	UMath::Vector2(0.52359879f, 0.45f),
	UMath::Vector2(0.78539819f, 0.85f),
	UMath::Vector2(1.0471976f, 0.95f),
	UMath::Vector2(1.5533431f, 1.15f),
	UMath::Vector2(1.5707964f, 0.f)
};
static tGraph<float> DriftStabilizerTable = tGraph<float>(DriftStabilizerData, 7);

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
};

} // namespace Chassis

class SuspensionRacer : ISuspension
{
public:
    SuspensionRacer();
    void OnTaskSimulate(float dT);
	void ComputeAckerman(const float steering, const Chassis::State& state, UMath::Vector4& left, UMath::Vector4& right);
	void DoDrifting(const Chassis::State& state);

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
    struct /* RBVehicle:: */IRigidBody *mRB;
    struct /* RBVehicle:: */ICollisionBody *mCollisionBody;
    struct /* EngineRacer:: */ITransmission *mTransmission;
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
		void Update(const float dT, const float speed_mph, const float wheel_slip, const int wheel_ind, const float yaw);
        
		uint32_t mState;
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
		float ComputeLateralForce(float wheel_load, float abs_slip_angle);
		float GetPilotFactor(const float speed);
		void CheckForBrakeLock(float ground_force);
		void CheckSign();
		float UpdateLoaded(float lat_vel, float fwd_vel, float body_speed, float load, float dT);
        
		float mRadius;
        float mBrake;
        float mEBrake;
        float mAV;
        float mLoad;
        float mLateralForce;
        float mLongitudeForce;
        volatile float mDriveTorque;
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
        volatile bool mBrakeLocked;

        enum LastRotationSign
        {
            WAS_POSITIVE,
            WAS_ZERO,
            WAS_NEGATIVE
        } mLastSign;

        float mDragReduction;
    } *mTires[4];

};
//const int offset = offsetof(SuspensionRacer::Tire, mRoadSpeed);