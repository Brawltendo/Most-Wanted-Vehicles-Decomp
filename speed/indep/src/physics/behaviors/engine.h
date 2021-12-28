#include "generated/attribsys/classes/engine.h"
#include "generated/attribsys/classes/induction.h"
#include "generated/attribsys/classes/nos.h"
#include "generated/attribsys/classes/tires.h"
#include "generated/attribsys/classes/transmission.h"

#include "interfaces/simables/iengine.h"
#include "interfaces/simables/itaskable.h"
#include "interfaces/simables/itransmission.h"
#include "interfaces/simables/ivehicle.h"
#include "physics/physicstypes.h"

class EngineRacer : Sim::ITaskable, IVehicle, ITransmission, IEngine 
{
private:
	float GetBrakingTorque(float engine_torque, float rpm);
	uint32_t GetNumGearRatios();
	float GetGearRatio(uint32_t idx);
	float GetGearEfficiency(uint32_t idx);
	float GetFinalGear();
	float GetRatioChange(uint32_t from, uint32_t to);
	float GetShiftDelay(uint32_t gear);
	bool RearWheelDrive();
	bool FrontWheelDrive();
	float GetDifferentialAngularVelocity(bool locked);
	void SetDifferentialAngularVelocity(float w);
	void LimitFreeWheels(float w);

	int pad[0x64 / 0x4];
	float mDriveTorque;
	GearID mGear;
	float mGearShiftTimer;
	float mThrottle;
	float mSpool;
	float mPSI;
	float mInductionBoost;
	float mShiftUpRPM[10];
	float mShiftDownRPM[10];
	float mAngularVelocity;
	float mAngularAcceleration;
	float mTransmissionVelocity;
	float mNOSCapacity;
	float mNOSBoost;
	float mNOSEngaged;
	float mClutchRPMDiff;
	bool mEngineBraking;
	float mSportShifting;
	/* struct IInput* mIInput;
	struct ISuspension* mSuspension;
	struct ICheater* mCheater; */
	int mIInput;
	struct ISuspension* mSuspension;
	int mCheater;
	Attrib::Gen::nos mNOSInfo;
	Attrib::Gen::induction mInductionInfo;
	Attrib::Gen::engine mEngineInfo;
	Attrib::Gen::transmission mTrannyInfo;
	Attrib::Gen::tires mTireInfo;
	float mRPM;
	ShiftStatus mShiftStatus;
	ShiftPotential mShiftPotential;
	float mPeakTorque;
	float mPeakTorqueRPM;
	float mMaxHP;

	struct Clutch 
	{
	public:
		enum State
		{
			ENGAGED, ENGAGING, DISENGAGED
		};

		Clutch();
		void Disengage();
		void Engage(float time);
		void Reset();
		float Update(float dT);
		State GetState();

		State mState;
		float mTime;
		float mEngageTime;
	} mClutch;

	float mBlown;
	float mSabotage;
	int gap158;

public:
	// interface functions will go here
};
//const int offset = offsetof(EngineRacer, mTrannyInfo);

float Engine_SmoothRPM(bool is_shifting, GearID gear, float dT, float old_rpm, float new_rpm, float engine_inertia);

