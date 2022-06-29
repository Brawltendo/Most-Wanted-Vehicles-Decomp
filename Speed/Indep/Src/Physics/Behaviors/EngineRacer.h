#pragma once
// attribs
#include "Generated/AttribSys/Classes/engine.h"
#include "Generated/AttribSys/Classes/induction.h"
#include "Generated/AttribSys/Classes/nos.h"
#include "Generated/AttribSys/Classes/tires.h"
#include "Generated/AttribSys/Classes/transmission.h"

// interfaces/inheritance
#include "Physics/VehicleBehaviors.h"
#include "Interfaces/IAttributeable.h"
#include "Interfaces/Simables/IEngine.h"
#include "Interfaces/Simables/IEngineDamage.h"
#include "Interfaces/Simables/IInductable.h"
#include "Interfaces/Simables/ITiptronic.h"
#include "Interfaces/Simables/ITransmission.h"

#include "Physics/PhysicsTypes.h"

namespace Physics { struct Tunings; } // namespace Physics


class EngineRacer : protected VehicleBehavior, 
					protected ITransmission, 
					protected IEngine, 
					public IAttributeable, 
					public IInductable, 
					public ITiptronic,
					public IRaceEngine,
					public IEngineDamage
{
public:
	virtual ~EngineRacer();
	virtual void OnService();
	virtual void Reset();
	virtual void GetPriority();
	virtual void OnOwnerAttached();
	virtual void OnOwnerDetached();
	virtual void OnTaskSimulate(float dT);
	virtual void OnBehaviorChange(const struct UCrc32&);
	virtual void OnPause();
	virtual void OnUnPause();
	virtual bool IsEngineBraking();
	virtual bool IsShiftingGear();
	virtual ShiftStatus OnGearChange(GearID gear);
	virtual bool UseRevLimiter();
	virtual void DoECU();
	virtual float DoThrottle();
	virtual void DoInduction(const Physics::Tunings* tunings, float dT);
	virtual float DoNos(const Physics::Tunings* tunings, float dT, bool engaged);
	virtual void DoShifting(float dT);

	uint32_t GetNumGearRatios();
	float GetGearRatio(uint32_t idx);
	float GetGearEfficiency(uint32_t idx);
	float GetFinalGear();
	float GetRatioChange(uint32_t from, uint32_t to);
	float GetShiftDelay(uint32_t gear);
	bool RearWheelDrive();
	bool FrontWheelDrive();
	ShiftPotential FindShiftPotential(GearID gear, float rpm);
	float GetDifferentialAngularVelocity(bool locked);
	float GetDriveWheelSlippage();
	void SetDifferentialAngularVelocity(float w);
	float CalcSpeedometer(float rpm, uint32_t gear);
	void LimitFreeWheels(float w);
	float GetBrakingTorque(float engine_torque, float rpm);
	bool DoGearChange(GearID gear, bool automatic);
	void AutoShift();
	

	// ITransmission
	
	bool Shift(GearID gear);
	float GetSpeedometer();
	float GetMaxSpeedometer();
	float GetShiftPoint(GearID from_gear, GearID to_gear);


	// IEngine

	bool IsNOSEngaged();
	float GetNOSFlowRate();
	bool HasNOS();
	void ChargeNOS(float charge);

	// IRaceEngine

	float GetPerfectLaunchRange(float& range);

private:
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
	struct IInput* mIInput;
	struct ISuspension* mSuspension;
	struct ICheater* mCheater;
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

};
//const int offset = offsetof(EngineRacer, mTrannyInfo);

float Engine_SmoothRPM(bool is_shifting, GearID gear, float dT, float old_rpm, float new_rpm, float engine_inertia);

