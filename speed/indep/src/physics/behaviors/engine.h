#include "generated/attribsys/classes/engine.h"
#include "generated/attribsys/classes/induction.h"
#include "generated/attribsys/classes/nos.h"
#include "generated/attribsys/classes/tires.h"
#include "generated/attribsys/classes/transmission.h"

#include "physics/physicstypes.h"

class EngineRacer
{


	int pad[0x6C / 0x4];
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
	int pad58[3];
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
		float GetClutchRatio(float dT);

		enum State
		{
			ENGAGED, ENGAGING, DISENGAGED
		} mState;

		float mTime;
		float mEngageTime;
	} mClutch;

	float mBlown;
	float mSabotage;
	int gap158;
};
//const int offset = offsetof(EngineRacer, mInductionInfo);