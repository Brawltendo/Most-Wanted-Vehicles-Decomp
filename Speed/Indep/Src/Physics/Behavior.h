#pragma once
#include "Sim/SimObject.h"
#include "Speed/indep/Libs/Support/Utility/UCrc.h"


class Behavior : public Sim::Object, public UTL::COM::Factory
{
public:
	UCrc32& GetMechanic() { return mMechanic; }
	UCrc32& GetSignature() { return mSignature; }
	bool IsPaused() { return mPaused; }
	class ISimable* GetOwner() { return mIOwner; }
	const int GetPriority() { return mPriority; }

private:
	bool mPaused;
	void* mOwner;
	class ISimable* mIOwner;
	UCrc32 mMechanic;
	UCrc32 mSignature;
	int mPriority;
	void* mProfile;
};