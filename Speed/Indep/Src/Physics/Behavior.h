#pragma once
#include "Sim/SimObject.h"
#include "Speed/indep/Libs/Support/Utility/UCrc.h"


class Behavior : public Sim::Object, public UTL::COM::Factory
{
public:
	bool mPaused;
	void* mOwner;
	class ISimable* mIOwner;
	UCrc32 mMechanic;
	UCrc32 mSignature;
	int mPriority;
	void* mProfile;
};