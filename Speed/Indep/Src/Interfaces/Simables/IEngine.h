#pragma once
#include "speedcommon.h"
#include "physics/physicstypes.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


SPEED_INTERFACE IEngine : public UTL::COM::IUnknown
{
public:
	virtual ~IEngine();
    virtual float GetRPM();
    virtual float GetRedline();
    virtual float GetMaxRPM();
    virtual float GetMinRPM();
    virtual float GetPeakTorqueRPM();
    virtual void MatchSpeed(float speed);
    virtual float GetNOSCapacity();
    virtual bool IsNOSEngaged();
    virtual float GetNOSFlowRate();
    virtual float GetNOSBoost();
    virtual bool HasNOS();
    virtual void ChargeNOS(float charge);
    virtual float GetMaxHorsePower();
    virtual float GetMinHorsePower();
    virtual float GetHorsePower();
};

SPEED_INTERFACE IRaceEngine : public UTL::COM::IUnknown
{
public:
	virtual ~IRaceEngine();
    virtual float GetPerfectLaunchRange(float& range);
};