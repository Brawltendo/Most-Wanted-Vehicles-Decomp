#pragma once
//#include "speedcommon.h"
#include "physics/physicstypes.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


class ITransmission : public UTL::COM::IUnknown
{
public:
    virtual GearID GetGear();
    virtual GearID GetTopGear();
    virtual bool Shift(GearID to_gear);
    virtual bool IsGearChanging();
    virtual bool IsReversing();
    virtual float GetSpeedometer();
    virtual float GetMaxSpeedometer();
    virtual float GetDriveTorque();
    virtual float GetOptimalShiftRange(GearID);
    virtual float GetShiftPoint(GearID, GearID);
    virtual ShiftStatus GetShiftStatus();
};