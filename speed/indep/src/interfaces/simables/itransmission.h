#include "speedcommon.h"
#include "physics/physicstypes.h"

class ITransmission
{
public:
	virtual void _PADDING();
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