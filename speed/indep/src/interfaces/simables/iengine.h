#include "speedcommon.h"
#include "physics/physicstypes.h"

class IEngine
{
public:
	virtual void _PADDING();
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