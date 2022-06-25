#pragma once
#include "speedcommon.h"
#include "physics/physicstypes.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


SPEED_INTERFACE IInductable : public UTL::COM::IUnknown
{
public:
	virtual void _PADDING();
    virtual eInductionType InductionType();
    virtual float InductionSpool();
    virtual float GetInductionPSI();
    virtual float GetMaxInductionPSI();
};