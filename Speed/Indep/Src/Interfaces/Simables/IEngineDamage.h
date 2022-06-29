#pragma once
#include "speedcommon.h"
#include "physics/physicstypes.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


SPEED_INTERFACE IEngineDamage : public UTL::COM::IUnknown
{
public:
	virtual ~IEngineDamage();
    virtual bool IsBlown();
    virtual void Blow();
    virtual void Sabotage(float time);
    virtual bool IsSabotaged();
    virtual void Repair();
};