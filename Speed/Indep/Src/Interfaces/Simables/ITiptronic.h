#pragma once
#include "speedcommon.h"
#include "physics/physicstypes.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


SPEED_INTERFACE ITiptronic : public UTL::COM::IUnknown
{
public:
    virtual bool SportShift();
};