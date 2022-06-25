#pragma once
#include "speedcommon.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


SPEED_INTERFACE ICheater : public UTL::COM::IUnknown
{
public:
    virtual float GetCatchupCheat();
};