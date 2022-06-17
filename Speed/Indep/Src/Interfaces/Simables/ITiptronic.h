#pragma once
//#include "speedcommon.h"
#include "physics/physicstypes.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


class ITiptronic : public UTL::COM::IUnknown
{
public:
	virtual void _PADDING();
    virtual bool SportShift();
};