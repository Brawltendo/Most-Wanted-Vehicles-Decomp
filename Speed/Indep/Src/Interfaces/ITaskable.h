#pragma once
#include "Speed/indep/Libs/Support/Utility/UCOM.h"


namespace Sim
{

SPEED_INTERFACE ITaskable : public UTL::COM::IUnknown
{
public:
	virtual ~ITaskable();
    //virtual bool OnTask(float);
};

} // namespace Sim
