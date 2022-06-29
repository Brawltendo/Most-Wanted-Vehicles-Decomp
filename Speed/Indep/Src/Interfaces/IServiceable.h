#pragma once
#include "speedcommon.h"
#include "Speed/indep/Libs/Support/Utility/UCOM.h"


namespace Sim
{
	
SPEED_INTERFACE IServiceable : public UTL::COM::IUnknown
{
public:
	virtual ~IServiceable();

};

} // namespace Sim
