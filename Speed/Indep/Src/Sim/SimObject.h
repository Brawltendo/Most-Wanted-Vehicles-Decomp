#pragma once
#include "interfaces/IServiceable.h"
#include "interfaces/ITaskable.h"
#include "Speed/indep/Libs/Support/Utility/UCOM.h"


namespace Sim
{
	
class Object : public UTL::COM::Object, public IServiceable, public ITaskable
{
public:
	virtual ~Object();
private:
	uint32_t mTaskCount;
	uint32_t mServiceCount;
};

} // namespace Sim
