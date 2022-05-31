#pragma once
#include "speedcommon.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribCollection.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribInstance.h"


static uint8_t gDefaultDataArea[2048];

namespace Attrib
{

void* DefaultDataArea(uint32_t bytes);

} // namespace Attrib
