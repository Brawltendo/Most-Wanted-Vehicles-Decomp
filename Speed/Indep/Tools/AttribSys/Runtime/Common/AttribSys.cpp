#include "Speed/indep/Tools/AttribSys/Runtime/AttribSys.h"


void* Attrib::DefaultDataArea(uint32_t bytes)
{
	return &gDefaultDataArea;
}