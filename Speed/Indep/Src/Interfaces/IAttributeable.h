#pragma once
#include "Speed/indep/Tools/AttribSys/Runtime/AttribCollection.h"


SPEED_INTERFACE IAttributeable
{
public:
	virtual ~IAttributeable();
	virtual void OnAttributeChange(const Attrib::Collection*, uint32_t) = 0;
};