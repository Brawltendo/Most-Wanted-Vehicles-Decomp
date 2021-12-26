#include "attrib/attrib.h"


static uint8_t gDefaultDataArea[2048];
void* Attrib::DefaultDataArea(uint32_t bytes)
{
	return &gDefaultDataArea;
}

int Attrib::Collection::GetLength()
{
	return Count;
}