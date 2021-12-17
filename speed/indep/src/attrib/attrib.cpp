#include "attrib/attrib.h"


static uint8_t gDefaultDataArea[2048];
void* Attrib::DefaultDataArea(uint32_t bytes)
{
	return &gDefaultDataArea;
}

int16_t Attrib::Private::GetLength(const Collection& collection)
{
	return collection.Count;
}