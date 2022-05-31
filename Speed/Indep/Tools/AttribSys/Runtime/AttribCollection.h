#pragma once
#include "speedcommon.h"


namespace Attrib
{
    
struct Collection
{
	int16_t Capacity;
	int16_t Count;
	char pad_0004[4];

	int GetLength();
};

} // namespace Attrib