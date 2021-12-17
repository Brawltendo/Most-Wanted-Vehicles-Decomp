#pragma once
#include "speedcommon.h"

namespace Attrib
{
    
struct Collection
{
  int16_t Capacity;
  int16_t Count;
  char pad_0004[4];
};

void* DefaultDataArea(uint32_t bytes);

namespace Private
{

	int16_t GetLength(const Collection& collection);

} // namespace Private
} // namespace Attrib
