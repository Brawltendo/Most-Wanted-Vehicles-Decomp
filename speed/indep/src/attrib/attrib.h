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

struct Instance
{
	void* GetLayoutPointer() { return mLayoutPtr; }
	struct IUnknown* mOwner;
	Collection* const mCollection;
	void* mLayoutPtr;
	uint32_t mMsgPort;
	uint16_t mFlags;
	uint16_t mLocks;
};

void* DefaultDataArea(uint32_t bytes);

namespace Private
{

	//int GetLength(const Collection& collection) { return collection.Count; }

} // namespace Private
} // namespace Attrib
