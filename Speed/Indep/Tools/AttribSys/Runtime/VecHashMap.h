#pragma once
#include "speedcommon.h"


template<class HashType, class ClassType, class TablePolicy, bool Fixed, int Count>
class VecHashMap
{
public:

private:
	ClassType* mTable;
	uint32_t mTableSize;
	uint32_t mNumEntries;
	uint32_t mFixedAlloc : 1;
	uint32_t mWorstCollision : 31;
};