#pragma once
#include "speedcommon.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribNode.h"


namespace Attrib
{

// Rotates (v) by (amount) bits
uint32_t RotateNTo32(uint32_t v, uint32_t amount) { return (v << amount) | (v >> (32 - amount)); }

class HashMap
{
public:
	bool ValidIndex(uint32_t index) { return index < mTableSize && mTable[index].IsValid(); }
	uint32_t FindIndex(uint32_t key);
	class Node* Find(uint32_t key)
	{
		if (!key)  return 0;

		uint32_t index = FindIndex(key);
		return ValidIndex(index) ? &mTable[index] : NULL;
	}

	struct HashMapTablePolicy
	{
		static uint32_t KeyIndex(uint32_t k, uint32_t tableSize, uint32_t keyShift) { return RotateNTo32(k, keyShift) % tableSize; }
	};

private:
	class Node* mTable;
	uint32_t mTableSize;
	uint32_t mNumEntries;
	uint16_t mWorstCollision;
	uint16_t mKeyShift;
};

} // namespace Attrib
