#include "Speed/indep/Tools/AttribSys/Runtime/Common/AttribHashMap.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribNode.h"


// bool Attrib::HashMap::ValidIndex(uint32_t index)


// MATCHING
uint32_t Attrib::HashMap::FindIndex(uint32_t key)
{
	if (!mNumEntries || !key)
		return mTableSize;

	uint32_t searchLen = mTableSize;
	Node* table = mTable;
	uint32_t actualIndex = HashMapTablePolicy::KeyIndex(key, searchLen, mKeyShift);
	uint32_t maxSearchlen = table[actualIndex].MaxSearch();

	for (int i = 0; i < maxSearchlen; ++i)
	{
		// found matching key, so stop searching
		if (table[actualIndex].GetKey() == key)
			break;
		else
			actualIndex = (actualIndex + 1) % searchLen;
	}

	return table[actualIndex].GetKey() == key ? actualIndex : searchLen;
}

// MATCHING
/* Attrib::Node* Attrib::HashMap::Find(uint32_t key)
{
	if (!key)  return 0;

	uint32_t index = FindIndex(key);
	return ValidIndex(index) ? &mTable[index] : NULL;
} */