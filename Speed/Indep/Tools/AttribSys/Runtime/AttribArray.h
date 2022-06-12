#pragma once
#include "speedcommon.h"


namespace Attrib
{

class Array
{
public:
	// Returns the base location of this array's data
	uint8_t* BasePointer() { return reinterpret_cast<uint8_t*>(&this[1]); }
	
	void* Data(uint32_t byteindex, uint8_t* base) { return &BasePointer()[byteindex + *base]; }
	bool IsReferences();
	uint32_t GetPad();
	void* GetData(uint32_t index);

private:
	uint16_t mAlloc;
	uint16_t mCount;
	uint16_t mSize;
	uint16_t mEncodedTypePad;

	uint8_t mData[];
};

} // namespace Attrib
