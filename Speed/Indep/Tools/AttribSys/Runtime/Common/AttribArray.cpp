#include "Speed/indep/Tools/AttribSys/Runtime/AttribArray.h"


uint32_t Attrib::Array::GetPad()
{
	return (mEncodedTypePad & 0x8000) ? 8 : 0;
}

// MATCHING
void* Attrib::Array::GetData(uint32_t index)
{
	if (index < mCount)
	{
		if (!mSize)
			return &mData[index * 4 + GetPad()];
		else
			return &mData[index * mSize + GetPad()];
	}
	else
	{
		return NULL;
	}
}
	