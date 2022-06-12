#pragma once
#include "speedcommon.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribArray.h"


namespace Attrib
{

class Node
{
public:
	enum Flags
	{
		Flag_RequiresRelease = 1 << 0,
		Flag_IsArray         = 1 << 1,
		Flag_IsInherited     = 1 << 2,
		Flag_IsAccessor      = 1 << 3,
		Flag_IsLaidOut       = 1 << 4,
		Flag_IsByValue       = 1 << 5,
		Flag_IsLocatable     = 1 << 6,
	};

	bool GetFlag(uint32_t mask) { return mFlags & mask; }
	bool RequiresRelease() { return GetFlag(Flag_RequiresRelease); }
	bool IsArray() { return GetFlag(Flag_IsArray); }
	bool IsInherited() { return GetFlag(Flag_IsInherited); }
	bool IsAccessor() { return GetFlag(Flag_IsAccessor); }
	bool IsLaidOut() { return GetFlag(Flag_IsLaidOut); }
	bool IsByValue() { return GetFlag(Flag_IsByValue); }
	bool IsLocatable() { return GetFlag(Flag_IsLocatable); }
	bool IsValid() { return IsLaidOut() || mPtr != this; }
	void* GetPointer(void* layoutptr)
	{
		if (IsByValue())  return &mValue;
		else if (IsLaidOut())  return (void*)(uintptr_t(layoutptr) + uintptr_t(mPtr));
		else  return mPtr;
	}
	class Array* GetArray(void* layoutptr)
	{
		if (IsLaidOut())  return (Array*)(uintptr_t(layoutptr) + uintptr_t(mArray));
		else  return mArray;
	}
	uint32_t GetValue() { return mValue; }
	uint32_t* GetRefValue() { return &mValue; }
	uint32_t GetKey() { return IsValid() ? mKey : 0; }
	uint32_t MaxSearch() { return mMax; }

private:
	uint32_t mKey;
	union
	{
		void* mPtr;
		class Array* mArray;
		uint32_t mValue;
		uint32_t mOffset;
	};
	uint16_t mTypeIndex;
	uint8_t mMax;
	uint8_t mFlags;
	
};

} // namespace Attrib
