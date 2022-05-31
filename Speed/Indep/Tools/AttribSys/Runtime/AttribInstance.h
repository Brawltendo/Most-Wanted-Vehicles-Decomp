#pragma once
#include "speedcommon.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribCollection.h"


namespace Attrib
{
    
struct Instance
{
	void* GetLayoutPointer() { return mLayoutPtr; }
	
	class IUnknown* mOwner;
	Collection* const mCollection;
	void* mLayoutPtr;
	uint32_t mMsgPort;
	uint16_t mFlags;
	uint16_t mLocks;
};

} // namespace Attrib
