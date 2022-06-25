#pragma once
#include "speedcommon.h"


namespace UTL { namespace COM {
	class IUnknown;
}}

namespace Attrib
{
    
struct Instance
{
	void* GetLayoutPointer() const { return mLayoutPtr; }
	void* GetLayoutPointer() { return mLayoutPtr; }
	void* GetAttributePointer(uint32_t attribkey, uint32_t index);
	
	class UTL::COM::IUnknown* mOwner;
	class Collection* const mCollection;
	void* mLayoutPtr;
	uint32_t mMsgPort;
	uint16_t mFlags;
	uint16_t mLocks;

	enum Flags
	{
		kDynamic = 1
	};

};

} // namespace Attrib
