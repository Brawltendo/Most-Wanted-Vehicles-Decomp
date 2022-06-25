#pragma once
#include "speedcommon.h"


namespace UTL { namespace Collections {

template<class HandleType, class InstanceClass>
class Instanceable
{
public:
	// MATCHING
	HandleType GetInstanceHandle() { return reinterpret_cast<HandleType>(_mHandle); }

private:
	uint32_t _mHandle;
	static uint32_t _mHNext;
};

} // namespace Collections 
} // namespace UTL