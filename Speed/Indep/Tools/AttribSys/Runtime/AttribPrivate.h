#pragma once
#include "speedcommon.h"


namespace Attrib
{
    
class Private
{
public:
	int GetLength();
private:
	uint8_t mData[8];
};

} // namespace Attrib