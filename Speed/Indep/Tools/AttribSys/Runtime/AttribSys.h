#pragma once
#include "speedcommon.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribCollection.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribInstance.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribPrivate.h"


static uint8_t gDefaultDataArea[2048];

namespace Attrib
{

const int32_t kTypeHandlerCount = 7;
uint32_t kTypeHandlerIds[kTypeHandlerCount] = 
{
	0x2B936EB7u, 0xAA229CD7u, 0x341F03A0u, 0x600994C4u, 0x681D219Cu, 0x5FDE6463u, 0x57D382C9u
};

void* DefaultDataArea(uint32_t bytes);

class RefSpec
{
public:
	uint32_t GetClassKey() { return mClassKey; }
	uint32_t GetCollectionKey() { return mCollectionKey; }

private:
	uint32_t mClassKey;
	uint32_t mCollectionKey;
	const Collection* mCollectionPtr;

};

} // namespace Attrib
