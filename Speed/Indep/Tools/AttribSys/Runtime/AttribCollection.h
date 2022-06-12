#pragma once
#include "speedcommon.h"
#include "Speed/indep/Tools/AttribSys/Runtime/Common/AttribHashMap.h"


namespace Attrib
{
    
class Collection
{
public:
	class Node* GetNode(uint32_t attributeKey, const Collection*& container);
	void* GetData(uint32_t attributeKey, uint32_t index);

private:
	HashMap mTable;
	const Collection* mParent;
	class Class* mClass;
	void* mLayout;
	uint32_t mRefCount;
	uint32_t mKey;
	class Vault* mSource;
	const char* mNamePtr;

};

} // namespace Attrib