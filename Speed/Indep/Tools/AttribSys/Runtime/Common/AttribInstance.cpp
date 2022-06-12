#include "Speed/indep/Tools/AttribSys/Runtime/AttribInstance.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribCollection.h"


// MATCHING
void* Attrib::Instance::GetAttributePointer(uint32_t attribkey, uint32_t index)
{
	if (mCollection)  return mCollection->GetData(attribkey, index);
	else  return NULL;
}