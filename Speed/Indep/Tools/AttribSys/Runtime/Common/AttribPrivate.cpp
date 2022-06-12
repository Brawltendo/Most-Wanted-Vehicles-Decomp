#include "Speed/indep/Tools/AttribSys/Runtime/AttribPrivate.h"


int Attrib::Private::GetLength()
{
	return *reinterpret_cast<uint16_t*>(&mData[2]);
}