#pragma once
#include "speedcommon.h"
#include "Speed/indep/Tools/AttribSys/Runtime/Common/AttribHashMap.h"
#include "Speed/indep/Tools/AttribSys/Runtime/VecHashMap.h"


namespace Attrib
{

class Class
{
public:
	class ClassPrivate& GetPrivates() { return mPrivates; }

	struct TablePolicy {};

private:
	uint32_t mKey;
	uint32_t mRefCount;
	class ClassPrivate& mPrivates;

};

class ClassPrivate : public Class
{
public:
	HashMap& GetLayoutTable() { return mLayoutTable; }

	struct CollectionHashMap : public VecHashMap<uint32_t, Attrib::Collection, Class::TablePolicy, true, 40> {};

private:
	HashMap mLayoutTable;

};

} // namespace Attrib
