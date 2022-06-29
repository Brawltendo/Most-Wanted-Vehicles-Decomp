#pragma once
#include <vector>
#include "speedcommon.h"


namespace UTL { namespace COM {

SPEED_INTERFACE IUnknown
{
public:
		virtual ~IUnknown();

private:
	class Object* _mCOMObject;

};

class Object
{

	struct _IPair
	{
		void* handle;
		IUnknown* ref;
	};

	struct _IList// : public std::vector<_IPair, _type_UComObject>
	{
		int pad;
		void* _M_start;
		void* _M_finish;
		void* _M_end_of_storage;
	};

	_IList _mInterfaces;

};

class Factory
{
	int dummy;
};

} // namespace COM 
} // namespace UTL

typedef UTL::COM::Object _type_UComObject;