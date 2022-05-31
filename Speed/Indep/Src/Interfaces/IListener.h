#pragma once
#include "speedcommon.h"


namespace Sim { namespace Collision
{

SPEED_INTERFACE IListener
{
public:
	virtual void OnCollision(const class Info&) = 0;
};

} // namespace Collision
} // namespace Sim