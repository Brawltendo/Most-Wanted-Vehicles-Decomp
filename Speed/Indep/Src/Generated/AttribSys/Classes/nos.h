#pragma once
#include "Speed/indep/Tools/AttribSys/Runtime/AttribSys.h"

namespace Attrib
{
namespace Gen
{

struct nos : Instance
{
    struct LayoutStruct
    {
		float NOS_DISENGAGE;
		float TORQUE_BOOST;
		float FLOW_RATE;
		float RECHARGE_MIN;
		float NOS_CAPACITY;
		float RECHARGE_MAX;
		float RECHARGE_MAX_SPEED;
		float RECHARGE_MIN_SPEED;
    };

	// The amount of time, in seconds, before nitrous can be used again
	float& NOS_DISENGAGE() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->NOS_DISENGAGE;
	}

	// The amount of additional torque to be added to the torque multiplier
	float& TORQUE_BOOST() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->TORQUE_BOOST;
	}

	// UNUSED
	float& FLOW_RATE() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->FLOW_RATE;
	}

	// The amount of time, in seconds, that it takes to recharge nitrous at NOS_CAPACITY_SPEED
	float& RECHARGE_MIN() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->RECHARGE_MIN;
	}

	// The maximum nitrous capacity for this car
	float& NOS_CAPACITY() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->NOS_CAPACITY;
	}

	// The amount of time, in seconds, that it takes to recharge nitrous at RECHARGE_MAX_SPEED
	float& RECHARGE_MAX() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->RECHARGE_MAX;
	}

	// The speed, in MPH, that determines when to reach RECHARGE_MAX
	float& RECHARGE_MAX_SPEED() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->RECHARGE_MAX_SPEED;
	}

	// The speed, in MPH, at which nitrous will begin recharging
	float& RECHARGE_MIN_SPEED() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->RECHARGE_MIN_SPEED;
	}

};

} // namespace Gen
} // namespace Attrib