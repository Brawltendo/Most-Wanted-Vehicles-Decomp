#include "attrib/attrib.h"

namespace Attrib
{
namespace Gen
{

struct nos
{
    char pad_0000[8];
    struct LayoutStruct
    {
		// The amount of time, in seconds, before nitrous can be used again
		float NOS_DISENGAGE;

		// The amount of additional torque to be added to the torque multiplier
		float TORQUE_BOOST;

		// UNUSED
		float FLOW_RATE;

		// The amount of time, in seconds, that it takes to recharge nitrous at RECHARGE_MIN_SPEED
		float RECHARGE_MIN;

		// The maximum nitrous capacity for this car
		float NOS_CAPACITY;

		// The amount of time, in seconds, that it takes to recharge nitrous at RECHARGE_MAX_SPEED
		float RECHARGE_MAX;

		// The speed, in MPH, that determines when to reach RECHARGE_MAX
		float RECHARGE_MAX_SPEED;

		// The speed, in MPH, at which nitrous will begin recharging
		float RECHARGE_MIN_SPEED;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib