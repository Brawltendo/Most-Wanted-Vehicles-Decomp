#include "attrib/collection.h"

namespace Attrib
{
namespace Gen
{

struct engine
{
    char pad_0000[8];
    struct LayoutStruct
    {
        Collection col_TORQUE;
		float TORQUE[9];
		Collection col_SPEED_LIMITER;
		float SPEED_LIMITER[2];
		Collection col_ENGINE_BRAKING;
		float ENGINE_BRAKING[3];
		float FLYWHEEL_MASS;
		float MAX_RPM;
		float RED_LINE;
		float IDLE;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib