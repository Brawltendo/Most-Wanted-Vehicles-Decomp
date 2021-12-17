#include "attrib/attrib.h"

namespace Attrib
{
namespace Gen
{

struct transmission
{
    char pad_0000[8];
    struct LayoutStruct
    {
        Collection _Array_GEAR_RATIO;
        float GEAR_RATIO[9];
        Collection _Array_DIFFERENTIAL;
        float DIFFERENTIAL[3];
        Collection _Array_GEAR_EFFICIENCY;
        float GEAR_EFFICIENCY[9];
        float TORQUE_CONVERTER;
        float TORQUE_SPLIT;
        float CLUTCH_SLIP;
        float OPTIMAL_SHIFT;
        float SHIFT_SPEED;
        float FINAL_GEAR;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib
