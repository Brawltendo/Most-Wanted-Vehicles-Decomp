#include "attrib/attrib.h"
#include "physics/physicstypes.h"

namespace Attrib
{
namespace Gen
{

struct tires
{
    char pad_0000[8];
    struct LayoutStruct
    {
        Collection _Array_YAW_CONTROL;
        float YAW_CONTROL[4];
        AxlePair GRIP_SCALE;
        AxlePair DYNAMIC_GRIP;
        AxlePair ASPECT_RATIO;
        AxlePair RIM_SIZE;
        AxlePair STATIC_GRIP;
        AxlePair SECTION_WIDTH;
        float STEERING;
        float YAW_SPEED;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib
