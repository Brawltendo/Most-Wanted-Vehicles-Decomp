#include "attrib/collection.h"
#include "math/vector.h"

struct TireEffectRecord
{
    uint32_t TireCondition;
    uint32_t EmitterClass;
    uint32_t EmitterCollection;
    float MinSpeed;
    float MaxSpeed;
};

struct RoadNoiseRecord
{
    float Frequency;
    float Amplitude;
    float MinSpeed;
    float MaxSpeed;
};

namespace Attrib
{
namespace Gen
{

struct simsurface
{
    char pad_0000[8];
    struct LayoutStruct
    {
        Collection col_TireDriveEffects;
        TireEffectRecord TireDriveEffects[3];
        Collection col_TireSlideEffects;
        TireEffectRecord TireSlideEffects[3];
        Collection col_TireSlipEffects;
        TireEffectRecord TireSlipEffects[3];
        RoadNoiseRecord RenderNoise;
        char *CollectionName;
        float GROUND_FRICTION;
        float ROLLING_RESISTANCE;
        float WORLD_FRICTION;
        float DRIVE_GRIP;
        float LATERAL_GRIP;
        float STICK;
        uint16_t WheelEffectFrequency;
        uint8_t WheelEffectIntensity;
        char pad_00FB[13];
        UMath::Vector4 DEBUG_COLOUR;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib
