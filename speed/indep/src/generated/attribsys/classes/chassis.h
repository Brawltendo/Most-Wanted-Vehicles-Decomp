
namespace Attrib
{
namespace Gen
{

struct chassis
{
    char pad_0000[8];
    struct LayoutStruct
    {
        float SHOCK_DIGRESSION[2];
        float SPRING_PROGRESSION[2];
        float TRAVEL[2];
        float RIDE_HEIGHT[2];
        float TRACK_WIDTH[2];
        float SHOCK_EXT_STIFFNESS[2];
        float SHOCK_STIFFNESS[2];
        float SPRING_STIFFNESS[2];
        float SHOCK_VALVING[2];
        float SWAYBAR_STIFFNESS[2];
        float ROLL_CENTER;
        float WHEEL_BASE;
        float SHOCK_BLOWOUT;
        float AERO_CG;
        float RENDER_MOTION;
        float FRONT_AXLE;
        float AERO_COEFFICIENT;
        float FRONT_WEIGHT_BIAS;
        float DRAG_COEFFICIENT;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib
