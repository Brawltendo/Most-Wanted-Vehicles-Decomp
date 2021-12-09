
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
		// The maximum distance the suspension can compress in inches
        float TRAVEL[2];
		// The suspension length in inches
        float RIDE_HEIGHT[2];
		// The distance a wheel should be from the center of the axle
        float TRACK_WIDTH[2];
        float SHOCK_EXT_STIFFNESS[2];
        float SHOCK_STIFFNESS[2];
        float SPRING_STIFFNESS[2];
        float SHOCK_VALVING[2];
        float SWAYBAR_STIFFNESS[2];
		// The vertical center of gravity in inches
        float ROLL_CENTER;
		// The distance from the front axle to the rear axle in inches
        float WHEEL_BASE;
        float SHOCK_BLOWOUT;
        float AERO_CG;
		// Multipler for ecar body movement behavior. Does not affect the vehicle sim
        float RENDER_MOTION;
		// The local physical position of the front axle in inches
        float FRONT_AXLE;
        float AERO_COEFFICIENT;
		// The percentage of weight that should be distributed to the front of the car
        float FRONT_WEIGHT_BIAS;
        float DRAG_COEFFICIENT;
    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib
