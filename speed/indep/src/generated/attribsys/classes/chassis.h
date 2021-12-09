
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

		// The distance, in meters, between a wheel and the center of the axle
        float TRACK_WIDTH[2];

		// The maximum spring force when extending, in lbf/in
        float SHOCK_EXT_STIFFNESS[2];

		// The maximum damping force in lbf/in
        float SHOCK_STIFFNESS[2];

		// The maximum spring force when compressing, in lbf/in
        float SPRING_STIFFNESS[2];

		// The length of the shocks in inches
        float SHOCK_VALVING[2];

		// The maximum swaybar force in lbf/in
        float SWAYBAR_STIFFNESS[2];

		// The vertical center of gravity in inches
        float ROLL_CENTER;

		// The distance, in meters, from the front axle to the rear axle
        float WHEEL_BASE;

		// Scales the maximum amount of force that the shocks can absorb before they're unable to dampen it
		// The maximum force is determined by the vehicle mass * gravity (9.81)
        float SHOCK_BLOWOUT;

		// The front/rear bias of where downforce should be applied, represented as a percentage
		// A value of 50 means that downforce will be applied directly in the center of the rigidbody
        float AERO_CG;

		// Multipler for ecar body movement behavior. Does not affect the vehicle sim
        float RENDER_MOTION;

		// The local physical position, in meters, of the front axle
        float FRONT_AXLE;

		// Influences the amount of downforce applied to the chassis
        float AERO_COEFFICIENT;

		// The front/rear weight distribution bias, represented as a percentage
		// A value of 50 makes for a 50/50 split. Values above 50 will be front heavy, while below will be rear heavy
        float FRONT_WEIGHT_BIAS;

		// Influences the amount of drag applied to the chassis
        float DRAG_COEFFICIENT;

    } *data;
    char pad_000C[8];
};

} // namespace Gen
} // namespace Attrib
