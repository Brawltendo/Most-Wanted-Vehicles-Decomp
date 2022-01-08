#pragma once
#include "attrib/attrib.h"
#include "physics/physicstypes.h"

namespace Attrib
{
namespace Gen
{

struct chassis : Instance
{
    struct LayoutStruct
    {
        AxlePair SHOCK_DIGRESSION;
        AxlePair SPRING_PROGRESSION;
        AxlePair TRAVEL;
        AxlePair RIDE_HEIGHT;
        AxlePair TRACK_WIDTH;
        AxlePair SHOCK_EXT_STIFFNESS;
        AxlePair SHOCK_STIFFNESS;
        AxlePair SPRING_STIFFNESS;
        AxlePair SHOCK_VALVING;
        AxlePair SWAYBAR_STIFFNESS;
        float ROLL_CENTER;
        float WHEEL_BASE;
        float SHOCK_BLOWOUT;
        float AERO_CG;
        float RENDER_MOTION;
        float FRONT_AXLE;
        float AERO_COEFFICIENT;
        float FRONT_WEIGHT_BIAS;
        float DRAG_COEFFICIENT;
    };

	AxlePair& SHOCK_DIGRESSION()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SHOCK_DIGRESSION;
	}

	AxlePair& SPRING_PROGRESSION()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SPRING_PROGRESSION;
	}

	// The maximum distance the suspension can compress in inches
	AxlePair& TRAVEL()
	{
		return ((LayoutStruct*)GetLayoutPointer())->TRAVEL;
	}

	// The suspension length in inches
	AxlePair& RIDE_HEIGHT()
	{
		return ((LayoutStruct*)GetLayoutPointer())->RIDE_HEIGHT;
	}

	// The distance, in meters, between a wheel and the center of the axle
	AxlePair& TRACK_WIDTH()
	{
		return ((LayoutStruct*)GetLayoutPointer())->TRACK_WIDTH;
	}

	// The maximum damping force when extending, in lbf/in
	AxlePair& SHOCK_EXT_STIFFNESS()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SHOCK_EXT_STIFFNESS;
	}

	// The maximum damping force in lbf/in
	AxlePair& SHOCK_STIFFNESS()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SHOCK_STIFFNESS;
	}

	// The maximum spring force when compressing, in lbf/in
	AxlePair& SPRING_STIFFNESS()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SPRING_STIFFNESS;
	}

	// The length of the shocks in inches
	AxlePair& SHOCK_VALVING()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SHOCK_VALVING;
	}

	// The maximum swaybar force in lbf/in
	AxlePair& SWAYBAR_STIFFNESS()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SWAYBAR_STIFFNESS;
	}

	// The vertical center of gravity in inches
	float& ROLL_CENTER()
	{
		return ((LayoutStruct*)GetLayoutPointer())->ROLL_CENTER;
	}

	// The distance, in meters, from the front axle to the rear axle
	float& WHEEL_BASE()
	{
		return ((LayoutStruct*)GetLayoutPointer())->WHEEL_BASE;
	}

	// Scales the maximum amount of force that the shocks can absorb before they're unable to dampen it
	// The maximum force is determined by the vehicle mass * gravity (9.81)
	float& SHOCK_BLOWOUT()
	{
		return ((LayoutStruct*)GetLayoutPointer())->SHOCK_BLOWOUT;
	}

	// The front/rear bias of where downforce should be applied, represented as a percentage
	// A value of 50 means that downforce will be applied directly in the center of the rigidbody
	float& AERO_CG()
	{
		return ((LayoutStruct*)GetLayoutPointer())->AERO_CG;
	}

	// Multipler for ecar body movement behavior. Does not affect the vehicle sim
	float& RENDER_MOTION()
	{
		return ((LayoutStruct*)GetLayoutPointer())->RENDER_MOTION;
	}

	// The local physical position, in meters, of the front axle
	float& FRONT_AXLE()
	{
		return ((LayoutStruct*)GetLayoutPointer())->FRONT_AXLE;
	}

	// Influences the amount of downforce applied to the chassis
	float& AERO_COEFFICIENT()
	{
		return ((LayoutStruct*)GetLayoutPointer())->AERO_COEFFICIENT;
	}

	// The front/rear weight distribution bias, represented as a percentage
	// A value of 50 makes for a 50/50 split. Values above 50 will be front heavy, while below 50 will be rear heavy
	float& FRONT_WEIGHT_BIAS()
	{
		return ((LayoutStruct*)GetLayoutPointer())->FRONT_WEIGHT_BIAS;
	}

	// Influences the amount of drag applied to the chassis
	float& DRAG_COEFFICIENT()
	{
		return ((LayoutStruct*)GetLayoutPointer())->DRAG_COEFFICIENT;
	}

};

} // namespace Gen
} // namespace Attrib
