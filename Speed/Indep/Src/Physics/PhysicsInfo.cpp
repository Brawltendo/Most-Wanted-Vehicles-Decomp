#include "physics/physicsinfo.hpp"

#include "math/mathcommon.h"


// MATCHING
float Physics::Info::AerodynamicDownforce(const Attrib::Gen::chassis& chassis, const float speed)
{
	return speed * ((Attrib::Gen::chassis&)chassis).AERO_COEFFICIENT() * 2000.f;
}

// MATCHING
float Physics::Info::EngineInertia(const Attrib::Gen::engine& engine, const bool loaded)
{
	float inertia_mul;
	if (loaded)
		inertia_mul = 1.f;
	else
		inertia_mul = 0.35f;
	return inertia_mul * (((Attrib::Gen::engine&)engine).FLYWHEEL_MASS() * 0.025f + 0.25f);
}

// MATCHING
Physics::Info::eInductionType Physics::Info::InductionType(const Attrib::Gen::induction& induction)
{
	if (((Attrib::Gen::induction&)induction).HIGH_BOOST() > 0.f || ((Attrib::Gen::induction&)induction).LOW_BOOST() > 0.f)
	{
		// turbochargers don't produce significant boost until above the boost threshold (the lowest engine RPM at which it will spool up)
		// meanwhile superchargers apply boost proportionally to the engine RPM, so this param isn't needed there
		if (((Attrib::Gen::induction&)induction).SPOOL() > 0.f)
			return INDUCTION_TURBO_CHARGER;
		else
			return INDUCTION_SUPER_CHARGER;
	}
	else
		return INDUCTION_NONE;
}

// MATCHING
float Physics::Info::InductionRPM(const Attrib::Gen::engine& engine, const Attrib::Gen::induction& induction, const Tunings& tunings)
{
	float spool = ((Attrib::Gen::induction&)induction).SPOOL();

	// tune the (normalized) RPM at which forced induction kicks in
	if (&tunings && spool > 0.f)
	{
		const float turboTuning = tunings.turboTuning;
		float spoolB = turboTuning < 0.f ? spool : (1.f - spool);
		spool += (spoolB * 0.25f) * turboTuning;
	}

	return spool * (((Attrib::Gen::engine&)engine).RED_LINE() - ((Attrib::Gen::engine&)engine).IDLE()) + ((Attrib::Gen::engine&)engine).IDLE();
}

// MATCHING
float Physics::Info::WheelDiameter(const Attrib::Gen::tires& tires, bool front)
{
	int axle = front ? 0 : 1;
	return ((((Attrib::Gen::tires&)tires).ASPECT_RATIO().At(axle) * 0.01f)
		 * ((Attrib::Gen::tires&)tires).SECTION_WIDTH().At(axle)) * 0.002f
		 + ((Attrib::Gen::tires&)tires).RIM_SIZE().At(axle) * 0.0254f;
}

// MATCHING
// <@>PRINT_ASM
float Physics::Info::Speedometer(const Attrib::Gen::transmission& transmission, const Attrib::Gen::engine& engine, const Attrib::Gen::tires& tires, float rpm, GearID gear, const Tunings* tunings)
{
	float speed = 0.f;
	float gear_ratio = ((Attrib::Gen::transmission&)transmission).GEAR_RATIO(gear);
	float total_ratio = ((Attrib::Gen::transmission&)transmission).FINAL_GEAR() * gear_ratio;
	float power_range = ((Attrib::Gen::engine&)engine).RED_LINE() - ((Attrib::Gen::engine&)engine).IDLE();
	total_ratio = UMath::Abs(total_ratio);
	if (total_ratio > 0.f && power_range > 0.f)
	{
		float wheelrear = Physics::Info::WheelDiameter(tires, false);
		float wheelfront = Physics::Info::WheelDiameter(tires, true);
		float avg_wheel_radius = ((wheelrear * 0.5f) + (wheelfront * 0.5f)) * 0.5f;
		float rpm_min = ((Attrib::Gen::engine&)engine).IDLE();
		float rpm_max = ((Attrib::Gen::engine&)engine).RED_LINE();
		float clutch_rpm = (((rpm - rpm_min) / total_ratio) / power_range) * rpm_max;
		speed = RPM2RPS(avg_wheel_radius * clutch_rpm);
	}
	float limiter = MPH2MPS(((Attrib::Gen::engine&)engine).SPEED_LIMITER(0));
	if (limiter > 0.f)
		return UMath::Min(speed, limiter);
	else
		return speed;
}

float Physics::Info::Torque(const Attrib::Gen::engine& engine, const float atRPM)
{
	return 0.f;
}
