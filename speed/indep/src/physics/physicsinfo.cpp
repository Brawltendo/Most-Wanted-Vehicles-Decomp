#include "physics/physicsinfo.h"


// MATCHING
float Physics::Info::AerodynamicDownforce(const Attrib::Gen::chassis& chassis, const float speed)
{
	return speed * chassis.AERO_COEFFICIENT() * 2000.f;
}

// MATCHING
float Physics::Info::EngineInertia(const Attrib::Gen::engine& engine, const bool loaded)
{
	float inertia_mul;
	if (loaded)
		inertia_mul = 1.f;
	else
		inertia_mul = 0.35f;
	return inertia_mul * (engine.FLYWHEEL_MASS() * 0.025f + 0.25f);
}

// MATCHING
Physics::Info::eInductionType Physics::Info::InductionType(const Attrib::Gen::induction& induction)
{
	if (induction.HIGH_BOOST() > 0.f || induction.LOW_BOOST() > 0.f)
	{
		// turbochargers don't produce significant boost until above the boost threshold (the lowest engine RPM at which it will spool up)
		// meanwhile superchargers apply boost proportionally to the engine RPM, so this param isn't needed there
		if (induction.SPOOL() > 0.f)
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
	float spool = induction.SPOOL();

	// tune the (normalized) RPM at which forced induction kicks in
	if (&tunings && spool > 0.f)
	{
		const float turboTuning = tunings.turboTuning;
		float spoolB = turboTuning < 0.f ? spool : (1.f - spool);
		spool += (spoolB * 0.25f) * turboTuning;
	}

	return spool * (engine.RED_LINE() - engine.IDLE()) + engine.IDLE();
}

float Physics::Info::Torque(const Attrib::Gen::engine& engine, const float atRPM)
{
	return 0.f;
}