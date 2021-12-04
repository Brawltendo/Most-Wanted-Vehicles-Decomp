#include "physics/physicsinfo.h"

// MATCHING
float Physics::Info::InductionRPM(const Attrib::Gen::engine* const engine, const Attrib::Gen::induction* const induction, const Tunings* const tunings)
{
	float spool = induction->data->SPOOL;

	// tune the RPM at which forced induction kicks in
	if (tunings && spool > 0.f)
	{
		const float turboTuning = tunings->turboTuning;
		float spoolB = turboTuning < 0.f ? spool : (1.f - spool);
		spool += (spoolB * 0.25f) * turboTuning;
	}

	return spool * (engine->data->RED_LINE - engine->data->IDLE) + engine->data->IDLE;
}

float Physics::Info::Torque(const Attrib::Gen::engine* const engine, const float atRPM)
{
	
}