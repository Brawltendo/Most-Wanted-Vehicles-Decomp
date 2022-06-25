#include "physics/physicsinfo.hpp"

#include "math/mathcommon.h"
#include "Speed/Indep/Tools/Inc/ConversionUtil.hpp"

#include "Speed/Indep/Tools/AttribSys/Runtime/AttribHash32.h"


//-------------------------------------------------------------------------------------
// MATCHING
float Physics::Info::AerodynamicDownforce(const Attrib::Gen::chassis& chassis, const float speed)
{
	return speed * chassis.AERO_COEFFICIENT() * 2000.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
float Physics::Info::EngineInertia(const Attrib::Gen::engine& engine, const bool loaded)
{
	float scale;
	if (loaded)
		scale = 1.f;
	else
		scale = 0.35f;
	return scale * (engine.FLYWHEEL_MASS() * 0.025f + 0.25f);
}


//-------------------------------------------------------------------------------------
// MATCHING
eInductionType Physics::Info::InductionType(const Attrib::Gen::induction& induction)
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


//-------------------------------------------------------------------------------------
// MATCHING
float Physics::Info::NosBoost(const Attrib::Gen::nos& nos, const Tunings* tunings)
{
	float boost = nos.TORQUE_BOOST();
	if (tunings) boost *= tunings->nitrousTuning * 0.25f + 1.f;
	return boost + 1.f;
}


//-------------------------------------------------------------------------------------
// MATCHING
float Physics::Info::NosCapacity(const Attrib::Gen::nos& nos, const Tunings* tunings)
{
	float capacity = nos.NOS_CAPACITY();
	if (tunings) return capacity - capacity * tunings->nitrousTuning * 0.25f;
	else return capacity;
}


//-------------------------------------------------------------------------------------
// MATCHING
float Physics::Info::InductionRPM(const Attrib::Gen::engine& engine, const Attrib::Gen::induction& induction, const Tunings* tunings)
{
	float spool = induction.SPOOL();

	// tune the (normalized) RPM at which forced induction kicks in
	if (tunings && spool > 0.f)
	{
		float value = tunings->turboTuning;
		float range = value < 0.f ? spool : (1.f - spool);
		spool += (range * 0.25f) * value;
	}

	// return the unnormalized RPM
	return spool * (engine.RED_LINE() - engine.IDLE()) + engine.IDLE();
}


//-------------------------------------------------------------------------------------
// EXTREMELY stubborn function to byte match, still functionally matches
// <@>PRINT_ASM
float Physics::Info::InductionBoost(const Attrib::Gen::engine& engine, const Attrib::Gen::induction& induction, float rpm, float spool, const Tunings* tunings, float* psi)
{
	if (psi)  *psi = 0.f;
	
	float spool_clamped = UMath::Clamp_(spool, 0.f, 1.f);
	float rpm_min = engine.IDLE();
	float rpm_max = engine.RED_LINE();
	float induction_boost = 0.f;
	float spool_rpm = InductionRPM(engine, induction, tunings);
	float high_boost = induction.HIGH_BOOST();
	float low_boost = induction.LOW_BOOST();
	float drag = induction.VACUUM();

	if (high_boost > 0.f || low_boost > 0.f)
	{
		// tuning slider adjusts the induction boost bias
		// -tuning will produce more boost in the low end, while +tuning produces more boost in the high end
		if (tunings)
		{
			float value = tunings->turboTuning;
			low_boost = low_boost - low_boost * value * 0.25f;
			high_boost = (value * 0.25f + 1.f) * high_boost;
		}

		if (rpm >= spool_rpm)
		{
			float induction_ratio = UMath::Ramp(rpm, spool_rpm, rpm_max);
			induction_boost = (1.f - induction_ratio) * low_boost + induction_ratio * high_boost;
			if (psi)
				*psi = induction.PSI() * UMath::Ramp(induction_boost, 0.f, UMath::Max(high_boost, low_boost)) * spool_clamped;
		}
		else if (drag < 0.f)
		{
			// vacuum effect applies from the min RPM to the boost RPM
			float drag_ratio = UMath::Ramp(rpm, rpm_min, spool_rpm);
			induction_boost = drag_ratio * drag;
			if (psi)
				*psi = -(induction.PSI() * UMath::Ramp(-induction_boost, 0.f, UMath::Max(high_boost, low_boost)) * drag_ratio);
		}
	}

	// this has to be the dumbest non-match I've hit so far
	// MSVC flips the multiplication order here (not that it matters functionally)
	// no matter what I do, it will NOT do it the right way
	return spool_clamped * induction_boost;
}


//-------------------------------------------------------------------------------------
// MATCHING
float Physics::Info::WheelDiameter(const Attrib::Gen::tires& tires, bool front)
{
	int axle = front ? 0 : 1;
	return ((tires.ASPECT_RATIO().At(axle) * 0.01f)
		 * tires.SECTION_WIDTH().At(axle)) * 0.002f
		 + tires.RIM_SIZE().At(axle) * 0.0254f;
}


//-------------------------------------------------------------------------------------
// MATCHING
float Physics::Info::Speedometer(const Attrib::Gen::transmission& transmission, const Attrib::Gen::engine& engine, const Attrib::Gen::tires& tires, float rpm, GearID gear, const Tunings* tunings)
{
	float speed = 0.f;
	float gear_ratio = transmission.GEAR_RATIO(gear);
	float total_ratio = transmission.FINAL_GEAR() * gear_ratio;
	float power_range = engine.RED_LINE() - engine.IDLE();
	total_ratio = UMath::Abs(total_ratio);
	if (total_ratio > 0.f && power_range > 0.f)
	{
		float wheelrear = Physics::Info::WheelDiameter(tires, false);
		float wheelfront = Physics::Info::WheelDiameter(tires, true);
		float avg_wheel_radius = ((wheelrear * 0.5f) + (wheelfront * 0.5f)) * 0.5f;
		float rpm_min = engine.IDLE();
		float rpm_max = engine.RED_LINE();
		float clutch_rpm = (((rpm - rpm_min) / total_ratio) / power_range) * rpm_max;
		speed = RPM2RPS(avg_wheel_radius * clutch_rpm);
	}
	float limiter = MPH2MPS(engine.SPEED_LIMITER(0));
	if (limiter > 0.f)
		return UMath::Min(speed, limiter);
	else
		return speed;
}


//-------------------------------------------------------------------------------------
float Physics::Info::Torque(const Attrib::Gen::engine& engine, const float atRPM)
{
	return 0.f;
}


Attrib::Gen::engine* engine;
//-------------------------------------------------------------------------------------
bool Physics::Info::EstimatePerformance(Physics::Info::Performance& perf)
{
const uint32_t hash = Attrib::StringHash32("junkman_current");
	return *reinterpret_cast<bool*>(engine->GetAttributePointer(hash, 0));
}