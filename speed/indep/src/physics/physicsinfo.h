#pragma once
#include "generated/attribsys/classes/chassis.h"
#include "generated/attribsys/classes/engine.h"
#include "generated/attribsys/classes/induction.h"

#include "physics/physicstunings.h"

namespace Physics
{	
namespace Info
{

	enum eInductionType
	{
		INDUCTION_NONE,
		INDUCTION_TURBO_CHARGER,
		INDUCTION_SUPER_CHARGER
	};

	float AerodynamicDownforce(const Attrib::Gen::chassis& chassis, const float speed);
	float EngineInertia(const Attrib::Gen::engine& engine, const bool loaded);
	eInductionType InductionType(const Attrib::Gen::induction& induction);
	float InductionRPM(const Attrib::Gen::engine& engine, const Attrib::Gen::induction& induction, const Tunings& tunings);
	float Torque(const Attrib::Gen::engine& engine, const float atRPM);

} // namespace Info
} // namespace Physics