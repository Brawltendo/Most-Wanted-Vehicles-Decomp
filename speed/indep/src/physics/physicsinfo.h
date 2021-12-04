#include "generated/attribsys/classes/engine.h"
#include "generated/attribsys/classes/induction.h"

#include "physics/physicstunings.h"

namespace Physics
{	
namespace Info
{

	float InductionRPM(const Attrib::Gen::engine* const engine, const Attrib::Gen::induction* const induction, const Tunings* const tunings);
	float Torque(const Attrib::Gen::engine* const engine, const float atRPM);

} // namespace Info
} // namespace Physics