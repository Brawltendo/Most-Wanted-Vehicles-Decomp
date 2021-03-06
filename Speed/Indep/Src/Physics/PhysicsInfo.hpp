#pragma once
#include "generated/attribsys/classes/chassis.h"
#include "generated/attribsys/classes/engine.h"
#include "generated/attribsys/classes/induction.h"
#include "generated/attribsys/classes/nos.h"
#include "generated/attribsys/classes/tires.h"
#include "generated/attribsys/classes/transmission.h"

#include "physics/physicstypes.h"
#include "physics/physicstunings.h"

#include "Speed/Win32/Libs/STL/STLport-4.5/stlport/stl/_list.h"


namespace Physics
{	
namespace Info
{

	float AerodynamicDownforce(const Attrib::Gen::chassis& chassis, const float speed);
	float EngineInertia(const Attrib::Gen::engine& engine, const bool loaded);
	eInductionType InductionType(const Attrib::Gen::induction& induction);
	float NosBoost(const Attrib::Gen::nos& nos, const Tunings* tunings);
	float NosCapacity(const Attrib::Gen::nos& nos, const Tunings* tunings);
	float InductionRPM(const Attrib::Gen::engine& engine, const Attrib::Gen::induction& induction, const Tunings* tunings);
	float InductionBoost(const Attrib::Gen::engine& engine, const Attrib::Gen::induction& induction, float rpm, float spool, const Tunings* tunings, float* psi);
	float WheelDiameter(const Attrib::Gen::tires& tires, bool front);
	float Speedometer(const Attrib::Gen::transmission& transmission, const Attrib::Gen::engine& engine, const Attrib::Gen::tires& tires, float rpm, GearID gear, const Tunings* tunings);
	float Torque(const Attrib::Gen::engine& engine, float rpm);
	bool ShiftPoints(const Attrib::Gen::transmission& transmission, const Attrib::Gen::engine& engine, const Attrib::Gen::induction& induction, float* shift_up, float* shift_down, uint32_t numpts);

	struct Performance
	{
		Performance(float topspeed, float handling, float accel)
		{
			TopSpeed = topspeed;
			Handling = handling;
			Acceleration = accel;
		}
		void Default();

		float TopSpeed;
		float Handling;
		float Acceleration;
	};

	Performance PerformanceWeights[] = 
	{
		Performance(0.25f, 1.5f, 0.25f),
		Performance(0.f, 0.5f, 0.f),
		Performance(0.25f, 1.f, 0.2f),
		Performance(1.f, 0.f, 0.75f),
		Performance(0.5f, 0.f, 1.f),
		Performance(0.25f, 0.f, 1.25f),
		Performance(0.25f, 0.f, 1.5f)
	};

	bool EstimatePerformance(Performance& perf);

} // namespace Info
} // namespace Physics