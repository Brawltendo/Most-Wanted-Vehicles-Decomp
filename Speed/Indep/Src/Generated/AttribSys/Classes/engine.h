#pragma once
#include "Speed/indep/Tools/AttribSys/Runtime/AttribSys.h"

namespace Attrib
{
namespace Gen
{

struct engine : Instance
{
    struct LayoutStruct
    {
        Private _Array_TORQUE;
		float TORQUE[9];
		Private _Array_SPEED_LIMITER;
		float SPEED_LIMITER[2];
		Private _Array_ENGINE_BRAKING;
		float ENGINE_BRAKING[3];
		float FLYWHEEL_MASS;
		float MAX_RPM;
		float RED_LINE;
		float IDLE;
    };

	float& TORQUE(uint32_t index) const
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_TORQUE.GetLength())
			return lp->TORQUE[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_TORQUE() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_TORQUE.GetLength();
	}

	float& SPEED_LIMITER(uint32_t index) const
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_SPEED_LIMITER.GetLength())
			return lp->SPEED_LIMITER[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_SPEED_LIMITER() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_SPEED_LIMITER.GetLength();
	}

	float& ENGINE_BRAKING(uint32_t index) const
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_ENGINE_BRAKING.GetLength())
			return lp->ENGINE_BRAKING[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_ENGINE_BRAKING() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_ENGINE_BRAKING.GetLength();
	}

	float& FLYWHEEL_MASS() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->FLYWHEEL_MASS;
	}

	float& MAX_RPM() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->MAX_RPM;
	}

	float& RED_LINE() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->RED_LINE;
	}

	float& IDLE() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->IDLE;
	}

};

} // namespace Gen
} // namespace Attrib