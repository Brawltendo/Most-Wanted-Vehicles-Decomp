#include "attrib/attrib.h"

namespace Attrib
{
namespace Gen
{

struct engine : Instance
{
    struct LayoutStruct
    {
        Collection _Array_TORQUE;
		float TORQUE[9];
		Collection _Array_SPEED_LIMITER;
		float SPEED_LIMITER[2];
		Collection _Array_ENGINE_BRAKING;
		float ENGINE_BRAKING[3];
		float FLYWHEEL_MASS;
		float MAX_RPM;
		float RED_LINE;
		float IDLE;
    };

	float& TORQUE(uint32_t index)
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_TORQUE.GetLength())
			return lp->TORQUE[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_TORQUE()
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_TORQUE.GetLength();
	}

	float& SPEED_LIMITER(uint32_t index)
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_SPEED_LIMITER.GetLength())
			return lp->SPEED_LIMITER[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_SPEED_LIMITER()
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_SPEED_LIMITER.GetLength();
	}

	float& ENGINE_BRAKING(uint32_t index)
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_ENGINE_BRAKING.GetLength())
			return lp->ENGINE_BRAKING[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_ENGINE_BRAKING()
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_ENGINE_BRAKING.GetLength();
	}

	float& FLYWHEEL_MASS()
	{
		return ((LayoutStruct*)GetLayoutPointer())->FLYWHEEL_MASS;
	}

	float& MAX_RPM()
	{
		return ((LayoutStruct*)GetLayoutPointer())->MAX_RPM;
	}

	float& RED_LINE()
	{
		return ((LayoutStruct*)GetLayoutPointer())->RED_LINE;
	}

	float& IDLE()
	{
		return ((LayoutStruct*)GetLayoutPointer())->IDLE;
	}

};

} // namespace Gen
} // namespace Attrib