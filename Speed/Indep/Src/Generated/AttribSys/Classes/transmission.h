#pragma once
#include "Speed/indep/Tools/AttribSys/Runtime/AttribSys.h"

namespace Attrib
{
namespace Gen
{

struct transmission : Instance
{
    struct LayoutStruct
    {
        Private _Array_GEAR_RATIO;
        float GEAR_RATIO[9];
        Private _Array_DIFFERENTIAL;
        float DIFFERENTIAL[3];
        Private _Array_GEAR_EFFICIENCY;
        float GEAR_EFFICIENCY[9];
        float TORQUE_CONVERTER;
        float TORQUE_SPLIT;
        float CLUTCH_SLIP;
        float OPTIMAL_SHIFT;
        float SHIFT_SPEED;
        float FINAL_GEAR;
    };

	float& GEAR_RATIO(uint32_t index) const
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_GEAR_RATIO.GetLength())
			return lp->GEAR_RATIO[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_GEAR_RATIO() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_GEAR_RATIO.GetLength();
	}

	float& DIFFERENTIAL(uint32_t index) const
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_DIFFERENTIAL.GetLength())
			return lp->DIFFERENTIAL[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_DIFFERENTIAL() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_DIFFERENTIAL.GetLength();
	}

	float& GEAR_EFFICIENCY(uint32_t index) const
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_GEAR_EFFICIENCY.GetLength())
			return lp->GEAR_EFFICIENCY[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_GEAR_EFFICIENCY() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_GEAR_EFFICIENCY.GetLength();
	}

	float& TORQUE_CONVERTER() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->TORQUE_CONVERTER;
	}

	float& TORQUE_SPLIT() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->TORQUE_SPLIT;
	}

	float& CLUTCH_SLIP() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->CLUTCH_SLIP;
	}

	float& OPTIMAL_SHIFT() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->OPTIMAL_SHIFT;
	}

	float& SHIFT_SPEED() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->SHIFT_SPEED;
	}

	float& FINAL_GEAR() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->FINAL_GEAR;
	}

};

} // namespace Gen
} // namespace Attrib
