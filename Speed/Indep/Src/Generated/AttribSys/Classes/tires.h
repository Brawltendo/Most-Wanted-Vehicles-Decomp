#pragma once
#include "Speed/indep/Tools/AttribSys/Runtime/AttribSys.h"
#include "physics/physicstypes.h"

namespace Attrib
{
namespace Gen
{

struct tires : Instance
{
    struct LayoutStruct
    {
        Private _Array_YAW_CONTROL;
        float YAW_CONTROL[4];
        AxlePair GRIP_SCALE;
        AxlePair DYNAMIC_GRIP;
        AxlePair ASPECT_RATIO;
        AxlePair RIM_SIZE;
        AxlePair STATIC_GRIP;
        AxlePair SECTION_WIDTH;
        float STEERING;
        float YAW_SPEED;
    };

	float& YAW_CONTROL(uint32_t index) const
	{
		LayoutStruct* const lp = (LayoutStruct*)GetLayoutPointer();
		if (index < lp->_Array_YAW_CONTROL.GetLength())
			return lp->YAW_CONTROL[index];
		else
			return *(float*)DefaultDataArea(sizeof(float));
	}
	unsigned int Num_YAW_CONTROL() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->_Array_YAW_CONTROL.GetLength();
	}

	AxlePair& GRIP_SCALE() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->GRIP_SCALE;
	}

	AxlePair& DYNAMIC_GRIP() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->DYNAMIC_GRIP;
	}

	AxlePair& ASPECT_RATIO() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->ASPECT_RATIO;
	}

	AxlePair& RIM_SIZE() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->RIM_SIZE;
	}

	AxlePair& STATIC_GRIP() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->STATIC_GRIP;
	}

	AxlePair& SECTION_WIDTH() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->SECTION_WIDTH;
	}

	float& STEERING() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->STEERING;
	}

	float& YAW_SPEED() const
	{
		return ((LayoutStruct*)GetLayoutPointer())->YAW_SPEED;
	}

};

} // namespace Gen
} // namespace Attrib
