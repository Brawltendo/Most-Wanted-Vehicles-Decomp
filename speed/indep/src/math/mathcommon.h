#pragma once
#include <math.h>

#define PI 3.14159265f
#define TWO_PI 6.2831855f
#define FLT_EPSILON 0.000001f

#define ABS(x) if (x < 0.f) x = -x
#define MPH_TO_MPS(x) (x / 2.23699f)
#define MPS_TO_MPH(x) (x * 2.23699f)
#define NORMALIZE_ANGLE_RADIANS(x) (x / TWO_PI)
#define NORMALIZE_ANGLE_DEGREES(x) (x / 360.f)
#define DEG_TO_RAD 0.017453292f

// This function was probably conditionally compiled per platform for the proper intrinsics
SPEED_NO_INLINE
float fsqrt(float x)
{
	return sqrtf(x);
}

namespace UMath
{

float Abs(const float a)
{
	return a < 0.f ? -a : a;
}

// MATCHING
// Returns the smallest of 2 float values
float Min(const float a, const float b)
{
	return a < b ? a : b;
}

// MATCHING
// Returns the smallest of 2 integer values
int Min(const int a, const int b)
{
	return a < b ? a : b;
}

// MATCHING
// Returns the smallest of 2 unsigned integer values
unsigned int Min(const unsigned int a, const unsigned int b)
{
	return a < b ? a : b;
}

// MATCHING
// Returns the greatest of 2 float values
float Max(const float a, const float b)
{
	return a > b ? a : b;
}

// MATCHING
// Returns the greatest of 2 integer values
int Max(const int a, const int b)
{
	return a > b ? a : b;
}

// MATCHING
// Returns the greatest of 2 unsigned integer values
unsigned int Max(const unsigned int a, const unsigned int b)
{
	return a > b ? a : b;
}

// MATCHING
// Clamps a float value within the range [-alimit,alimit]
float Bound(const float a, const float alimit)
{
	return Max(-alimit, Min(a, alimit));
}

// The game uses 2 clamp functions (that I know of)
// One is in the first order and the other is in the second order
// Don't know why they did that but ok

// MATCHING
// Clamps a float value within a defined range
float Clamp_(const float a, const float amin, const float amax)
{
	return Max(amin, Min(a, amax));
}

// MATCHING
// Clamps a float value within a defined range
float Clamp(const float a, const float amin, const float amax)
{
	return Min(amax, Max(a, amin));
}

// MATCHING
// Returns the interpolant (clamped to [0-1]) for the input value given a min/max range
float InverseLerp(const float val, const float low_end, const float high_end)
{
	float range = high_end - low_end;
	// range needs to be a nonzero value to avoid division errors
	// it also needs to be above zero in order to output a value between 0 and 1
	if ((range > FLT_EPSILON))
		// clamp to 0-1 range
		return Max(0.f, Min((val - low_end) / range, 1.f));
	else
		return 0.f;
}

// Can't match this because it was inlined, but this is what compiles to the right asm so I'll assume it is for now
uint32_t InterpolateIndex(uint32_t last_index, float value, float limit_min, float limit_max, float& ratio)
{
	float value_range = limit_max - limit_min;
	float value_offset = value - limit_min;
	float index = value_offset / value_range * last_index;
	uint32_t index1 = index;
	ratio = index - index1;
	return index1;
}

} // namespace UMath
