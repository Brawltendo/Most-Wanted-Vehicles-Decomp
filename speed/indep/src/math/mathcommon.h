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

float Abs(const float x)
{
	return x < 0.f ? -x : x;
}

// MATCHING
// Returns the smallest of 2 float values
float Min(const float a, const float b)
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
// Clamps a float value within a defined range
float Clamp(const float in, const float min, const float max)
{
	return Min(max, Max(in, min));
}

// MATCHING
// Returns the interpolant for the input value given a min/max range
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

} // namespace UMath
