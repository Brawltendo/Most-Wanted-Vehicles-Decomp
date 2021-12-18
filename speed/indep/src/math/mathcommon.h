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

// MATCHING
/* float InverseLerp(const float val, const float low_end, const float high_end)
{
	float range = high_end - low_end;
	if ((range > FLT_EPSILON))
	{
		volatile float ramp = (val - low_end) / range;
		if (ramp < 1.f)
		{
			float ramp = ramp > 0.f ? 0.f : ramp;
			return ramp;
		}
		else return 1.f;
	}
	else return 0.f;
} */

// MATCHING
// Returns the smallest of 2 float values
_forceinline
float Min(const float a, const float b)
{
	return a < b ? a : b;
}

// MATCHING
// Returns the greatest of 2 float values
_forceinline
float Max(const float a, const float b)
{
	return a > b ? a : b;
}

// MATCHING
// Clamps a float value within a defined range
_forceinline
float Clamp(const float in, const float min, const float max)
{
	return Min(max, Max(in, min));
}

} // namespace UMath
