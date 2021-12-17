#include <math.h>

#define PI 3.14159265f
#define TWO_PI 6.2831855f
#define FLT_EPSILON 0.000001f

#define ABS(x) if (x < 0.f) x = -x
#define MPH_TO_MPS(x) (x / 2.23699f)
#define MPS_TO_MPH(x) (x * 2.23699f)
#define NORMALIZE_ANGLE_RADIANS(x) (x / TWO_PI)
#define NORMALIZE_ANGLE_DEGREES(x) (x / 360.f)

// This function was probably conditionally compiled per platform for the proper intrinsics
/* SPEED_NO_INLINE
float fsqrt(float x)
{
	return sqrtf(x);
} */

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
// Returns the smallest of 2 floating point values
/* float Min(const float a, const float b)
{
	if (a < b)
		return a;
	else
		return b;
} */

// MATCHING
// Returns the greatest of 2 floating point values
/* float Max(const float a, const float b)
{
	if (a > b)
		return a;
	else
		return b;
} */

// MATCHING
// force inline here just so the output asm is easier to diff
// it'll automatically get inlined depending on where it's used otherwise, but it'll throw the offsets off in the output
_forceinline
float Clamp(const float in, const float min, const float max)
{
	float c;// = in;
	if (in > min)
		c = in;
	else
		c = min;
	if (max < c)
		c = max;
		
	/* if (c > min)
	{
		//c = in;
		if (max < c)
			c = max;
	}
	else
		c = min; */
	
	return c;
}

} // namespace UMath
