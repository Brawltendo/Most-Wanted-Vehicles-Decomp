#include <math.h>

#define PI 3.14159265f
#define TWO_PI 6.2831855f
#define FLT_EPSILON 0.000001f

#define ABS(x) if (x < 0.f) x = -x
#define MPH_TO_MPS(x) (x / 2.23699f)
#define MPS_TO_MPH(x) (x * 2.23699f)
#define NORMALIZE_ANGLE_RADIANS(x) (x / TWO_PI)
#define NORMALIZE_ANGLE_DEGREES(x) (x / 360.f)

/* float _cdecl fsqrt(float x)
{
	// use inline asm here because the compiler is failing me with the intrinsics lol
	__asm fld [esp+4]
	__asm fsqrt
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

} // namespace UMath
