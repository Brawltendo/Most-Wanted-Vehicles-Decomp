#pragma once
#include "speedcommon.h"
#include "math/mathcommon.h"
#include "Speed/Indep/Tools/Inc/ConversionUtil.hpp"

static uint16_t bFastATanTable[] =
{
	0, 41, 81, 122, 163, 204, 244, 285, 326, 367, 407, 448, 489, 529, 570,
	610, 651, 692, 732, 773, 813, 854, 894, 935, 975, 1015, 1056, 1096,
	1136, 1177, 1217, 1257, 1297, 1337, 1377, 1417, 1457, 1497, 1537, 1577,
	1617, 1656, 1696, 1736, 1775, 1815, 1854, 1894, 1933, 1973, 2012, 2051,
	2090, 2129, 2168, 2207, 2246, 2285, 2324, 2363, 2401, 2440, 2478, 2517,
	2555, 2594, 2632, 2670, 2708, 2746, 2784, 2822, 2860, 2897, 2935, 2973,
	3010, 3047, 3085, 3122, 3159, 3196, 3233, 3270, 3307, 3344, 3380, 3417,
	3453, 3490, 3526, 3562, 3599, 3635, 3670, 3706, 3742, 3778, 3813, 3849,
	3884, 3920, 3955, 3990, 4025, 4060, 4095, 4129, 4164, 4199, 4233, 4267,
	4302, 4336, 4370, 4404, 4438, 4471, 4505, 4539, 4572, 4605, 4639, 4672,
	4705, 4738, 4771, 4803, 4836, 4869, 4901, 4933, 4966, 4998, 5030, 5062,
	5094, 5125, 5157, 5188, 5220, 5251, 5282, 5313, 5344, 5375, 5406, 5437,
	5467, 5498, 5528, 5559, 5589, 5619, 5649, 5679, 5708, 5738, 5768, 5797,
	5826, 5856, 5885, 5914, 5943, 5972, 6000, 6029, 6058, 6086, 6114, 6142,
	6171, 6199, 6227, 6254, 6282, 6310, 6337, 6365, 6392, 6419, 6446, 6473,
	6500, 6527, 6554, 6580, 6607, 6633, 6660, 6686, 6712, 6738, 6764, 6790,
	6815, 6841, 6867, 6892, 6917, 6943, 6968, 6993, 7018, 7043, 7068, 7092,
	7117, 7141, 7166, 7190, 7214, 7238, 7262, 7286, 7310, 7334, 7358, 7381,
	7405, 7428, 7451, 7475, 7498, 7521, 7544, 7566, 7589, 7612, 7635, 7657,
	7679, 7702, 7724, 7746, 7768, 7790, 7812, 7834, 7856, 7877, 7899, 7920,
	7942, 7963, 7984, 8005, 8026, 8047, 8068, 8089, 8110, 8131, 8151, 8172,
	8192, 8192, 0, 0
};

// MATCHING
uint16_t bATan(float x, float y)
{
	int quad = 0;
	if (x < 0.f)
	{
		quad = 1;
		x = -x;
	}
	float r = y;
	if (y < 0.f)
	{
		quad ^= 3;
		y = -y;
	}

	uint16_t a;
	if (x > y)
	{
		int i = ((y / x) * 65536.f);
		const uint16_t* table = &bFastATanTable[i >> 8];
		a = (table[0] + (((table[1] - table[0]) * (i & 0xFF)) >> 8));
	}
	else
	{
		if (y > x)
		{
			int i = ((x / y) * 65536.f);
			const uint16_t* table = &bFastATanTable[i >> 8];
			a = 16384 - (((table[1] - table[0]) * (i & 0xFF)) >> 8) - table[0];
		}
		else if (y == 0.f)
			a = 0;
		else
			a = 8192;
	}

	if (!quad)
		return a;
	else if (quad == 3)
		return -a;
	else if (quad == 1)
		return 32768 - a;
	else
		return 32768 + a;
}

// MATCHING
float VU0_Atan2(float opposite, float adjacent)
{
	float x = adjacent < 0.f ? -adjacent : adjacent;
	x = x > FLT_EPSILON ? adjacent : 0.f;
	float y = opposite < 0.f ? -opposite : opposite;
	y = y > FLT_EPSILON ? opposite : 0.f;

  	return (int16_t)bATan(x, y) / 65536.f;
}

// MATCHING
float Atan2d(float o, float a)
{
  	return ANGLE2DEG(VU0_Atan2(o, a));
}

// MATCHING
// Returns the greatest of 2 float values
float bMax(const float a, const float b)
{
	return a > b ? a : b;
}

// MATCHING
// Returns the smallest of 2 float values
float bMin(const float a, const float b)
{
	return a < b ? a : b;
}

// MATCHING
// Clamps a float value within a defined range
float bClamp(const float a, const float amin, const float amax)
{
	return bMin(bMax(a, amin), amax);
}

float bAbs(float a)
{
	float f_abs = fabs(a);
	return f_abs;
}
