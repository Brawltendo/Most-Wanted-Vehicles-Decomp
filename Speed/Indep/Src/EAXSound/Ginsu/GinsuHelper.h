#pragma once
#include "speedcommon.h"


/*
 *  NOTE:
 *  The Ginsu library was likely compiled using the /QIfist option, so the inline asm in these helper functions probably isn't necessary
 *  For our purposes here, we'll keep it since it works and these can easily be replaced with sane functions
 */

// MATCHING
int IntRound(float x)
{
	int i;
	__asm
	{
		fld dword ptr x   // load x
		fistp dword ptr i // store (int)x in i and pop
	}
	return i;
	// in other words:
	// return (int)x;
}

// MATCHING
int IntCeil(float x)
{
	int i;
	int temp;
	__asm
	{
		fld dword ptr x     // load x
		fist dword ptr i    // store (int)x in i
		fisubr dword ptr i  // (int)x - x
		fstp dword ptr temp // store x in temp
	}
	// note that we're storing the temp remainder from above as an int
	return i - (temp >> 31);
}

// MATCHING
int IntFloor(float x)
{
	int i;
	int temp;
	__asm
	{
		fld dword ptr x     // load x
		fist dword ptr i    // store (int)x in i
		fisub dword ptr i   // x - (int)x
		fstp dword ptr temp // store x in temp
	}
	// note that we're storing the temp remainder from above as an int
	return i + (temp >> 31);
}

// MATCHING
short convertsample(float x)
{
	short val;
	__asm
	{
		fld dword ptr x    // load x
		fistp word ptr val // store (short)x in val
	}
	if ((unsigned short)val == 0x8000)
		return (*(unsigned int*)&x >> 31) + 0x7FFF; // type pun to get the sign bit
	return val;
}