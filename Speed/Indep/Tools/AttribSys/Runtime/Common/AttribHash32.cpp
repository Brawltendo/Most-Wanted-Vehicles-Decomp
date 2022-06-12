#include "Speed/Indep/Tools/AttribSys/Runtime/AttribHash32.h"
#include <string.h>


// FUNCTIONAL MATCH
// seems to want __fastcall in order to byte match
// function prologue is also being very stubborn, probably needs whole program optimizations on
uint32_t __fastcall Attrib::hash32(const unsigned char *k, uint32_t initval, uint32_t length)
{
    uint32_t len = length;
    uint32_t a = 0x9E3779B9u;
    uint32_t b = 0x9E3779B9u;
    uint32_t c = initval;

	// loop through string chars in 12 byte chunks
	for (; len >= 12; k += 12, len -= 12)
    {
		// split 12 byte chunk into 3 uints
		// each 8 bits of the uint, from lowest to highest, stores a char

        a += k[0] + (k[1] << 8) + (k[2] << 16) + (k[3] << 24);
        b += k[4] + (k[5] << 8) + (k[6] << 16) + (k[7] << 24);
        c += k[8] + (k[9] << 8) + (k[10] << 16) + (k[11] << 24);

        a = a - b - c; a ^= (c >> 13);
        b = b - c - a; b ^= (a << 8);
        c = c - a - b; c ^= (b >> 13);

        a = a - b - c; a ^= (c >> 12);
        b = b - c - a; b ^= (a << 16);
        c = c - a - b; c ^= (b >> 5);

        a = a - b - c; a ^= (c >> 3);
        b = b - c - a; b ^= (a << 10);
        c = c - a - b; c ^= (b >> 15);
	}
    
    c += length;
	// finish any remaining chars or the entire string if it's less than 12 bytes
	// we can just fall through the switch cases to do this
    switch (len)
    {
        case 11:
            c += k[10] << 24;
        case 10:
            c += k[9] << 16;
        case 9:
            c += k[8] << 8;
        case 8:
            b += k[7] << 24;
        case 7:
            b += k[6] << 16;
        case 6:
            b += k[5] << 8;
        case 5:
            b += k[4];
        case 4:
            a += k[3] << 24;
        case 3:
            a += k[2] << 16;
        case 2:
            a += k[1] << 8;
        case 1:
            a += k[0];
            break;
	}
    
    a = a - b - c; a ^= (c >> 13);
    b = b - c - a; b ^= (a << 8);
    c = c - a - b; c ^= (b >> 13);
    a = a - b - c; a ^= (c >> 12);
    b = b - c - a; b ^= (a << 16);
    c = c - a - b; c ^= (b >> 5);
    a = a - b - c; a ^= (c >> 3);
    b = b - c - a; b ^= (a << 10);
    c = c - a - b; c ^= (b >> 15);
    return c;
}

uint32_t Attrib::StringHash32(const char *k)
{
	if (k && k[0])
		return hash32(reinterpret_cast<const unsigned char*>(k), 0x0ABCDEF00u, strlen(k));
	else
		return 0;
}