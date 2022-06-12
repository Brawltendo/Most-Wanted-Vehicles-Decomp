#pragma once
#include "speedcommon.h"


namespace Attrib
{
    
uint32_t __fastcall hash32(const unsigned char *k, uint32_t initval, uint32_t length);
uint32_t StringHash32(const char *k);

} // namespace Attrib