#pragma once
#include "stddef.h"

typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
//typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
//typedef unsigned long long uint64_t;

#define SPEED_NO_INLINE _declspec(noinline)

#define CHECK_FLAG_ON(in_flag, check) (in_flag & check) != 0