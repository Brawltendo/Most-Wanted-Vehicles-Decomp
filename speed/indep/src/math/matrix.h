#pragma once
#include "math/vector.h"

namespace UMath
{
    
class Matrix4
{
public:
	Matrix4()
    {
        // init matrix with identity
        v0 = Vector4(1.f, 0.f, 0.f, 0.f);
        v1 = Vector4(0.f, 1.f, 0.f, 0.f);
        v2 = Vector4(0.f, 0.f, 1.f, 0.f);
        v3 = Vector4(0.f, 0.f, 0.f, 1.f);
    }
    union
    {
        struct
        {
            Vector4 v0, v1, v2, v3;
        };
        float mat[16];
    };
};

} // namespace UMath
