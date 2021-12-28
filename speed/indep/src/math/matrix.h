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

// MATCHING
void Rotate(const Vector3& a, const Matrix4& m, Vector3& r)
{
	Vector3 v = a;
	r.x = (v.z * m.v2.x) + m.v1.x * v.y + v.x * m.v0.x;
	r.y = ((v.z * m.v2.y) + m.v1.y * v.y) + v.x * m.v0.y;
	r.z = ((v.z * m.v2.z) + m.v1.z * v.y) + v.x * m.v0.z;
}

} // namespace UMath
