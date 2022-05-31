// Basic vector structs and helper functions
#pragma once
#include "math/mathcommon.h"

namespace UMath
{

struct Vector2
{
	static const Vector2 kZero;

	float x, y;

	Vector2()
    {
        /*
		x = 0.f;
        y = 0.f;
		*/
    }

	float& operator[](int index)
	{
		return (&x)[index];
	}

    Vector2(const float f)
    {
        x = f;
        y = f;
    }

    Vector2(const float fx, const float fy)
    {
        x = fx;
        y = fy;
    }

    Vector2 operator+(const Vector2& b)
    {
        Vector2 v;
        v.x = this->x + b.x;
        v.y = this->y + b.y;
        return v;
    }

    Vector2 operator+(const float b)
    {
        Vector2 v;
        v.x = this->x + b;
        v.y = this->y + b;
        return v;
    }

    Vector2 operator-(const Vector2& b)
    {
        Vector2 v;
        v.x = this->x - b.x;
        v.y = this->y - b.y;
        return v;
    }

    Vector2 operator-(const float b)
    {
        Vector2 v;
        v.x = this->x - b;
        v.y = this->y - b;
        return v;
    }

    Vector2 operator*(const Vector2& b)
    {
        Vector2 v;
        v.x = this->x * b.x;
        v.y = this->y * b.y;
        return v;
    }

    Vector2 operator*(const float b)
    {
        Vector2 v;
        v.x = this->x * b;
        v.y = this->y * b;
        return v;
    }

    Vector2 operator/(const Vector2& b)
    {
        Vector2 v;
        v.x = this->x / b.x;
        v.y = this->y / b.y;
        return v;
    }

    Vector2 operator/(const float b)
    {
        Vector2 v;
        v.x = this->x / b;
        v.y = this->y / b;
        return v;
    }

    Vector2 operator=(const float b)
    {
        Vector2 v;
        v.x = b;
        v.y = b;
        return v;
    }

};

const Vector2 Vector2::kZero = (0.0f);

struct Vector3
{
	static const Vector3 kZero;

	float x, y, z;

	Vector3()
    {
        /*
		x = 0.f;
        y = 0.f;
        z = 0.f;
		*/
    }

	float& operator[](int index)
	{
		return (&x)[index];
	}

	Vector3(const Vector3& From)
	{
		x = From.x;
		y = From.y;
		z = From.z;
	}

    Vector3(const float f)
    {
        x = f;
        y = f;
        z = f;
    }

    Vector3(const float fx, const float fy, const float fz)
    {
        x = fx;
        y = fy;
        z = fz;
    }

	/* Vector3& operator=(const Vector3& From)
    {
        x = From.x;
        y = From.y;
        z = From.z;
        return *this;
    } */

	Vector3& operator*=(const Vector3& b)
    {
        x *= b.x;
        y *= b.y;
        z *= b.z;
        return *this;
    }

	Vector3& operator/=(const float scalar)
    {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

	Vector3& operator*=(const float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

	Vector3& operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

	Vector3& operator-=(const Vector3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3 operator+(const Vector3& v)
    {
        Vector3 result;
        result.x = x + v.x;
        result.y = y + v.y;
        result.z = z + v.z;
        return result;
    }

    Vector3 operator+(const float b)
    {
        Vector3 v;
        v.x = this->x + b;
        v.y = this->y + b;
        v.z = this->z + b;
        return v;
    }

    Vector3 operator-(const Vector3& b)
    {
        Vector3 v;
        v.x = this->x - b.x;
        v.y = this->y - b.y;
        v.z = this->z - b.z;
        return v;
    }

    Vector3 operator-(const float b)
    {
        Vector3 v;
        v.x = this->x - b;
        v.y = this->y - b;
        v.z = this->z - b;
        return v;
    }

    Vector3 operator*(const Vector3& b)
    {
        Vector3 v;
        v.x = this->x * b.x;
        v.y = this->y * b.y;
        v.z = this->z * b.z;
        return v;
    }

    Vector3 operator*(const float b)
    {
        Vector3 v;
        v.x = this->x * b;
        v.y = this->y * b;
        v.z = this->z * b;
        return v;
    }

    Vector3 operator/(const Vector3& b)
    {
        Vector3 v;
        v.x = this->x / b.x;
        v.y = this->y / b.y;
        v.z = this->z / b.z;
        return v;
    }

    Vector3 operator/(const float b)
    {
        Vector3 v;
        v.x = this->x / b;
        v.y = this->y / b;
        v.z = this->z / b;
        return v;
    }

    Vector3 operator=(const float b)
    {
        Vector3 v;
        v.x = b;
        v.y = b;
        v.z = b;
        return v;
    }

	Vector3& Vector3::operator=(const struct Vector4& b);

};

const Vector3 Vector3::kZero = (0.0f);

struct Vector4
{
	static const Vector4 kZero;
	static const Vector4 kIdentity;

	float x, y, z, w;

    Vector4()
    {
        /* x = 0.f;
        y = 0.f;
        z = 0.f;
        w = 0.f; */
    }

	float& operator[](int index)
	{
		return (&x)[index];
	}

    Vector4(const float in)
    {
        x = in;
        y = in;
        z = in;
        w = in;
    }

    Vector4(const float inX, const float inY, const float inZ, const float inW)
    {
        x = inX;
        y = inY;
        z = inZ;
        w = inW;
    }

	Vector4(const Vector3& v, const float inW)
    {
        w = inW;
        x = v.x;
        y = v.y;
        z = v.z;
    }


    Vector4 operator+(const Vector4& b)
    {
        Vector4 v;
        v.x = this->x + b.x;
        v.y = this->y + b.y;
        v.z = this->z + b.z;
        v.w = this->w + b.z;
        return v;
    }

    Vector4 operator+(const float b)
    {
        Vector4 v;
        v.x = this->x + b;
        v.y = this->y + b;
        v.z = this->z + b;
        v.w = this->w + b;
        return v;
    }

    Vector4 operator-(const Vector4& b)
    {
        Vector4 v;
        v.x = this->x - b.x;
        v.y = this->y - b.y;
        v.z = this->z - b.z;
        v.w = this->w - b.z;
        return v;
    }

    Vector4 operator-(const float b)
    {
        Vector4 v;
        v.x = this->x - b;
        v.y = this->y - b;
        v.z = this->z - b;
        v.w = this->w - b;
        return v;
    }

    Vector4 operator*(const Vector4& b)
    {
        Vector4 v;
        v.x = this->x * b.x;
        v.y = this->y * b.y;
        v.z = this->z * b.z;
        v.w = this->w * b.z;
        return v;
    }

    Vector4 operator*(const float b)
    {
        Vector4 v;
        v.x = this->x * b;
        v.y = this->y * b;
        v.z = this->z * b;
        v.w = this->w * b;
        return v;
    }

    Vector4 operator/(const Vector4& b)
    {
        Vector4 v;
        v.x = this->x / b.x;
        v.y = this->y / b.y;
        v.z = this->z / b.z;
        v.w = this->w / b.z;
        return v;
    }

    Vector4 operator/(const float b)
    {
        Vector4 v;
        v.x = this->x / b;
        v.y = this->y / b;
        v.z = this->z / b;
        v.w = this->w / b;
        return v;
    }

    Vector4 operator=(const float b)
    {
        Vector4 v;
        v.x = b;
        v.y = b;
        v.z = b;
        v.w = b;
        return v;
    }

	Vector4& operator=(const Vector3& b) 
	{
	    x = b.x;
		y = b.y;
		z = b.z;
	    return *this;
	}

};

const Vector4 Vector4::kZero = (0.0f);
const Vector4 Vector4::kIdentity = (0.0f, 0.0f, 0.0f, 1.f);

Vector3& Vector3::operator=(const Vector4& b) 
{
    x = b.x;
	y = b.y;
	z = b.z;
    return *this;
}

Vector3& Vector4To3(Vector4& c4)
{
	return (Vector3&)c4;
}

Vector3& Vector4To3(const Vector4& c4)
{
	return (Vector3&)c4;
}

// MATCHING
float Dot(const Vector3& a, const Vector3& b)
{
	return (a.z * b.z) + a.y * b.y + a.x * b.x;
}

/* void Cross(const Vector3& a, const Vector3& b, Vector3& r)
{
	float t0 = (b.z * a.y);
	float t1 = (b.y * a.z);
	r.x = t0 - t1;

	float t2 = (b.x * a.z);
	float t3 = (a.x * b.z);
	r.y = t2 - t3;

	float t4 = (a.x * b.y);
	float t5 = (b.x * a.y);
	r.z = t4 - t5;
} */

// MATCHING
void Cross(Vector3& r, const Vector3& a, const Vector3& b)
{
	float fy = b.x * a.z - b.z * a.x;
	float fz = b.y * a.x - b.x * a.y;
	float fx = b.z * a.y - b.y * a.z;

	r.x = fx;
	r.y = fy;
	r.z = fz;
}

Vector3& Cross(const Vector3& a, const Vector3& b)
{
	Vector3 r;
	r.x = (b.z * a.y) - (b.y * a.z);
	r.y = (b.x * a.z) - (a.x * b.z);
	r.z = (a.x * b.y) - (b.x * a.y);
	return r;
}

// MATCHING
void Scale(const Vector3& a, float s, Vector3& r)
{
	r.x = a.x * s;
	r.y = a.y * s;
	r.z = a.z * s;
}

void ScaleAdd(const Vector3& a, float s, const Vector3& add, Vector3& r)
{
	Scale(a, s, r);
	r += add;
}

// MATCHING
float LengthSquared(const Vector3& a)
{
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

void Unit(const Vector3& a, Vector3& r)
{
	float len = LengthSquared(a);
	float rlen = 0.f;
	if (len > FLT_EPSILON)
		rlen = Rsqrt(len);
	Scale(a, rlen, r);
}

// MATCHING
void UnitCross(const Vector3& a, const Vector3& b, Vector3& r)
{
	Vector3 c;
	Cross(c, a, b);
	/* c.x = (b.z * a.y) - b.y * a.z;
	c.y = (b.x * a.z) - b.z * a.x;
	c.z = (b.y * a.x) - b.x * a.y; */
	r = c;
	Unit(r, r);
}

} // namespace UMath
