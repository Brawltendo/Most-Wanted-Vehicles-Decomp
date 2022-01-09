// Basic vector structs and helper functions
#pragma once

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

    Vector2(const float in)
    {
        x = in;
        y = in;
    }

    Vector2(const float inX, const float inY)
    {
        x = inX;
        y = inY;
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
        x = v.x;
        y = v.y;
        z = v.z;
        w = inW;
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

float Dot(const Vector3& a, const Vector3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

void Scale(const Vector3& a, float s, Vector3& r)
{
	r.x = a.x * s;
	r.y = a.y * s;
	r.z = a.z * s;
}

} // namespace UMath

struct UVector3 : UMath::Vector3
{
	
};
