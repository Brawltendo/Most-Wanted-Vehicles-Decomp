// Basic vector structs and helper functions
#pragma once

namespace UMath
{

struct Vector2
{
    union
    {
        struct
        {
            float x, y;
        };
        float arr[2];
    };

    Vector2()
    {
        x = 0.f;
        y = 0.f;
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
    union
    {
        struct
        {
            float x, y, z;
        };
        float arr[3];
    };

    Vector3()
    {
        /* x = 0.f;
        y = 0.f;
        z = 0.f; */
    }

    Vector3(const float in)
    {
        x = in;
        y = in;
        z = in;
    }

    Vector3(const float inX, const float inY, const float inZ)
    {
        x = inX;
        y = inY;
        z = inZ;
    }

    Vector3 operator+(const Vector3& b)
    {
        Vector3 v;
        v.x = this->x + b.x;
        v.y = this->y + b.y;
        v.z = this->z + b.z;
        return v;
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

};

struct Vector4
{
    union
    {
        struct
        {
            float x, y, z, w;
        };
        float arr[4];
    };

    Vector4()
    {
        x = 0.f;
        y = 0.f;
        z = 0.f;
        w = 0.f;
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

};

}