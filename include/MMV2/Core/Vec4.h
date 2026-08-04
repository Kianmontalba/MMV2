#pragma once
#ifndef MMV2_VEC4_H
#define MMV2_VEC4_H

#include "Config.h"
#include <cmath>
#include <algorithm>

MMV2_NAMESPACE_BEGIN

struct Vec3;

struct MMV2_ALIGN(16) Vec4 {
    float32 x, y, z, w;

    MMV2_FORCE_INLINE constexpr Vec4() noexcept : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
    MMV2_FORCE_INLINE constexpr Vec4(float32 v) noexcept : x(v), y(v), z(v), w(v) {}
    MMV2_FORCE_INLINE constexpr Vec4(float32 x_, float32 y_, float32 z_, float32 w_) noexcept : x(x_), y(y_), z(z_), w(w_) {}
    MMV2_FORCE_INLINE constexpr Vec4(const Vec3& v3, float32 w_ = 1.0f) noexcept;

    MMV2_FORCE_INLINE float32& operator[](size_t i) noexcept { return (&x)[i]; }
    MMV2_FORCE_INLINE const float32& operator[](size_t i) const noexcept { return (&x)[i]; }

    MMV2_FORCE_INLINE Vec4 operator+(const Vec4& o) const noexcept { return Vec4(x + o.x, y + o.y, z + o.z, w + o.w); }
    MMV2_FORCE_INLINE Vec4 operator-(const Vec4& o) const noexcept { return Vec4(x - o.x, y - o.y, z - o.z, w - o.w); }
    MMV2_FORCE_INLINE Vec4 operator*(const Vec4& o) const noexcept { return Vec4(x * o.x, y * o.y, z * o.z, w * o.w); }
    MMV2_FORCE_INLINE Vec4 operator/(const Vec4& o) const noexcept { return Vec4(x / o.x, y / o.y, z / o.z, w / o.w); }
    MMV2_FORCE_INLINE Vec4 operator*(float32 s) const noexcept { return Vec4(x * s, y * s, z * s, w * s); }
    MMV2_FORCE_INLINE Vec4 operator/(float32 s) const noexcept { return Vec4(x / s, y / s, z / s, w / s); }

    MMV2_FORCE_INLINE Vec4& operator+=(const Vec4& o) noexcept { x += o.x; y += o.y; z += o.z; w += o.w; return *this; }
    MMV2_FORCE_INLINE Vec4& operator-=(const Vec4& o) noexcept { x -= o.x; y -= o.y; z -= o.z; w -= o.w; return *this; }
    MMV2_FORCE_INLINE Vec4& operator*=(float32 s) noexcept { x *= s; y *= s; z *= s; w *= s; return *this; }
    MMV2_FORCE_INLINE Vec4& operator/=(float32 s) noexcept { x /= s; y /= s; z /= s; w /= s; return *this; }

    MMV2_FORCE_INLINE Vec4 operator-() const noexcept { return Vec4(-x, -y, -z, -w); }

    MMV2_FORCE_INLINE bool operator==(const Vec4& o) const noexcept {
        return std::abs(x - o.x) < MMV2_EPSILON && std::abs(y - o.y) < MMV2_EPSILON
            && std::abs(z - o.z) < MMV2_EPSILON && std::abs(w - o.w) < MMV2_EPSILON;
    }

    MMV2_FORCE_INLINE float32 Dot(const Vec4& o) const noexcept { return x * o.x + y * o.y + z * o.z + w * o.w; }
    MMV2_FORCE_INLINE float32 LengthSq() const noexcept { return x * x + y * y + z * z + w * w; }
    MMV2_FORCE_INLINE float32 Length() const noexcept { return std::sqrt(LengthSq()); }

    MMV2_FORCE_INLINE Vec4 Normalized() const noexcept {
        float32 len = Length();
        if (len > MMV2_EPSILON) return *this / len;
        return Vec4(0.0f);
    }

    MMV2_FORCE_INLINE Vec3 XYZ() const noexcept;
    MMV2_FORCE_INLINE Vec3 ToVec3() const noexcept { return XYZ(); }

    MMV2_FORCE_INLINE static Vec4 Lerp(const Vec4& a, const Vec4& b, float32 t) noexcept {
        return a + (b - a) * t;
    }

    MMV2_FORCE_INLINE static constexpr Vec4 Zero() noexcept { return Vec4(0.0f); }
    MMV2_FORCE_INLINE static constexpr Vec4 One() noexcept { return Vec4(1.0f); }
};

MMV2_FORCE_INLINE Vec4 operator*(float32 s, const Vec4& v) noexcept { return v * s; }

MMV2_NAMESPACE_END

#endif
