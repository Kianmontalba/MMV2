#pragma once
#ifndef MMV2_VEC2_H
#define MMV2_VEC2_H

#include "Config.h"
#include <cmath>
#include <algorithm>

MMV2_NAMESPACE_BEGIN

struct MMV2_ALIGN(8) Vec2 {
    float32 x, y;

    MMV2_FORCE_INLINE constexpr Vec2() noexcept : x(0.0f), y(0.0f) {}
    MMV2_FORCE_INLINE constexpr Vec2(float32 v) noexcept : x(v), y(v) {}
    MMV2_FORCE_INLINE constexpr Vec2(float32 x_, float32 y_) noexcept : x(x_), y(y_) {}

    // Element access
    MMV2_FORCE_INLINE float32& operator[](size_t i) noexcept { return (&x)[i]; }
    MMV2_FORCE_INLINE const float32& operator[](size_t i) const noexcept { return (&x)[i]; }

    // Arithmetic operators
    MMV2_FORCE_INLINE Vec2 operator+(const Vec2& o) const noexcept { return Vec2(x + o.x, y + o.y); }
    MMV2_FORCE_INLINE Vec2 operator-(const Vec2& o) const noexcept { return Vec2(x - o.x, y - o.y); }
    MMV2_FORCE_INLINE Vec2 operator*(const Vec2& o) const noexcept { return Vec2(x * o.x, y * o.y); }
    MMV2_FORCE_INLINE Vec2 operator/(const Vec2& o) const noexcept { return Vec2(x / o.x, y / o.y); }
    MMV2_FORCE_INLINE Vec2 operator*(float32 s) const noexcept { return Vec2(x * s, y * s); }
    MMV2_FORCE_INLINE Vec2 operator/(float32 s) const noexcept { return Vec2(x / s, y / s); }

    MMV2_FORCE_INLINE Vec2& operator+=(const Vec2& o) noexcept { x += o.x; y += o.y; return *this; }
    MMV2_FORCE_INLINE Vec2& operator-=(const Vec2& o) noexcept { x -= o.x; y -= o.y; return *this; }
    MMV2_FORCE_INLINE Vec2& operator*=(const Vec2& o) noexcept { x *= o.x; y *= o.y; return *this; }
    MMV2_FORCE_INLINE Vec2& operator/=(const Vec2& o) noexcept { x /= o.x; y /= o.y; return *this; }
    MMV2_FORCE_INLINE Vec2& operator*=(float32 s) noexcept { x *= s; y *= s; return *this; }
    MMV2_FORCE_INLINE Vec2& operator/=(float32 s) noexcept { x /= s; y /= s; return *this; }

    MMV2_FORCE_INLINE Vec2 operator-() const noexcept { return Vec2(-x, -y); }

    // Comparison
    MMV2_FORCE_INLINE bool operator==(const Vec2& o) const noexcept { return x == o.x && y == o.y; }
    MMV2_FORCE_INLINE bool operator!=(const Vec2& o) const noexcept { return !(*this == o); }

    // Dot product
    MMV2_FORCE_INLINE float32 Dot(const Vec2& o) const noexcept { return x * o.x + y * o.y; }

    // Length
    MMV2_FORCE_INLINE float32 LengthSq() const noexcept { return x * x + y * y; }
    MMV2_FORCE_INLINE float32 Length() const noexcept { return std::sqrt(LengthSq()); }

    // Normalization
    MMV2_FORCE_INLINE Vec2 Normalized() const noexcept {
        float32 len = Length();
        if (len > MMV2_EPSILON) return *this / len;
        return Vec2(0.0f);
    }
    MMV2_FORCE_INLINE void Normalize() noexcept {
        float32 len = Length();
        if (len > MMV2_EPSILON) { *this /= len; }
    }

    // Distance
    MMV2_FORCE_INLINE float32 DistanceTo(const Vec2& o) const noexcept { return (*this - o).Length(); }
    MMV2_FORCE_INLINE float32 DistanceToSq(const Vec2& o) const noexcept { return (*this - o).LengthSq(); }

    // Lerp
    MMV2_FORCE_INLINE static Vec2 Lerp(const Vec2& a, const Vec2& b, float32 t) noexcept {
        return a + (b - a) * t;
    }

    // Min/Max
    MMV2_FORCE_INLINE static Vec2 Min(const Vec2& a, const Vec2& b) noexcept {
        return Vec2(std::min(a.x, b.x), std::min(a.y, b.y));
    }
    MMV2_FORCE_INLINE static Vec2 Max(const Vec2& a, const Vec2& b) noexcept {
        return Vec2(std::max(a.x, b.x), std::max(a.y, b.y));
    }

    // Clamp
    MMV2_FORCE_INLINE static Vec2 Clamp(const Vec2& v, const Vec2& min, const Vec2& max) noexcept {
        return Vec2(std::clamp(v.x, min.x, max.x), std::clamp(v.y, min.y, max.y));
    }

    // Perpendicular
    MMV2_FORCE_INLINE Vec2 Perpendicular() const noexcept { return Vec2(-y, x); }

    // Cross product (scalar result for 2D)
    MMV2_FORCE_INLINE float32 Cross(const Vec2& o) const noexcept { return x * o.y - y * o.x; }

    // Angle
    MMV2_FORCE_INLINE float32 Angle() const noexcept { return std::atan2(y, x); }
    MMV2_FORCE_INLINE static float32 AngleBetween(const Vec2& a, const Vec2& b) noexcept {
        float32 dot = a.Dot(b);
        float32 det = a.Cross(b);
        return std::atan2(det, dot);
    }

    // Zero/One vectors
    MMV2_FORCE_INLINE static constexpr Vec2 Zero() noexcept { return Vec2(0.0f); }
    MMV2_FORCE_INLINE static constexpr Vec2 One() noexcept { return Vec2(1.0f); }
    MMV2_FORCE_INLINE static constexpr Vec2 Right() noexcept { return Vec2(1.0f, 0.0f); }
    MMV2_FORCE_INLINE static constexpr Vec2 Up() noexcept { return Vec2(0.0f, 1.0f); }
};

// Scalar multiplication (reverse)
MMV2_FORCE_INLINE Vec2 operator*(float32 s, const Vec2& v) noexcept { return v * s; }

MMV2_NAMESPACE_END

#endif
