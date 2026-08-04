#pragma once
#ifndef MMV2_VEC3_H
#define MMV2_VEC3_H

#include "Config.h"
#include <cmath>
#include <algorithm>

MMV2_NAMESPACE_BEGIN

struct Vec2;
struct Vec4;
struct Quat;

struct MMV2_ALIGN(16) Vec3 {
    float32 x, y, z;

    MMV2_FORCE_INLINE constexpr Vec3() noexcept : x(0.0f), y(0.0f), z(0.0f) {}
    MMV2_FORCE_INLINE constexpr Vec3(float32 v) noexcept : x(v), y(v), z(v) {}
    MMV2_FORCE_INLINE constexpr Vec3(float32 x_, float32 y_, float32 z_) noexcept : x(x_), y(y_), z(z_) {}
    MMV2_FORCE_INLINE constexpr Vec3(const Vec2& v2, float32 z_ = 0.0f) noexcept;

    // Element access
    MMV2_FORCE_INLINE float32& operator[](size_t i) noexcept { return (&x)[i]; }
    MMV2_FORCE_INLINE const float32& operator[](size_t i) const noexcept { return (&x)[i]; }

    // Arithmetic
    MMV2_FORCE_INLINE Vec3 operator+(const Vec3& o) const noexcept { return Vec3(x + o.x, y + o.y, z + o.z); }
    MMV2_FORCE_INLINE Vec3 operator-(const Vec3& o) const noexcept { return Vec3(x - o.x, y - o.y, z - o.z); }
    MMV2_FORCE_INLINE Vec3 operator*(const Vec3& o) const noexcept { return Vec3(x * o.x, y * o.y, z * o.z); }
    MMV2_FORCE_INLINE Vec3 operator/(const Vec3& o) const noexcept { return Vec3(x / o.x, y / o.y, z / o.z); }
    MMV2_FORCE_INLINE Vec3 operator*(float32 s) const noexcept { return Vec3(x * s, y * s, z * s); }
    MMV2_FORCE_INLINE Vec3 operator/(float32 s) const noexcept { return Vec3(x / s, y / s, z / s); }

    MMV2_FORCE_INLINE Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    MMV2_FORCE_INLINE Vec3& operator-=(const Vec3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    MMV2_FORCE_INLINE Vec3& operator*=(const Vec3& o) noexcept { x *= o.x; y *= o.y; z *= o.z; return *this; }
    MMV2_FORCE_INLINE Vec3& operator/=(const Vec3& o) noexcept { x /= o.x; y /= o.y; z /= o.z; return *this; }
    MMV2_FORCE_INLINE Vec3& operator*=(float32 s) noexcept { x *= s; y *= s; z *= s; return *this; }
    MMV2_FORCE_INLINE Vec3& operator/=(float32 s) noexcept { x /= s; y /= s; z /= s; return *this; }

    MMV2_FORCE_INLINE Vec3 operator-() const noexcept { return Vec3(-x, -y, -z); }

    // Comparison
    MMV2_FORCE_INLINE bool operator==(const Vec3& o) const noexcept {
        return std::abs(x - o.x) < MMV2_EPSILON && std::abs(y - o.y) < MMV2_EPSILON && std::abs(z - o.z) < MMV2_EPSILON;
    }
    MMV2_FORCE_INLINE bool operator!=(const Vec3& o) const noexcept { return !(*this == o); }

    // Dot product
    MMV2_FORCE_INLINE float32 Dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }

    // Cross product
    MMV2_FORCE_INLINE Vec3 Cross(const Vec3& o) const noexcept {
        return Vec3(
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        );
    }

    // Length
    MMV2_FORCE_INLINE float32 LengthSq() const noexcept { return x * x + y * y + z * z; }
    MMV2_FORCE_INLINE float32 Length() const noexcept { return std::sqrt(LengthSq()); }

    // Normalization
    MMV2_FORCE_INLINE Vec3 Normalized() const noexcept {
        float32 len = Length();
        if (len > MMV2_EPSILON) return *this / len;
        return Vec3(0.0f);
    }
    MMV2_FORCE_INLINE void Normalize() noexcept {
        float32 len = Length();
        if (len > MMV2_EPSILON) { *this /= len; }
    }

    // Distance
    MMV2_FORCE_INLINE float32 DistanceTo(const Vec3& o) const noexcept { return (*this - o).Length(); }
    MMV2_FORCE_INLINE float32 DistanceToSq(const Vec3& o) const noexcept { return (*this - o).LengthSq(); }

    // Lerp
    MMV2_FORCE_INLINE static Vec3 Lerp(const Vec3& a, const Vec3& b, float32 t) noexcept {
        return a + (b - a) * t;
    }

    // Slerp (for direction vectors)
    static Vec3 Slerp(const Vec3& a, const Vec3& b, float32 t) noexcept;

    // Min/Max
    MMV2_FORCE_INLINE static Vec3 Min(const Vec3& a, const Vec3& b) noexcept {
        return Vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
    }
    MMV2_FORCE_INLINE static Vec3 Max(const Vec3& a, const Vec3& b) noexcept {
        return Vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
    }

    // Clamp
    MMV2_FORCE_INLINE static Vec3 Clamp(const Vec3& v, const Vec3& min, const Vec3& max) noexcept {
        return Vec3(std::clamp(v.x, min.x, max.x), std::clamp(v.y, min.y, max.y), std::clamp(v.z, min.z, max.z));
    }

    // Project/Reject
    MMV2_FORCE_INLINE Vec3 ProjectOnto(const Vec3& onto) const noexcept {
        float32 lenSq = onto.LengthSq();
        if (lenSq < MMV2_EPSILON_SQ) return Vec3::Zero();
        return onto * (Dot(onto) / lenSq);
    }
    MMV2_FORCE_INLINE Vec3 RejectFrom(const Vec3& from) const noexcept { return *this - ProjectOnto(from); }

    // Reflect
    MMV2_FORCE_INLINE Vec3 Reflect(const Vec3& normal) const noexcept {
        return *this - normal * (2.0f * Dot(normal));
    }

    // Angle between vectors
    MMV2_FORCE_INLINE float32 AngleTo(const Vec3& o) const noexcept {
        float32 dot = Dot(o);
        float32 lenProduct = Length() * o.Length();
        if (lenProduct < MMV2_EPSILON) return 0.0f;
        return std::acos(std::clamp(dot / lenProduct, -1.0f, 1.0f));
    }

    // Component-wise abs
    MMV2_FORCE_INLINE Vec3 Abs() const noexcept { return Vec3(std::abs(x), std::abs(y), std::abs(z)); }

    // Is zero/near zero
    MMV2_FORCE_INLINE bool IsNearZero() const noexcept { return LengthSq() < MMV2_EPSILON_SQ; }
    MMV2_FORCE_INLINE bool IsNormalized() const noexcept { return std::abs(LengthSq() - 1.0f) < MMV2_EPSILON; }

    // Swizzle
    MMV2_FORCE_INLINE Vec2 XY() const noexcept;
    MMV2_FORCE_INLINE Vec2 XZ() const noexcept;
    MMV2_FORCE_INLINE Vec2 YZ() const noexcept;

    // Constants
    MMV2_FORCE_INLINE static constexpr Vec3 Zero() noexcept { return Vec3(0.0f); }
    MMV2_FORCE_INLINE static constexpr Vec3 One() noexcept { return Vec3(1.0f); }
    MMV2_FORCE_INLINE static constexpr Vec3 Right() noexcept { return Vec3(1.0f, 0.0f, 0.0f); }
    MMV2_FORCE_INLINE static constexpr Vec3 Up() noexcept { return Vec3(0.0f, 1.0f, 0.0f); }
    MMV2_FORCE_INLINE static constexpr Vec3 Forward() noexcept { return Vec3(0.0f, 0.0f, 1.0f); }
};

MMV2_FORCE_INLINE Vec3 operator*(float32 s, const Vec3& v) noexcept { return v * s; }

MMV2_NAMESPACE_END

#endif
