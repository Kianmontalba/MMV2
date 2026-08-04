#pragma once
#ifndef MMV2_QUAT_H
#define MMV2_QUAT_H

#include "Config.h"
#include "Vec3.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

struct Mat4;

struct MMV2_ALIGN(16) Quat {
    float32 x, y, z, w;

    MMV2_FORCE_INLINE constexpr Quat() noexcept : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
    MMV2_FORCE_INLINE constexpr Quat(float32 x_, float32 y_, float32 z_, float32 w_) noexcept : x(x_), y(y_), z(z_), w(w_) {}
    MMV2_FORCE_INLINE Quat(const Vec3& axis, float32 angle) noexcept {
        float32 halfAngle = angle * 0.5f;
        float32 s = std::sin(halfAngle);
        x = axis.x * s;
        y = axis.y * s;
        z = axis.z * s;
        w = std::cos(halfAngle);
    }
    MMV2_FORCE_INLINE Quat(float32 pitch, float32 yaw, float32 roll) noexcept;

    // Element access
    MMV2_FORCE_INLINE float32& operator[](size_t i) noexcept { return (&x)[i]; }
    MMV2_FORCE_INLINE const float32& operator[](size_t i) const noexcept { return (&x)[i]; }

    // Arithmetic
    MMV2_FORCE_INLINE Quat operator+(const Quat& o) const noexcept { return Quat(x + o.x, y + o.y, z + o.z, w + o.w); }
    MMV2_FORCE_INLINE Quat operator-(const Quat& o) const noexcept { return Quat(x - o.x, y - o.y, z - o.z, w - o.w); }
    MMV2_FORCE_INLINE Quat operator*(float32 s) const noexcept { return Quat(x * s, y * s, z * s, w * s); }
    MMV2_FORCE_INLINE Quat operator/(float32 s) const noexcept { return Quat(x / s, y / s, z / s, w / s); }

    // Quaternion multiplication
    MMV2_FORCE_INLINE Quat operator*(const Quat& o) const noexcept {
        return Quat(
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w,
            w * o.w - x * o.x - y * o.y - z * o.z
        );
    }

    MMV2_FORCE_INLINE Quat& operator*=(const Quat& o) noexcept { *this = *this * o; return *this; }

    // Conjugate
    MMV2_FORCE_INLINE Quat Conjugate() const noexcept { return Quat(-x, -y, -z, w); }

    // Inverse
    MMV2_FORCE_INLINE Quat Inverse() const noexcept {
        float32 lenSq = x * x + y * y + z * z + w * w;
        if (lenSq < MMV2_EPSILON_SQ) return Quat::Identity();
        return Conjugate() / lenSq;
    }

    // Length
    MMV2_FORCE_INLINE float32 LengthSq() const noexcept { return x * x + y * y + z * z + w * w; }
    MMV2_FORCE_INLINE float32 Length() const noexcept { return std::sqrt(LengthSq()); }

    // Normalization
    MMV2_FORCE_INLINE Quat Normalized() const noexcept {
        float32 len = Length();
        if (len > MMV2_EPSILON) return *this / len;
        return Quat::Identity();
    }
    MMV2_FORCE_INLINE void Normalize() noexcept {
        float32 len = Length();
        if (len > MMV2_EPSILON) { *this /= len; }
    }

    // Rotate vector
    MMV2_FORCE_INLINE Vec3 Rotate(const Vec3& v) const noexcept {
        Quat qv(v.x, v.y, v.z, 0.0f);
        Quat result = *this * qv * Conjugate();
        return Vec3(result.x, result.y, result.z);
    }

    // To/from Euler angles
    Vec3 ToEulerAngles() const noexcept;
    static Quat FromEulerAngles(float32 pitch, float32 yaw, float32 roll) noexcept;
    static Quat FromEulerAngles(const Vec3& euler) noexcept;

    // To/from axis-angle
    void ToAxisAngle(Vec3& axis, float32& angle) const noexcept;
    static Quat FromAxisAngle(const Vec3& axis, float32 angle) noexcept;

    // To/from rotation matrix
    static Quat FromRotationMatrix(const Mat4& m) noexcept;
    Mat4 ToRotationMatrix() const noexcept;

    // To/from direction vectors
    static Quat FromToRotation(const Vec3& from, const Vec3& to) noexcept;
    static Quat LookRotation(const Vec3& forward, const Vec3& up) noexcept;

    // Lerp / Slerp
    MMV2_FORCE_INLINE static Quat Lerp(const Quat& a, const Quat& b, float32 t) noexcept {
        return (a * (1.0f - t) + b * t).Normalized();
    }
    static Quat Slerp(const Quat& a, const Quat& b, float32 t) noexcept;
    static Quat SlerpShortestPath(const Quat& a, const Quat& b, float32 t) noexcept;

    // Nlerp (normalized lerp) - faster than slerp
    MMV2_FORCE_INLINE static Quat Nlerp(const Quat& a, const Quat& b, float32 t) noexcept {
        float32 dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        if (dot < 0.0f) {
            return (a * (1.0f - t) - b * t).Normalized();
        }
        return (a * (1.0f - t) + b * t).Normalized();
    }

    // Dot product
    MMV2_FORCE_INLINE float32 Dot(const Quat& o) const noexcept { return x * o.x + y * o.y + z * o.z + w * o.w; }

    // Angular distance
    MMV2_FORCE_INLINE float32 AngularDistance(const Quat& o) const noexcept {
        float32 dot = std::abs(Dot(o));
        return 2.0f * std::acos(std::min(dot, 1.0f));
    }

    // Is normalized
    MMV2_FORCE_INLINE bool IsNormalized() const noexcept { return std::abs(LengthSq() - 1.0f) < MMV2_EPSILON; }

    // Identity
    MMV2_FORCE_INLINE static constexpr Quat Identity() noexcept { return Quat(0.0f, 0.0f, 0.0f, 1.0f); }
};

MMV2_FORCE_INLINE Quat operator*(float32 s, const Quat& q) noexcept { return q * s; }

MMV2_NAMESPACE_END

#endif
