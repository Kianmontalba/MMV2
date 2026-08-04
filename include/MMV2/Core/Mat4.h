#pragma once
#ifndef MMV2_MAT4_H
#define MMV2_MAT4_H

#include "Config.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Quat.h"
#include <cstring>

MMV2_NAMESPACE_BEGIN

struct MMV2_ALIGN(64) Mat4 {
    union {
        float32 m[4][4];
        float32 flat[16];
        Vec4 cols[4];
    };

    MMV2_FORCE_INLINE Mat4() noexcept { *this = Identity(); }
    MMV2_FORCE_INLINE Mat4(const Vec4& c0, const Vec4& c1, const Vec4& c2, const Vec4& c3) noexcept {
        cols[0] = c0; cols[1] = c1; cols[2] = c2; cols[3] = c3;
    }
    MMV2_FORCE_INLINE explicit Mat4(const float32* data) noexcept { std::memcpy(flat, data, sizeof(flat)); }

    // Element access
    MMV2_FORCE_INLINE float32& operator()(size_t row, size_t col) noexcept { return m[col][row]; }
    MMV2_FORCE_INLINE const float32& operator()(size_t row, size_t col) const noexcept { return m[col][row]; }
    MMV2_FORCE_INLINE float32& operator[](size_t i) noexcept { return flat[i]; }
    MMV2_FORCE_INLINE const float32& operator[](size_t i) const noexcept { return flat[i]; }

    // Matrix multiplication
    MMV2_FORCE_INLINE Mat4 operator*(const Mat4& o) const noexcept {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[j][i] = m[0][i] * o.m[j][0] + m[1][i] * o.m[j][1] + m[2][i] * o.m[j][2] + m[3][i] * o.m[j][3];
            }
        }
        return result;
    }

    MMV2_FORCE_INLINE Mat4& operator*=(const Mat4& o) noexcept { *this = *this * o; return *this; }

    // Matrix-vector multiplication
    MMV2_FORCE_INLINE Vec4 operator*(const Vec4& v) const noexcept {
        return Vec4(
            cols[0].x * v.x + cols[1].x * v.y + cols[2].x * v.z + cols[3].x * v.w,
            cols[0].y * v.x + cols[1].y * v.y + cols[2].y * v.z + cols[3].y * v.w,
            cols[0].z * v.x + cols[1].z * v.y + cols[2].z * v.z + cols[3].z * v.w,
            cols[0].w * v.x + cols[1].w * v.y + cols[2].w * v.z + cols[3].w * v.w
        );
    }

    MMV2_FORCE_INLINE Vec3 TransformPoint(const Vec3& p) const noexcept {
        Vec4 v = *this * Vec4(p, 1.0f);
        return v.XYZ() / v.w;
    }

    MMV2_FORCE_INLINE Vec3 TransformVector(const Vec3& v) const noexcept {
        return Vec3(
            cols[0].x * v.x + cols[1].x * v.y + cols[2].x * v.z,
            cols[0].y * v.x + cols[1].y * v.y + cols[2].y * v.z,
            cols[0].z * v.x + cols[1].z * v.y + cols[2].z * v.z
        );
    }

    MMV2_FORCE_INLINE Vec3 TransformDirection(const Vec3& v) const noexcept {
        return TransformVector(v).Normalized();
    }

    // Transpose
    MMV2_FORCE_INLINE Mat4 Transposed() const noexcept {
        Mat4 result;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                result.m[i][j] = m[j][i];
        return result;
    }

    // Inverse
    Mat4 Inverted() const noexcept;
    bool Invert() noexcept;

    // Determinant
    float32 Determinant() const noexcept;

    // Translation
    MMV2_FORCE_INLINE Vec3 GetTranslation() const noexcept { return Vec3(cols[3].x, cols[3].y, cols[3].z); }
    MMV2_FORCE_INLINE void SetTranslation(const Vec3& t) noexcept { cols[3].x = t.x; cols[3].y = t.y; cols[3].z = t.z; }

    // Scale
    Vec3 GetScale() const noexcept;
    void SetScale(const Vec3& s) noexcept;

    // Rotation (as quaternion)
    Quat GetRotation() const noexcept;
    void SetRotation(const Quat& q) noexcept;

    // TRS decomposition
    void Decompose(Vec3& translation, Quat& rotation, Vec3& scale) const noexcept;
    static Mat4 Compose(const Vec3& translation, const Quat& rotation, const Vec3& scale) noexcept;

    // Factory methods
    MMV2_FORCE_INLINE static Mat4 Identity() noexcept {
        Mat4 m;
        std::memset(m.flat, 0, sizeof(m.flat));
        m.m[0][0] = m.m[1][1] = m.m[2][2] = m.m[3][3] = 1.0f;
        return m;
    }

    MMV2_FORCE_INLINE static Mat4 Translation(const Vec3& t) noexcept {
        Mat4 m = Identity();
        m.SetTranslation(t);
        return m;
    }

    MMV2_FORCE_INLINE static Mat4 Scale(const Vec3& s) noexcept {
        Mat4 m = Identity();
        m.m[0][0] = s.x; m.m[1][1] = s.y; m.m[2][2] = s.z;
        return m;
    }

    MMV2_FORCE_INLINE static Mat4 Rotation(const Quat& q) noexcept { return q.ToRotationMatrix(); }

    MMV2_FORCE_INLINE static Mat4 TRS(const Vec3& translation, const Quat& rotation, const Vec3& scale) noexcept {
        return Compose(translation, rotation, scale);
    }

    // Perspective projection
    static Mat4 Perspective(float32 fovY, float32 aspect, float32 nearPlane, float32 farPlane) noexcept;
    static Mat4 PerspectiveInfinite(float32 fovY, float32 aspect, float32 nearPlane) noexcept;

    // Orthographic projection
    static Mat4 Orthographic(float32 left, float32 right, float32 bottom, float32 top, float32 nearPlane, float32 farPlane) noexcept;

    // LookAt
    static Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up) noexcept;

    // View matrix
    static Mat4 View(const Vec3& eye, const Vec3& forward, const Vec3& up, const Vec3& right) noexcept;

    // Lerp (element-wise)
    MMV2_FORCE_INLINE static Mat4 Lerp(const Mat4& a, const Mat4& b, float32 t) noexcept {
        Mat4 result;
        for (int i = 0; i < 16; ++i) result.flat[i] = a.flat[i] + (b.flat[i] - a.flat[i]) * t;
        return result;
    }
};

MMV2_NAMESPACE_END

#endif
