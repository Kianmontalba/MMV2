#pragma once
#ifndef MMV2_TRANSFORM_H
#define MMV2_TRANSFORM_H

#include "Config.h"
#include "Vec3.h"
#include "Quat.h"
#include "Mat4.h"

MMV2_NAMESPACE_BEGIN

struct MMV2_ALIGN(32) Transform {
    Vec3 position;
    Quat rotation;
    Vec3 scale;

    MMV2_FORCE_INLINE Transform() noexcept : position(Vec3::Zero()), rotation(Quat::Identity()), scale(Vec3::One()) {}
    MMV2_FORCE_INLINE Transform(const Vec3& pos, const Quat& rot, const Vec3& scl) noexcept
        : position(pos), rotation(rot), scale(scl) {}
    MMV2_FORCE_INLINE explicit Transform(const Mat4& matrix) noexcept { FromMatrix(matrix); }

    // To/From matrix
    MMV2_FORCE_INLINE Mat4 ToMatrix() const noexcept { return Mat4::TRS(position, rotation, scale); }
    MMV2_FORCE_INLINE void FromMatrix(const Mat4& matrix) noexcept { matrix.Decompose(position, rotation, scale); }

    // Transform operations
    MMV2_FORCE_INLINE Vec3 TransformPoint(const Vec3& p) const noexcept {
        return rotation.Rotate(p * scale) + position;
    }
    MMV2_FORCE_INLINE Vec3 TransformVector(const Vec3& v) const noexcept {
        return rotation.Rotate(v * scale);
    }
    MMV2_FORCE_INLINE Vec3 TransformDirection(const Vec3& d) const noexcept {
        return rotation.Rotate(d);
    }

    MMV2_FORCE_INLINE Vec3 InverseTransformPoint(const Vec3& p) const noexcept {
        return rotation.Inverse().Rotate(p - position) / scale;
    }
    MMV2_FORCE_INLINE Vec3 InverseTransformVector(const Vec3& v) const noexcept {
        return rotation.Inverse().Rotate(v) / scale;
    }
    MMV2_FORCE_INLINE Vec3 InverseTransformDirection(const Vec3& d) const noexcept {
        return rotation.Inverse().Rotate(d);
    }

    // Forward/Right/Up vectors
    MMV2_FORCE_INLINE Vec3 Forward() const noexcept { return TransformDirection(Vec3::Forward()); }
    MMV2_FORCE_INLINE Vec3 Right() const noexcept { return TransformDirection(Vec3::Right()); }
    MMV2_FORCE_INLINE Vec3 Up() const noexcept { return TransformDirection(Vec3::Up()); }

    // Combine transforms
    MMV2_FORCE_INLINE Transform operator*(const Transform& o) const noexcept {
        return Transform(
            TransformPoint(o.position),
            rotation * o.rotation,
            scale * o.scale
        );
    }

    // Inverse
    MMV2_FORCE_INLINE Transform Inverted() const noexcept {
        Quat invRot = rotation.Inverse();
        Vec3 invScale = Vec3(1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z);
        return Transform(
            invRot.Rotate(-position) * invScale,
            invRot,
            invScale
        );
    }

    // Lerp
    MMV2_FORCE_INLINE static Transform Lerp(const Transform& a, const Transform& b, float32 t) noexcept {
        return Transform(
            Vec3::Lerp(a.position, b.position, t),
            Quat::SlerpShortestPath(a.rotation, b.rotation, t),
            Vec3::Lerp(a.scale, b.scale, t)
        );
    }

    // Identity
    MMV2_FORCE_INLINE static Transform Identity() noexcept { return Transform(); }
};

MMV2_NAMESPACE_END

#endif
