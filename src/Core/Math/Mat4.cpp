#include "Mat4.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

float32 Mat4::Determinant() const noexcept {
    float32 det = 0.0f;
    det += m[0][0] * (m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) + m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]));
    det -= m[0][1] * (m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]));
    det += m[0][2] * (m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) - m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]));
    det -= m[0][3] * (m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) - m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) + m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]));
    return det;
}

Mat4 Mat4::Inverted() const noexcept {
    Mat4 inv;
    float32 det = Determinant();
    if (std::abs(det) < MMV2_EPSILON) return Identity();
    float32 invDet = 1.0f / det;

    inv.m[0][0] =  (m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) + m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])) * invDet;
    inv.m[0][1] = -(m[0][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - m[0][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) + m[0][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])) * invDet;
    inv.m[0][2] =  (m[0][1] * (m[1][2] * m[3][3] - m[1][3] * m[3][2]) - m[0][2] * (m[1][1] * m[3][3] - m[1][3] * m[3][1]) + m[0][3] * (m[1][1] * m[3][2] - m[1][2] * m[3][1])) * invDet;
    inv.m[0][3] = -(m[0][1] * (m[1][2] * m[2][3] - m[1][3] * m[2][2]) - m[0][2] * (m[1][1] * m[2][3] - m[1][3] * m[2][1]) + m[0][3] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])) * invDet;

    inv.m[1][0] = -(m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])) * invDet;
    inv.m[1][1] =  (m[0][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - m[0][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + m[0][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])) * invDet;
    inv.m[1][2] = -(m[0][0] * (m[1][2] * m[3][3] - m[1][3] * m[3][2]) - m[0][2] * (m[1][0] * m[3][3] - m[1][3] * m[3][0]) + m[0][3] * (m[1][0] * m[3][2] - m[1][2] * m[3][0])) * invDet;
    inv.m[1][3] =  (m[0][0] * (m[1][2] * m[2][3] - m[1][3] * m[2][2]) - m[0][2] * (m[1][0] * m[2][3] - m[1][3] * m[2][0]) + m[0][3] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])) * invDet;

    inv.m[2][0] =  (m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) - m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * invDet;
    inv.m[2][1] = -(m[0][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) - m[0][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + m[0][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * invDet;
    inv.m[2][2] =  (m[0][0] * (m[1][1] * m[3][3] - m[1][3] * m[3][1]) - m[0][1] * (m[1][0] * m[3][3] - m[1][3] * m[3][0]) + m[0][3] * (m[1][0] * m[3][1] - m[1][1] * m[3][0])) * invDet;
    inv.m[2][3] = -(m[0][0] * (m[1][1] * m[2][3] - m[1][3] * m[2][1]) - m[0][1] * (m[1][0] * m[2][3] - m[1][3] * m[2][0]) + m[0][3] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) * invDet;

    inv.m[3][0] = -(m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) - m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) + m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * invDet;
    inv.m[3][1] =  (m[0][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) - m[0][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) + m[0][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * invDet;
    inv.m[3][2] = -(m[0][0] * (m[1][1] * m[3][2] - m[1][2] * m[3][1]) - m[0][1] * (m[1][0] * m[3][2] - m[1][2] * m[3][0]) + m[0][2] * (m[1][0] * m[3][1] - m[1][1] * m[3][0])) * invDet;
    inv.m[3][3] =  (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) * invDet;

    return inv;
}

bool Mat4::Invert() noexcept {
    Mat4 inv = Inverted();
    if (inv == *this && Determinant() < MMV2_EPSILON) return false;
    *this = inv;
    return true;
}

Vec3 Mat4::GetScale() const noexcept {
    return Vec3(
        Vec3(cols[0].x, cols[0].y, cols[0].z).Length(),
        Vec3(cols[1].x, cols[1].y, cols[1].z).Length(),
        Vec3(cols[2].x, cols[2].y, cols[2].z).Length()
    );
}

void Mat4::SetScale(const Vec3& s) noexcept {
    Vec3 currentScale = GetScale();
    if (currentScale.x > MMV2_EPSILON) cols[0] = Vec4(cols[0].XYZ() * (s.x / currentScale.x), cols[0].w);
    if (currentScale.y > MMV2_EPSILON) cols[1] = Vec4(cols[1].XYZ() * (s.y / currentScale.y), cols[1].w);
    if (currentScale.z > MMV2_EPSILON) cols[2] = Vec4(cols[2].XYZ() * (s.z / currentScale.z), cols[2].w);
}

Quat Mat4::GetRotation() const noexcept {
    return Quat::FromRotationMatrix(*this);
}

void Mat4::SetRotation(const Quat& q) noexcept {
    Vec3 t = GetTranslation();
    Vec3 s = GetScale();
    *this = Compose(t, q, s);
}

void Mat4::Decompose(Vec3& translation, Quat& rotation, Vec3& scale) const noexcept {
    translation = GetTranslation();
    scale = GetScale();
    Mat4 rotMat = *this;
    rotMat.SetTranslation(Vec3::Zero());
    if (scale.x > MMV2_EPSILON) {
        rotMat.m[0][0] /= scale.x; rotMat.m[0][1] /= scale.x; rotMat.m[0][2] /= scale.x;
    }
    if (scale.y > MMV2_EPSILON) {
        rotMat.m[1][0] /= scale.y; rotMat.m[1][1] /= scale.y; rotMat.m[1][2] /= scale.y;
    }
    if (scale.z > MMV2_EPSILON) {
        rotMat.m[2][0] /= scale.z; rotMat.m[2][1] /= scale.z; rotMat.m[2][2] /= scale.z;
    }
    rotation = Quat::FromRotationMatrix(rotMat);
}

Mat4 Mat4::Compose(const Vec3& translation, const Quat& rotation, const Vec3& scale) noexcept {
    Mat4 rot = rotation.ToRotationMatrix();
    Mat4 result;
    result.m[0][0] = rot.m[0][0] * scale.x; result.m[0][1] = rot.m[0][1] * scale.x; result.m[0][2] = rot.m[0][2] * scale.x;
    result.m[1][0] = rot.m[1][0] * scale.y; result.m[1][1] = rot.m[1][1] * scale.y; result.m[1][2] = rot.m[1][2] * scale.y;
    result.m[2][0] = rot.m[2][0] * scale.z; result.m[2][1] = rot.m[2][1] * scale.z; result.m[2][2] = rot.m[2][2] * scale.z;
    result.m[3][0] = 0.0f; result.m[3][1] = 0.0f; result.m[3][2] = 0.0f;
    result.m[0][3] = 0.0f; result.m[1][3] = 0.0f; result.m[2][3] = 0.0f; result.m[3][3] = 1.0f;
    result.SetTranslation(translation);
    return result;
}

Mat4 Mat4::Perspective(float32 fovY, float32 aspect, float32 nearPlane, float32 farPlane) noexcept {
    Mat4 result;
    std::memset(result.flat, 0, sizeof(result.flat));
    float32 tanHalfFov = std::tan(fovY * 0.5f);
    result.m[0][0] = 1.0f / (aspect * tanHalfFov);
    result.m[1][1] = 1.0f / tanHalfFov;
    result.m[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    result.m[2][3] = -1.0f;
    result.m[3][2] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
    return result;
}

Mat4 Mat4::PerspectiveInfinite(float32 fovY, float32 aspect, float32 nearPlane) noexcept {
    Mat4 result;
    std::memset(result.flat, 0, sizeof(result.flat));
    float32 tanHalfFov = std::tan(fovY * 0.5f);
    result.m[0][0] = 1.0f / (aspect * tanHalfFov);
    result.m[1][1] = 1.0f / tanHalfFov;
    result.m[2][2] = -1.0f;
    result.m[2][3] = -1.0f;
    result.m[3][2] = -2.0f * nearPlane;
    return result;
}

Mat4 Mat4::Orthographic(float32 left, float32 right, float32 bottom, float32 top, float32 nearPlane, float32 farPlane) noexcept {
    Mat4 result = Identity();
    result.m[0][0] = 2.0f / (right - left);
    result.m[1][1] = 2.0f / (top - bottom);
    result.m[2][2] = -2.0f / (farPlane - nearPlane);
    result.m[3][0] = -(right + left) / (right - left);
    result.m[3][1] = -(top + bottom) / (top - bottom);
    result.m[3][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    return result;
}

Mat4 Mat4::LookAt(const Vec3& eye, const Vec3& target, const Vec3& up) noexcept {
    Vec3 f = (target - eye).Normalized();
    Vec3 s = f.Cross(up).Normalized();
    Vec3 u = s.Cross(f);

    Mat4 result = Identity();
    result.m[0][0] = s.x; result.m[0][1] = u.x; result.m[0][2] = -f.x;
    result.m[1][0] = s.y; result.m[1][1] = u.y; result.m[1][2] = -f.y;
    result.m[2][0] = s.z; result.m[2][1] = u.z; result.m[2][2] = -f.z;
    result.m[3][0] = -s.Dot(eye); result.m[3][1] = -u.Dot(eye); result.m[3][2] = f.Dot(eye);
    return result;
}

Mat4 Mat4::View(const Vec3& eye, const Vec3& forward, const Vec3& up, const Vec3& right) noexcept {
    Mat4 result = Identity();
    result.m[0][0] = right.x; result.m[0][1] = right.y; result.m[0][2] = right.z;
    result.m[1][0] = up.x;    result.m[1][1] = up.y;    result.m[1][2] = up.z;
    result.m[2][0] = forward.x; result.m[2][1] = forward.y; result.m[2][2] = forward.z;
    result.m[3][0] = -right.Dot(eye); result.m[3][1] = -up.Dot(eye); result.m[3][2] = -forward.Dot(eye);
    return result;
}

MMV2_NAMESPACE_END
