#include "Quat.h"
#include "Mat4.h"

MMV2_NAMESPACE_BEGIN

Quat::Quat(float32 pitch, float32 yaw, float32 roll) noexcept {
    float32 cy = std::cos(yaw * 0.5f);
    float32 sy = std::sin(yaw * 0.5f);
    float32 cp = std::cos(pitch * 0.5f);
    float32 sp = std::sin(pitch * 0.5f);
    float32 cr = std::cos(roll * 0.5f);
    float32 sr = std::sin(roll * 0.5f);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

Vec3 Quat::ToEulerAngles() const noexcept {
    Vec3 angles;
    float32 sinr_cosp = 2.0f * (w * x + y * z);
    float32 cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    float32 sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f)
        angles.y = std::copysign(MMV2_PI_2, sinp);
    else
        angles.y = std::asin(sinp);

    float32 siny_cosp = 2.0f * (w * z + x * y);
    float32 cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quat Quat::FromEulerAngles(float32 pitch, float32 yaw, float32 roll) noexcept {
    return Quat(pitch, yaw, roll);
}

Quat Quat::FromEulerAngles(const Vec3& euler) noexcept {
    return FromEulerAngles(euler.x, euler.y, euler.z);
}

void Quat::ToAxisAngle(Vec3& axis, float32& angle) const noexcept {
    float32 lenSq = x * x + y * y + z * z;
    if (lenSq < MMV2_EPSILON_SQ) {
        axis = Vec3::Up();
        angle = 0.0f;
        return;
    }
    float32 len = std::sqrt(lenSq);
    axis = Vec3(x / len, y / len, z / len);
    angle = 2.0f * std::atan2(len, w);
}

Quat Quat::FromAxisAngle(const Vec3& axis, float32 angle) noexcept {
    return Quat(axis.Normalized(), angle);
}

Quat Quat::FromToRotation(const Vec3& from, const Vec3& to) noexcept {
    Vec3 f = from.Normalized();
    Vec3 t = to.Normalized();
    float32 d = f.Dot(t);

    if (d >= 1.0f - MMV2_EPSILON) return Quat::Identity();
    if (d <= -1.0f + MMV2_EPSILON) {
        Vec3 axis = Vec3::Right().Cross(f);
        if (axis.IsNearZero()) axis = Vec3::Up().Cross(f);
        return Quat(axis.Normalized(), MMV2_PI);
    }

    Vec3 cross = f.Cross(t);
    Quat result(cross.x, cross.y, cross.z, 1.0f + d);
    return result.Normalized();
}

Quat Quat::LookRotation(const Vec3& forward, const Vec3& up) noexcept {
    Vec3 f = forward.Normalized();
    Vec3 r = up.Cross(f).Normalized();
    Vec3 u = f.Cross(r);

    float32 m00 = r.x, m01 = r.y, m02 = r.z;
    float32 m10 = u.x, m11 = u.y, m12 = u.z;
    float32 m20 = f.x, m21 = f.y, m22 = f.z;

    float32 num8 = m00 + m11 + m22;
    Quat q;
    if (num8 > 0.0f) {
        float32 num = std::sqrt(num8 + 1.0f);
        q.w = num * 0.5f;
        num = 0.5f / num;
        q.x = (m12 - m21) * num;
        q.y = (m20 - m02) * num;
        q.z = (m01 - m10) * num;
    } else if ((m00 >= m11) && (m00 >= m22)) {
        float32 num7 = std::sqrt(1.0f + m00 - m11 - m22);
        float32 num4 = 0.5f / num7;
        q.x = 0.5f * num7;
        q.y = (m01 + m10) * num4;
        q.z = (m02 + m20) * num4;
        q.w = (m12 - m21) * num4;
    } else if (m11 > m22) {
        float32 num6 = std::sqrt(1.0f + m11 - m00 - m22);
        float32 num3 = 0.5f / num6;
        q.x = (m10 + m01) * num3;
        q.y = 0.5f * num6;
        q.z = (m21 + m12) * num3;
        q.w = (m20 - m02) * num3;
    } else {
        float32 num5 = std::sqrt(1.0f + m22 - m00 - m11);
        float32 num2 = 0.5f / num5;
        q.x = (m20 + m02) * num2;
        q.y = (m21 + m12) * num2;
        q.z = 0.5f * num5;
        q.w = (m01 - m10) * num2;
    }
    return q;
}

Quat Quat::Slerp(const Quat& a, const Quat& b, float32 t) noexcept {
    float32 dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (dot < 0.0f) {
        Quat negB = Quat(-b.x, -b.y, -b.z, -b.w);
        return Slerp(a, negB, t);
    }
    if (dot > 0.9995f) return Nlerp(a, b, t);

    float32 theta0 = std::acos(dot);
    float32 theta = theta0 * t;
    float32 sinTheta = std::sin(theta);
    float32 sinTheta0 = std::sin(theta0);
    float32 s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    float32 s1 = sinTheta / sinTheta0;
    return Quat(a.x * s0 + b.x * s1, a.y * s0 + b.y * s1, a.z * s0 + b.z * s1, a.w * s0 + b.w * s1);
}

Quat Quat::SlerpShortestPath(const Quat& a, const Quat& b, float32 t) noexcept {
    float32 dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (dot < 0.0f) {
        Quat negB(-b.x, -b.y, -b.z, -b.w);
        return Slerp(a, negB, t);
    }
    return Slerp(a, b, t);
}

MMV2_NAMESPACE_END
