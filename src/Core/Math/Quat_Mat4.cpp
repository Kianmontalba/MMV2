// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Quat-Mat4 Conversion Implementation
// ============================================================================

#include "MMV2/Core/Quat.h"
#include "MMV2/Core/Mat4.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

Mat4 Quat::ToMat4() const {
    Mat4 result;
    float32 xx = x * x, yy = y * y, zz = z * z;
    float32 xy = x * y, xz = x * z, yz = y * z;
    float32 wx = w * x, wy = w * y, wz = w * z;

    result.m[0][0] = 1.0f - 2.0f * (yy + zz);
    result.m[0][1] = 2.0f * (xy + wz);
    result.m[0][2] = 2.0f * (xz - wy);
    result.m[0][3] = 0.0f;

    result.m[1][0] = 2.0f * (xy - wz);
    result.m[1][1] = 1.0f - 2.0f * (xx + zz);
    result.m[1][2] = 2.0f * (yz + wx);
    result.m[1][3] = 0.0f;

    result.m[2][0] = 2.0f * (xz + wy);
    result.m[2][1] = 2.0f * (yz - wx);
    result.m[2][2] = 1.0f - 2.0f * (xx + yy);
    result.m[2][3] = 0.0f;

    result.m[3][0] = 0.0f;
    result.m[3][1] = 0.0f;
    result.m[3][2] = 0.0f;
    result.m[3][3] = 1.0f;

    return result;
}

Quat Quat::FromMat4(const Mat4& m) {
    float32 trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if (trace > 0.0f) {
        float32 s = 0.5f / std::sqrt(trace + 1.0f);
        return Quat(0.25f / s, (m.m[2][1] - m.m[1][2]) * s, (m.m[0][2] - m.m[2][0]) * s, (m.m[1][0] - m.m[0][1]) * s);
    } else {
        if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
            float32 s = 2.0f * std::sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]);
            return Quat((m.m[2][1] - m.m[1][2]) / s, 0.25f * s, (m.m[0][1] + m.m[1][0]) / s, (m.m[0][2] + m.m[2][0]) / s);
        } else if (m.m[1][1] > m.m[2][2]) {
            float32 s = 2.0f * std::sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]);
            return Quat((m.m[0][2] - m.m[2][0]) / s, (m.m[0][1] + m.m[1][0]) / s, 0.25f * s, (m.m[1][2] + m.m[2][1]) / s);
        } else {
            float32 s = 2.0f * std::sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]);
            return Quat((m.m[1][0] - m.m[0][1]) / s, (m.m[0][2] + m.m[2][0]) / s, (m.m[1][2] + m.m[2][1]) / s, 0.25f * s);
        }
    }
}

MMV2_NAMESPACE_END
