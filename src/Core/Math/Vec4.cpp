// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Vec4 Implementation
// ============================================================================

#include "MMV2/Core/Vec4.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

const Vec4 Vec4::Zero(0.0f, 0.0f, 0.0f, 0.0f);
const Vec4 Vec4::One(1.0f, 1.0f, 1.0f, 1.0f);

Vec4::Vec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
Vec4::Vec4(float32 v) : x(v), y(v), z(v), w(v) {}
Vec4::Vec4(float32 x_, float32 y_, float32 z_, float32 w_) : x(x_), y(y_), z(z_), w(w_) {}
Vec4::Vec4(const Vec3& v, float32 w_) : x(v.x), y(v.y), z(v.z), w(w_) {}

Vec4 Vec4::operator+(const Vec4& other) const { return Vec4(x + other.x, y + other.y, z + other.z, w + other.w); }
Vec4 Vec4::operator-(const Vec4& other) const { return Vec4(x - other.x, y - other.y, z - other.z, w - other.w); }
Vec4 Vec4::operator*(float32 scalar) const { return Vec4(x * scalar, y * scalar, z * scalar, w * scalar); }
Vec4 Vec4::operator/(float32 scalar) const { float32 inv = 1.0f / scalar; return Vec4(x * inv, y * inv, z * inv, w * inv); }

Vec4& Vec4::operator+=(const Vec4& other) { x += other.x; y += other.y; z += other.z; w += other.w; return *this; }
Vec4& Vec4::operator-=(const Vec4& other) { x -= other.x; y -= other.y; z -= other.z; w -= other.w; return *this; }

bool Vec4::operator==(const Vec4& other) const { return Math::IsNearEqual(x, other.x) && Math::IsNearEqual(y, other.y) && Math::IsNearEqual(z, other.z) && Math::IsNearEqual(w, other.w); }

float32 Vec4::Dot(const Vec4& other) const { return x * other.x + y * other.y + z * other.z + w * other.w; }
float32 Vec4::Length() const { return std::sqrt(x * x + y * y + z * z + w * w); }
float32 Vec4::LengthSq() const { return x * x + y * y + z * z + w * w; }

Vec4& Vec4::Normalize() {
    float32 len = Length();
    if (len > 1e-6f) { float32 inv = 1.0f / len; x *= inv; y *= inv; z *= inv; w *= inv; }
    return *this;
}

Vec3 Vec4::ToVec3() const { return Vec3(x, y, z); }

Vec4 Vec4::Lerp(const Vec4& a, const Vec4& b, float32 t) { return a + (b - a) * t; }

MMV2_NAMESPACE_END
