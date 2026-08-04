// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Vec3 Implementation
// ============================================================================

#include "MMV2/Core/Vec3.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

const Vec3 Vec3::Zero(0.0f, 0.0f, 0.0f);
const Vec3 Vec3::One(1.0f, 1.0f, 1.0f);
const Vec3 Vec3::Up(0.0f, 1.0f, 0.0f);
const Vec3 Vec3::Down(0.0f, -1.0f, 0.0f);
const Vec3 Vec3::Right(1.0f, 0.0f, 0.0f);
const Vec3 Vec3::Left(-1.0f, 0.0f, 0.0f);
const Vec3 Vec3::Forward(0.0f, 0.0f, 1.0f);
const Vec3 Vec3::Back(0.0f, 0.0f, -1.0f);

Vec3::Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
Vec3::Vec3(float32 v) : x(v), y(v), z(v) {}
Vec3::Vec3(float32 x_, float32 y_, float32 z_) : x(x_), y(y_), z(z_) {}

Vec3 Vec3::operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
Vec3 Vec3::operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
Vec3 Vec3::operator*(float32 scalar) const { return Vec3(x * scalar, y * scalar, z * scalar); }
Vec3 Vec3::operator/(float32 scalar) const { float32 inv = 1.0f / scalar; return Vec3(x * inv, y * inv, z * inv); }
Vec3 Vec3::operator*(const Vec3& other) const { return Vec3(x * other.x, y * other.y, z * other.z); }
Vec3 Vec3::operator/(const Vec3& other) const { return Vec3(x / other.x, y / other.y, z / other.z); }

Vec3& Vec3::operator+=(const Vec3& other) { x += other.x; y += other.y; z += other.z; return *this; }
Vec3& Vec3::operator-=(const Vec3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
Vec3& Vec3::operator*=(float32 scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
Vec3& Vec3::operator/=(float32 scalar) { float32 inv = 1.0f / scalar; x *= inv; y *= inv; z *= inv; return *this; }

bool Vec3::operator==(const Vec3& other) const { return Math::IsNearEqual(x, other.x) && Math::IsNearEqual(y, other.y) && Math::IsNearEqual(z, other.z); }
bool Vec3::operator!=(const Vec3& other) const { return !(*this == other); }

float32 Vec3::Dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }
Vec3 Vec3::Cross(const Vec3& other) const { return Vec3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x); }

float32 Vec3::Length() const { return std::sqrt(x * x + y * y + z * z); }
float32 Vec3::LengthSq() const { return x * x + y * y + z * z; }

Vec3& Vec3::Normalize() {
    float32 len = Length();
    if (len > 1e-6f) { float32 inv = 1.0f / len; x *= inv; y *= inv; z *= inv; }
    return *this;
}

Vec3 Vec3::Normalized() const {
    float32 len = Length();
    if (len > 1e-6f) { float32 inv = 1.0f / len; return Vec3(x * inv, y * inv, z * inv); }
    return Vec3::Zero;
}

float32 Vec3::DistanceTo(const Vec3& other) const { return (*this - other).Length(); }
float32 Vec3::DistanceToSq(const Vec3& other) const { return (*this - other).LengthSq(); }

Vec3 Vec3::Lerp(const Vec3& a, const Vec3& b, float32 t) { return a + (b - a) * t; }
Vec3 Vec3::Slerp(const Vec3& a, const Vec3& b, float32 t) {
    float32 dot = a.Dot(b);
    dot = Math::Clamp(dot, -1.0f, 1.0f);
    float32 theta = std::acos(dot) * t;
    Vec3 relative = (b - a * dot).Normalized();
    return a * std::cos(theta) + relative * std::sin(theta);
}

Vec3 Vec3::Reflect(const Vec3& direction, const Vec3& normal) { return direction - normal * 2.0f * direction.Dot(normal); }
Vec3 Vec3::Project(const Vec3& a, const Vec3& b) { return b * (a.Dot(b) / b.LengthSq()); }
Vec3 Vec3::Reject(const Vec3& a, const Vec3& b) { return a - Project(a, b); }

bool Vec3::IsNearZero() const { return LengthSq() < 1e-6f; }
bool Vec3::IsNormalized() const { return Math::IsNearEqual(LengthSq(), 1.0f); }

float32 Vec3::MinComponent() const { return std::min(x, std::min(y, z)); }
float32 Vec3::MaxComponent() const { return std::max(x, std::max(y, z)); }

Vec3 Vec3::Min(const Vec3& a, const Vec3& b) { return Vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)); }
Vec3 Vec3::Max(const Vec3& a, const Vec3& b) { return Vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)); }
Vec3 Vec3::Clamp(const Vec3& v, const Vec3& min, const Vec3& max) { return Vec3(Math::Clamp(v.x, min.x, max.x), Math::Clamp(v.y, min.y, max.y), Math::Clamp(v.z, min.z, max.z)); }

Vec3 Vec3::Abs() const { return Vec3(std::abs(x), std::abs(y), std::abs(z)); }
Vec3 Vec3::Floor() const { return Vec3(std::floor(x), std::floor(y), std::floor(z)); }
Vec3 Vec3::Ceil() const { return Vec3(std::ceil(x), std::ceil(y), std::ceil(z)); }
Vec3 Vec3::Round() const { return Vec3(std::round(x), std::round(y), std::round(z)); }

Vec3 operator*(float32 scalar, const Vec3& v) { return v * scalar; }

MMV2_NAMESPACE_END
