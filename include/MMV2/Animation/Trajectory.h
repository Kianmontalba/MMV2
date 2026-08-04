#pragma once
#ifndef MMV2_TRAJECTORY_H
#define MMV2_TRAJECTORY_H

#include "Config.h"
#include "Vec3.h"
#include "Quat.h"
#include "Vector.h"
#include "Span.h"

MMV2_NAMESPACE_BEGIN

struct TrajectoryPoint {
    float32 time;
    Vec3 position;
    Vec3 velocity;
    Vec3 direction;
    Quat orientation;
    float32 speed;
    bool isValid;

    TrajectoryPoint() : time(0.0f), speed(0.0f), isValid(false) {}
};

struct Trajectory {
    Vector<TrajectoryPoint> points;
    float32 totalTime;
    float32 sampleInterval;
    int32 sampleCount;

    Trajectory() : totalTime(1.0f), sampleInterval(0.1f), sampleCount(10) {
        points.Resize(10);
        for (int32 i = 0; i < 10; ++i) {
            points[i].time = i * sampleInterval;
        }
    }

    explicit Trajectory(int32 count, float32 interval = 0.1f)
        : totalTime(count * interval), sampleInterval(interval), sampleCount(count) {
        points.Resize(count);
        for (int32 i = 0; i < count; ++i) {
            points[i].time = i * interval;
        }
    }

    void Resize(int32 count);
    void SetFromVelocities(const Vec3& currentVelocity, const Vec3& currentPosition, const Quat& currentOrientation);
    void SetFromInput(const Vec3& moveInput, float32 maxSpeed, float32 turnSpeed);
    void PredictFromHistory(Span<const Vec3> positions, Span<const Quat> orientations, float32 deltaTime);
    void Smooth(float32 smoothing);
    void ClampSpeed(float32 maxSpeed);
    void ClampHeight(float32 minHeight, float32 maxHeight);
    TrajectoryPoint Sample(float32 time) const;
    float32 DistanceTo(const Trajectory& other) const;
    float32 DistanceToSq(const Trajectory& other) const;
    void BlendWith(const Trajectory& other, float32 t);
    void Lerp(const Trajectory& a, const Trajectory& b, float32 t);
    bool IsValid() const;
    Vec3 GetEndPosition() const;
    Vec3 GetEndDirection() const;
    Quat GetEndOrientation() const;
};

struct TrajectoryGenerator {
    float32 predictionTime;
    float32 sampleInterval;
    int32 sampleCount;
    float32 maxSpeed;
    float32 acceleration;
    float32 deceleration;
    float32 turnSpeed;
    float32 smoothing;

    TrajectoryGenerator()
        : predictionTime(1.0f), sampleInterval(0.1f), sampleCount(10),
          maxSpeed(5.0f), acceleration(10.0f), deceleration(8.0f),
          turnSpeed(720.0f), smoothing(0.3f) {}

    Trajectory Generate(const Vec3& currentPosition, const Vec3& currentVelocity,
                        const Quat& currentOrientation, const Vec3& targetInput) const;
    Trajectory GenerateFromPath(Span<const Vec3> path, float32 speed) const;
    Trajectory GenerateCircle(const Vec3& center, float32 radius, float32 speed, float32 startAngle) const;
    Trajectory GenerateStraightLine(const Vec3& start, const Vec3& end) const;
};

MMV2_NAMESPACE_END

#endif
