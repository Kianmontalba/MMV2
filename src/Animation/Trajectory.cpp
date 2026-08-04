#include "MMV2/Animation/Trajectory.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

void Trajectory::Resize(int32 count) {
    sampleCount = count;
    totalTime = count * sampleInterval;
    points.Resize(count);
    for (int32 i = 0; i < count; ++i) {
        points[i].time = i * sampleInterval;
    }
}

void Trajectory::SetFromVelocities(const Vec3& currentVelocity, const Vec3& currentPosition, const Quat& currentOrientation) {
    Vec3 pos = currentPosition;
    Vec3 vel = currentVelocity;
    Quat orient = currentOrientation;
    for (int32 i = 0; i < sampleCount; ++i) {
        points[i].position = pos;
        points[i].velocity = vel;
        points[i].direction = vel.IsNearZero() ? orient.Rotate(Vec3::Forward()) : vel.Normalized();
        points[i].orientation = orient;
        points[i].speed = vel.Length();
        points[i].isValid = true;
        pos += vel * sampleInterval;
    }
}

void Trajectory::SetFromInput(const Vec3& moveInput, float32 maxSpeed, float32 turnSpeed) {
    Vec3 pos = Vec3::Zero();
    Vec3 vel = moveInput * maxSpeed;
    Quat orient = Quat::Identity();
    for (int32 i = 0; i < sampleCount; ++i) {
        points[i].position = pos;
        points[i].velocity = vel;
        points[i].direction = vel.IsNearZero() ? Vec3::Forward() : vel.Normalized();
        points[i].orientation = orient;
        points[i].speed = vel.Length();
        points[i].isValid = true;
        pos += vel * sampleInterval;
    }
}

void Trajectory::PredictFromHistory(Span<const Vec3> positions, Span<const Quat> orientations, float32 deltaTime) {
    if (positions.Size() < 2 || orientations.Size() < 2) return;
    Vec3 currentVel = (positions.Back() - positions[positions.Size() - 2]) / deltaTime;
    Quat currentOrient = orientations.Back();
    SetFromVelocities(currentVel, positions.Back(), currentOrient);
}

void Trajectory::Smooth(float32 smoothing) {
    if (sampleCount < 3) return;
    for (int32 pass = 0; pass < 3; ++pass) {
        for (int32 i = 1; i < sampleCount - 1; ++i) {
            points[i].position = Vec3::Lerp(points[i].position,
                (points[i - 1].position + points[i].position + points[i + 1].position) / 3.0f, smoothing);
            points[i].velocity = Vec3::Lerp(points[i].velocity,
                (points[i - 1].velocity + points[i].velocity + points[i + 1].velocity) / 3.0f, smoothing);
        }
    }
}

void Trajectory::ClampSpeed(float32 maxSpeed) {
    for (int32 i = 0; i < sampleCount; ++i) {
        if (points[i].speed > maxSpeed) {
            points[i].velocity = points[i].velocity.Normalized() * maxSpeed;
            points[i].speed = maxSpeed;
        }
    }
}

void Trajectory::ClampHeight(float32 minHeight, float32 maxHeight) {
    for (int32 i = 0; i < sampleCount; ++i) {
        points[i].position.y = std::clamp(points[i].position.y, minHeight, maxHeight);
    }
}

TrajectoryPoint Trajectory::Sample(float32 time) const {
    if (sampleCount == 0) return TrajectoryPoint();
    if (time <= 0.0f) return points[0];
    if (time >= totalTime) return points[sampleCount - 1];
    float32 t = time / sampleInterval;
    int32 idx = static_cast<int32>(t);
    float32 frac = t - idx;
    if (idx >= sampleCount - 1) return points[sampleCount - 1];
    TrajectoryPoint result;
    result.time = time;
    result.position = Vec3::Lerp(points[idx].position, points[idx + 1].position, frac);
    result.velocity = Vec3::Lerp(points[idx].velocity, points[idx + 1].velocity, frac);
    result.direction = Vec3::Slerp(points[idx].direction, points[idx + 1].direction, frac).Normalized();
    result.orientation = Quat::SlerpShortestPath(points[idx].orientation, points[idx + 1].orientation, frac);
    result.speed = points[idx].speed + (points[idx + 1].speed - points[idx].speed) * frac;
    result.isValid = true;
    return result;
}

float32 Trajectory::DistanceTo(const Trajectory& other) const {
    return std::sqrt(DistanceToSq(other));
}

float32 Trajectory::DistanceToSq(const Trajectory& other) const {
    int32 count = std::min(sampleCount, other.sampleCount);
    float32 distSq = 0.0f;
    for (int32 i = 0; i < count; ++i) {
        distSq += points[i].position.DistanceToSq(other.points[i].position);
        distSq += (points[i].velocity - other.points[i].velocity).LengthSq() * 0.1f;
    }
    return distSq;
}

void Trajectory::BlendWith(const Trajectory& other, float32 t) {
    int32 count = std::min(sampleCount, other.sampleCount);
    for (int32 i = 0; i < count; ++i) {
        points[i].position = Vec3::Lerp(points[i].position, other.points[i].position, t);
        points[i].velocity = Vec3::Lerp(points[i].velocity, other.points[i].velocity, t);
        points[i].direction = Vec3::Slerp(points[i].direction, other.points[i].direction, t).Normalized();
        points[i].orientation = Quat::SlerpShortestPath(points[i].orientation, other.points[i].orientation, t);
        points[i].speed = points[i].speed + (other.points[i].speed - points[i].speed) * t;
    }
}

void Trajectory::Lerp(const Trajectory& a, const Trajectory& b, float32 t) {
    int32 count = std::min(a.sampleCount, b.sampleCount);
    Resize(count);
    for (int32 i = 0; i < count; ++i) {
        points[i].position = Vec3::Lerp(a.points[i].position, b.points[i].position, t);
        points[i].velocity = Vec3::Lerp(a.points[i].velocity, b.points[i].velocity, t);
        points[i].direction = Vec3::Slerp(a.points[i].direction, b.points[i].direction, t).Normalized();
        points[i].orientation = Quat::SlerpShortestPath(a.points[i].orientation, b.points[i].orientation, t);
        points[i].speed = a.points[i].speed + (b.points[i].speed - a.points[i].speed) * t;
        points[i].time = a.points[i].time + (b.points[i].time - a.points[i].time) * t;
        points[i].isValid = true;
    }
}

bool Trajectory::IsValid() const {
    for (int32 i = 0; i < sampleCount; ++i) {
        if (!points[i].isValid) return false;
        if (std::isnan(points[i].position.x)) return false;
    }
    return sampleCount > 0;
}

Vec3 Trajectory::GetEndPosition() const {
    return sampleCount > 0 ? points[sampleCount - 1].position : Vec3::Zero();
}

Vec3 Trajectory::GetEndDirection() const {
    return sampleCount > 0 ? points[sampleCount - 1].direction : Vec3::Forward();
}

Quat Trajectory::GetEndOrientation() const {
    return sampleCount > 0 ? points[sampleCount - 1].orientation : Quat::Identity();
}

// TrajectoryGenerator
Trajectory TrajectoryGenerator::Generate(const Vec3& currentPosition, const Vec3& currentVelocity,
                                           const Quat& currentOrientation, const Vec3& targetInput) const {
    Trajectory traj(sampleCount, sampleInterval);
    Vec3 pos = currentPosition;
    Vec3 vel = currentVelocity;
    Quat orient = currentOrientation;

    for (int32 i = 0; i < sampleCount; ++i) {
        // Apply acceleration toward target
        Vec3 targetVel = targetInput * maxSpeed;
        Vec3 deltaVel = targetVel - vel;
        float32 deltaLen = deltaVel.Length();
        if (deltaLen > MMV2_EPSILON) {
            float32 accel = deltaLen > 0.0f ? acceleration : deceleration;
            float32 maxDelta = accel * sampleInterval;
            if (deltaLen > maxDelta) deltaVel = deltaVel.Normalized() * maxDelta;
            vel += deltaVel;
        }

        // Clamp speed
        float32 speed = vel.Length();
        if (speed > maxSpeed) vel = vel.Normalized() * maxSpeed;

        // Update orientation
        if (!vel.IsNearZero()) {
            Vec3 targetDir = vel.Normalized();
            Quat targetOrient = Quat::LookRotation(targetDir, Vec3::Up());
            orient = Quat::SlerpShortestPath(orient, targetOrient,
                std::min(1.0f, turnSpeed * sampleInterval * MMV2_DEG2RAD / orient.AngularDistance(targetOrient)));
        }

        pos += vel * sampleInterval;

        traj.points[i].time = i * sampleInterval;
        traj.points[i].position = pos;
        traj.points[i].velocity = vel;
        traj.points[i].direction = vel.IsNearZero() ? orient.Rotate(Vec3::Forward()) : vel.Normalized();
        traj.points[i].orientation = orient;
        traj.points[i].speed = vel.Length();
        traj.points[i].isValid = true;
    }

    traj.Smooth(smoothing);
    return traj;
}

Trajectory TrajectoryGenerator::GenerateFromPath(Span<const Vec3> path, float32 speed) const {
    Trajectory traj(sampleCount, sampleInterval);
    if (path.Size() < 2) return traj;

    float32 totalDist = 0.0f;
    for (size_type i = 1; i < path.Size(); ++i) {
        totalDist += path[i].DistanceTo(path[i - 1]);
    }

    float32 distPerSample = totalDist / sampleCount;
    Vec3 currentPos = path[0];
    size_type pathIdx = 0;

    for (int32 i = 0; i < sampleCount; ++i) {
        float32 targetDist = distPerSample * i;
        float32 accumulated = 0.0f;
        for (size_type j = 1; j < path.Size(); ++j) {
            float32 segDist = path[j].DistanceTo(path[j - 1]);
            if (accumulated + segDist >= targetDist) {
                float32 t = (targetDist - accumulated) / segDist;
                currentPos = Vec3::Lerp(path[j - 1], path[j], t);
                break;
            }
            accumulated += segDist;
        }

        traj.points[i].position = currentPos;
        traj.points[i].time = i * sampleInterval;
        traj.points[i].isValid = true;
    }

    // Compute velocities from positions
    for (int32 i = 0; i < sampleCount - 1; ++i) {
        traj.points[i].velocity = (traj.points[i + 1].position - traj.points[i].position) / sampleInterval;
        traj.points[i].speed = traj.points[i].velocity.Length();
        traj.points[i].direction = traj.points[i].velocity.Normalized();
    }
    traj.points[sampleCount - 1].velocity = traj.points[sampleCount - 2].velocity;
    traj.points[sampleCount - 1].speed = traj.points[sampleCount - 2].speed;
    traj.points[sampleCount - 1].direction = traj.points[sampleCount - 2].direction;

    return traj;
}

Trajectory TrajectoryGenerator::GenerateCircle(const Vec3& center, float32 radius, float32 speed, float32 startAngle) const {
    Trajectory traj(sampleCount, sampleInterval);
    float32 angularSpeed = speed / radius;
    for (int32 i = 0; i < sampleCount; ++i) {
        float32 angle = startAngle + angularSpeed * i * sampleInterval;
        Vec3 pos(center.x + std::cos(angle) * radius, center.y, center.z + std::sin(angle) * radius);
        Vec3 vel(-std::sin(angle) * speed, 0.0f, std::cos(angle) * speed);
        traj.points[i].position = pos;
        traj.points[i].velocity = vel;
        traj.points[i].direction = vel.Normalized();
        traj.points[i].orientation = Quat::LookRotation(vel, Vec3::Up());
        traj.points[i].speed = speed;
        traj.points[i].time = i * sampleInterval;
        traj.points[i].isValid = true;
    }
    return traj;
}

Trajectory TrajectoryGenerator::GenerateStraightLine(const Vec3& start, const Vec3& end) const {
    Trajectory traj(sampleCount, sampleInterval);
    Vec3 dir = (end - start).Normalized();
    float32 dist = start.DistanceTo(end);
    float32 speed = dist / totalTime;
    for (int32 i = 0; i < sampleCount; ++i) {
        float32 t = static_cast<float32>(i) / (sampleCount - 1);
        traj.points[i].position = Vec3::Lerp(start, end, t);
        traj.points[i].velocity = dir * speed;
        traj.points[i].direction = dir;
        traj.points[i].orientation = Quat::LookRotation(dir, Vec3::Up());
        traj.points[i].speed = speed;
        traj.points[i].time = i * sampleInterval;
        traj.points[i].isValid = true;
    }
    return traj;
}

MMV2_NAMESPACE_END
