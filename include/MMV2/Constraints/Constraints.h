// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Constraints System
// ============================================================================

#pragma once
#ifndef MMV2_CONSTRAINTS_H
#define MMV2_CONSTRAINTS_H

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/String.h"
#include "MMV2/Database/Database.h"

MMV2_NAMESPACE_BEGIN

enum class ConstraintType : uint8 {
    None = 0,
    Transition = 1,
    Pose = 2,
    Velocity = 3,
    Trajectory = 4,
    Contact = 5,
    Phase = 6,
    Time = 7,
    Distance = 8,
    Angle = 9,
    Custom = 10
};

struct TransitionConstraint {
    int32 fromClipIndex;
    int32 toClipIndex;
    bool allowed;
    float32 minTransitionTime;
    float32 maxTransitionTime;
    float32 costMultiplier;

    TransitionConstraint()
        : fromClipIndex(-1), toClipIndex(-1), allowed(true),
          minTransitionTime(0.0f), maxTransitionTime(1.0f), costMultiplier(1.0f) {}
};

struct PoseConstraint {
    int32 boneIndex;
    Vec3 minPosition;
    Vec3 maxPosition;
    Quat minRotation;
    Quat maxRotation;
    float32 positionTolerance;
    float32 rotationTolerance;
    bool enabled;

    PoseConstraint()
        : boneIndex(-1), positionTolerance(0.1f), rotationTolerance(15.0f * MMV2_DEG2RAD), enabled(true) {}
};

struct VelocityConstraint {
    int32 boneIndex;
    Vec3 minVelocity;
    Vec3 maxVelocity;
    float32 maxSpeed;
    bool enabled;

    VelocityConstraint()
        : boneIndex(-1), maxSpeed(10.0f), enabled(true) {}
};

struct TrajectoryConstraint {
    float32 maxDeviation;
    float32 maxAngleChange;
    bool enabled;

    TrajectoryConstraint()
        : maxDeviation(1.0f), maxAngleChange(90.0f * MMV2_DEG2RAD), enabled(true) {}
};

struct ConstraintSet {
    String name;
    Vector<TransitionConstraint> transitionConstraints;
    Vector<PoseConstraint> poseConstraints;
    Vector<VelocityConstraint> velocityConstraints;
    Vector<TrajectoryConstraint> trajectoryConstraints;
    bool enabled;

    ConstraintSet() : enabled(true) {}
};

class MMV2_API ConstraintValidator {
public:
    ConstraintValidator();
    ~ConstraintValidator();

    void AddConstraintSet(const ConstraintSet& set);
    void RemoveConstraintSet(int32 index);
    void ClearConstraintSets();

    bool ValidateTransition(int32 fromEntry, int32 toEntry, const MotionDatabase* database) const;
    bool ValidatePose(const Pose& pose) const;
    bool ValidateVelocity(const PoseVelocity& velocity) const;
    bool ValidateTrajectory(const Trajectory& trajectory, const Trajectory& desired) const;

    float32 ComputeTransitionCost(int32 fromEntry, int32 toEntry, const MotionDatabase* database) const;

    void SetDefaultConstraints();

private:
    Vector<ConstraintSet> m_constraintSets;

    bool CheckTransitionConstraints(int32 fromClip, int32 toClip) const;
    bool CheckPoseConstraints(const Pose& pose) const;
    bool CheckVelocityConstraints(const PoseVelocity& velocity) const;
    bool CheckTrajectoryConstraints(const Trajectory& trajectory, const Trajectory& desired) const;
};

MMV2_NAMESPACE_END

#endif
