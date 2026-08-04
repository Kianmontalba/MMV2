// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Constraints Implementation
// ============================================================================

#include "MMV2/Constraints/Constraints.h"
#include "MMV2/Core/Math.h"
#include <algorithm>

MMV2_NAMESPACE_BEGIN

ConstraintValidator::ConstraintValidator() {}

ConstraintValidator::~ConstraintValidator() {}

void ConstraintValidator::AddConstraintSet(const ConstraintSet& set) {
    m_constraintSets.PushBack(set);
}

void ConstraintValidator::RemoveConstraintSet(int32 index) {
    if (index < 0 || index >= static_cast<int32>(m_constraintSets.Size())) return;
    m_constraintSets.Erase(m_constraintSets.begin() + index);
}

void ConstraintValidator::ClearConstraintSets() {
    m_constraintSets.Clear();
}

bool ConstraintValidator::ValidateTransition(int32 fromEntry, int32 toEntry, const MotionDatabase* database) const {
    if (!database) return true;
    const DatabaseEntry* from = database->GetEntry(fromEntry);
    const DatabaseEntry* to = database->GetEntry(toEntry);
    if (!from || !to) return true;
    return CheckTransitionConstraints(from->clipIndex, to->clipIndex);
}

bool ConstraintValidator::ValidatePose(const Pose& pose) const {
    return CheckPoseConstraints(pose);
}

bool ConstraintValidator::ValidateVelocity(const PoseVelocity& velocity) const {
    return CheckVelocityConstraints(velocity);
}

bool ConstraintValidator::ValidateTrajectory(const Trajectory& trajectory, const Trajectory& desired) const {
    return CheckTrajectoryConstraints(trajectory, desired);
}

float32 ConstraintValidator::ComputeTransitionCost(int32 fromEntry, int32 toEntry, const MotionDatabase* database) const {
    float32 cost = 1.0f;
    if (!database) return cost;

    const DatabaseEntry* from = database->GetEntry(fromEntry);
    const DatabaseEntry* to = database->GetEntry(toEntry);
    if (!from || !to) return cost;

    for (const auto& set : m_constraintSets) {
        if (!set.enabled) continue;
        for (const auto& constraint : set.transitionConstraints) {
            if (constraint.fromClipIndex == from->clipIndex && constraint.toClipIndex == to->clipIndex) {
                if (!constraint.allowed) return 0.0f; // Invalid
                cost *= constraint.costMultiplier;
            }
        }
    }

    return cost;
}

bool ConstraintValidator::CheckTransitionConstraints(int32 fromClip, int32 toClip) const {
    for (const auto& set : m_constraintSets) {
        if (!set.enabled) continue;
        for (const auto& constraint : set.transitionConstraints) {
            if (constraint.fromClipIndex == fromClip && constraint.toClipIndex == toClip) {
                if (!constraint.allowed) return false;
            }
        }
    }
    return true;
}

bool ConstraintValidator::CheckPoseConstraints(const Pose& pose) const {
    for (const auto& set : m_constraintSets) {
        if (!set.enabled) continue;
        for (const auto& constraint : set.poseConstraints) {
            if (!constraint.enabled) continue;
            if (constraint.boneIndex < 0 || constraint.boneIndex >= pose.GetBoneCount()) continue;

            const Transform& bone = pose.GetBoneTransform(constraint.boneIndex);

            // Position check
            if (bone.position.x < constraint.minPosition.x || bone.position.x > constraint.maxPosition.x ||
                bone.position.y < constraint.minPosition.y || bone.position.y > constraint.maxPosition.y ||
                bone.position.z < constraint.minPosition.z || bone.position.z > constraint.maxPosition.z) {
                return false;
            }
        }
    }
    return true;
}

bool ConstraintValidator::CheckVelocityConstraints(const PoseVelocity& velocity) const {
    for (const auto& set : m_constraintSets) {
        if (!set.enabled) continue;
        for (const auto& constraint : set.velocityConstraints) {
            if (!constraint.enabled) continue;
            if (constraint.boneIndex < 0 || constraint.boneIndex >= velocity.linear.Size()) continue;

            const Vec3& vel = velocity.linear[constraint.boneIndex];
            if (vel.Length() > constraint.maxSpeed) return false;
        }
    }
    return true;
}

bool ConstraintValidator::CheckTrajectoryConstraints(const Trajectory& trajectory, const Trajectory& desired) const {
    for (const auto& set : m_constraintSets) {
        if (!set.enabled) continue;
        for (const auto& constraint : set.trajectoryConstraints) {
            if (!constraint.enabled) continue;

            if (trajectory.sampleCount != desired.sampleCount) return false;

            for (int32 i = 0; i < trajectory.sampleCount; ++i) {
                float32 posDev = (trajectory.points[i].position - desired.points[i].position).Length();
                if (posDev > constraint.maxDeviation) return false;

                float32 angle = std::acos(Math::Clamp(trajectory.points[i].direction.Dot(desired.points[i].direction), -1.0f, 1.0f));
                if (angle > constraint.maxAngleChange) return false;
            }
        }
    }
    return true;
}

void ConstraintValidator::SetDefaultConstraints() {
    m_constraintSets.Clear();

    ConstraintSet defaultSet;
    defaultSet.name = "Default";

    // Default transition constraints - allow all
    TransitionConstraint defaultTransition;
    defaultTransition.allowed = true;
    defaultSet.transitionConstraints.PushBack(defaultTransition);

    // Default pose constraints - no limits
    PoseConstraint defaultPose;
    defaultPose.enabled = false;
    defaultSet.poseConstraints.PushBack(defaultPose);

    // Default velocity constraints
    VelocityConstraint defaultVel;
    defaultVel.maxSpeed = 20.0f;
    defaultSet.velocityConstraints.PushBack(defaultVel);

    // Default trajectory constraints
    TrajectoryConstraint defaultTraj;
    defaultSet.trajectoryConstraints.PushBack(defaultTraj);

    m_constraintSets.PushBack(defaultSet);
}

MMV2_NAMESPACE_END
