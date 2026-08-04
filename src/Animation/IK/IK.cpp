// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// IK System Implementation
// ============================================================================

#include "MMV2/Animation/IK/IK.h"
#include "MMV2/Core/Math.h"
#include <cmath>
#include <algorithm>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// IKChain
// ============================================================================

void IKChain::SetJoints(const Vector<int32>& jointIndices) {
    m_jointIndices = jointIndices;
    m_jointTransforms.Resize(jointIndices.Size());
    m_jointPositions.Resize(jointIndices.Size());
    m_constraints.Resize(jointIndices.Size());
    for (auto& c : m_constraints) c = JointConstraint();
}

void IKChain::SetTarget(const Vec3& position, const Quat& rotation) {
    m_targetPosition = position;
    m_targetRotation = rotation;
    m_hasTarget = true;
}

void IKChain::ClearTarget() {
    m_hasTarget = false;
}

void IKChain::SetBaseTransform(const Transform& transform) {
    m_baseTransform = transform;
}

// ============================================================================
// CCD Solver
// ============================================================================

bool CCDSolver::Solve(IKChain& chain, const IKSolverSettings& settings) {
    if (chain.GetJointCount() < 2 || !chain.HasTarget()) return false;

    const Vec3 target = chain.GetTargetPosition();
    const int32 iterations = settings.maxIterations;
    const float32 tolerance = settings.tolerance;

    for (int32 iter = 0; iter < iterations; ++iter) {
        bool improved = false;

        // Iterate from end effector to base
        for (int32 i = chain.GetJointCount() - 2; i >= 0; --i) {
            Vec3 jointPos = chain.GetJointPosition(i);
            Vec3 endEffector = chain.GetJointPosition(chain.GetJointCount() - 1);

            Vec3 toEnd = (endEffector - jointPos).Normalized();
            Vec3 toTarget = (target - jointPos).Normalized();

            if (toEnd.IsNearZero() || toTarget.IsNearZero()) continue;

            float32 dot = Math::Clamp(toEnd.Dot(toTarget), -1.0f, 1.0f);
            float32 angle = std::acos(dot);

            if (angle < tolerance) continue;

            // Limit angle change per iteration
            angle = std::min(angle, settings.maxAngleChange);

            Vec3 axis = toEnd.Cross(toTarget);
            if (axis.IsNearZero()) {
                // Vectors are parallel or anti-parallel
                if (dot < 0.0f) {
                    axis = Vec3::Up().Cross(toEnd);
                    if (axis.IsNearZero()) axis = Vec3::Right().Cross(toEnd);
                } else {
                    continue;
                }
            }
            axis.Normalize();

            Quat rotation = Quat::FromAxisAngle(axis, angle);
            chain.RotateJoint(i, rotation);

            // Apply constraint
            if (settings.enableConstraints && i < chain.GetConstraintCount()) {
                chain.ApplyConstraint(i);
            }

            improved = true;
        }

        // Check convergence
        float32 error = (chain.GetJointPosition(chain.GetJointCount() - 1) - target).Length();
        if (error < tolerance) return true;

        if (!improved) break;
    }

    return (chain.GetJointPosition(chain.GetJointCount() - 1) - target).Length() < tolerance * 10.0f;
}

// ============================================================================
// FABRIK Solver
// ============================================================================

bool FABRIKSolver::Solve(IKChain& chain, const IKSolverSettings& settings) {
    if (chain.GetJointCount() < 2 || !chain.HasTarget()) return false;

    const int32 jointCount = chain.GetJointCount();
    const Vec3 target = chain.GetTargetPosition();
    const float32 tolerance = settings.tolerance;
    const int32 iterations = settings.maxIterations;

    // Store original positions
    Vector<Vec3> positions;
    positions.Resize(jointCount);
    for (int32 i = 0; i < jointCount; ++i) {
        positions[i] = chain.GetJointPosition(i);
    }

    // Store segment lengths
    Vector<float32> segmentLengths;
    segmentLengths.Resize(jointCount - 1);
    float32 totalLength = 0.0f;
    for (int32 i = 0; i < jointCount - 1; ++i) {
        segmentLengths[i] = (positions[i + 1] - positions[i]).Length();
        totalLength += segmentLengths[i];
    }

    // Check if target is reachable
    float32 rootToTarget = (target - positions[0]).Length();
    if (rootToTarget > totalLength) {
        // Target is unreachable - stretch toward it
        Vec3 direction = (target - positions[0]).Normalized();
        for (int32 i = 1; i < jointCount; ++i) {
            positions[i] = positions[i - 1] + direction * segmentLengths[i - 1];
        }
        for (int32 i = 0; i < jointCount; ++i) {
            chain.SetJointPosition(i, positions[i]);
        }
        return false;
    }

    // FABRIK iterations
    for (int32 iter = 0; iter < iterations; ++iter) {
        // Forward reaching
        positions[jointCount - 1] = target;
        for (int32 i = jointCount - 2; i >= 0; --i) {
            Vec3 dir = (positions[i] - positions[i + 1]).Normalized();
            positions[i] = positions[i + 1] + dir * segmentLengths[i];
        }

        // Backward reaching
        Vec3 basePos = chain.GetBaseTransform().position;
        positions[0] = basePos;
        for (int32 i = 1; i < jointCount; ++i) {
            Vec3 dir = (positions[i] - positions[i - 1]).Normalized();
            positions[i] = positions[i - 1] + dir * segmentLengths[i - 1];
        }

        // Check convergence
        float32 error = (positions[jointCount - 1] - target).Length();
        if (error < tolerance) {
            for (int32 i = 0; i < jointCount; ++i) {
                chain.SetJointPosition(i, positions[i]);
            }
            return true;
        }
    }

    // Apply final positions
    for (int32 i = 0; i < jointCount; ++i) {
        chain.SetJointPosition(i, positions[i]);
    }

    return (positions[jointCount - 1] - target).Length() < tolerance * 10.0f;
}

// ============================================================================
// TwoBone Solver
// ============================================================================

bool TwoBoneSolver::Solve(IKChain& chain, const IKSolverSettings& settings) {
    if (chain.GetJointCount() != 3 || !chain.HasTarget()) return false;

    Vec3 rootPos = chain.GetJointPosition(0);
    Vec3 midPos = chain.GetJointPosition(1);
    Vec3 endPos = chain.GetJointPosition(2);
    Vec3 target = chain.GetTargetPosition();

    float32 lenUpper = (midPos - rootPos).Length();
    float32 lenLower = (endPos - midPos).Length();
    float32 lenTarget = (target - rootPos).Length();

    // Clamp target distance
    float32 maxReach = lenUpper + lenLower;
    if (lenTarget > maxReach) {
        Vec3 dir = (target - rootPos).Normalized();
        target = rootPos + dir * maxReach;
        lenTarget = maxReach;
    }

    // Law of cosines
    float32 cosAngle = (lenUpper * lenUpper + lenTarget * lenTarget - lenLower * lenLower) /
                       (2.0f * lenUpper * lenTarget);
    cosAngle = Math::Clamp(cosAngle, -1.0f, 1.0f);
    float32 angle = std::acos(cosAngle);

    // Compute rotation plane
    Vec3 toTarget = (target - rootPos).Normalized();
    Vec3 currentPlane = (midPos - rootPos).Cross(endPos - midPos);
    Vec3 targetPlane = currentPlane.IsNearZero() ? Vec3::Up() : currentPlane;

    // Rotate upper bone
    Quat upperRotation = Quat::FromToRotation((midPos - rootPos).Normalized(), toTarget);
    upperRotation = Quat::FromAxisAngle(targetPlane, angle) * upperRotation;
    chain.RotateJoint(0, upperRotation);

    // Rotate lower bone to reach target
    Vec3 newMid = rootPos + toTarget * lenUpper;
    Vec3 toEnd = (target - newMid).Normalized();
    Vec3 currentLower = (endPos - midPos).Normalized();
    Quat lowerRotation = Quat::FromToRotation(currentLower, toEnd);
    chain.RotateJoint(1, lowerRotation);

    return true;
}

// ============================================================================
// Limb Solver
// ============================================================================

bool LimbSolver::Solve(IKChain& chain, const IKSolverSettings& settings) {
    // Limb solver is essentially a two-bone solver with pole target
    if (chain.GetJointCount() != 3 || !chain.HasTarget()) return false;

    // First solve with two-bone
    if (!TwoBoneSolver::Solve(chain, settings)) return false;

    // Apply pole vector if set
    if (chain.HasPoleVector()) {
        Vec3 rootPos = chain.GetJointPosition(0);
        Vec3 target = chain.GetTargetPosition();
        Vec3 pole = chain.GetPoleVector();

        Vec3 limbAxis = (target - rootPos).Normalized();
        Vec3 poleAxis = (pole - rootPos).Normalized();
        Vec3 currentMid = chain.GetJointPosition(1);

        // Project pole onto limb plane
        Vec3 projectedPole = poleAxis - limbAxis * poleAxis.Dot(limbAxis);
        if (!projectedPole.IsNearZero()) {
            projectedPole.Normalize();

            Vec3 currentDir = (currentMid - rootPos).Normalized();
            Vec3 projectedCurrent = currentDir - limbAxis * currentDir.Dot(limbAxis);

            if (!projectedCurrent.IsNearZero()) {
                projectedCurrent.Normalize();
                Quat poleRotation = Quat::FromToRotation(projectedCurrent, projectedPole);
                chain.RotateJoint(0, poleRotation);
            }
        }
    }

    return true;
}

// ============================================================================
// LookAt Solver
// ============================================================================

bool LookAtSolver::Solve(IKChain& chain, const IKSolverSettings& settings) {
    if (chain.GetJointCount() < 1 || !chain.HasTarget()) return false;

    Vec3 jointPos = chain.GetJointPosition(0);
    Vec3 target = chain.GetTargetPosition();
    Vec3 forward = (target - jointPos).Normalized();

    if (forward.IsNearZero()) return false;

    Vec3 up = Vec3::Up();
    if (std::abs(forward.Dot(up)) > 0.99f) {
        up = Vec3::Right();
    }

    Quat rotation = Quat::LookRotation(forward, up);
    chain.SetJointRotation(0, rotation);

    return true;
}

// ============================================================================
// Foot Solver
// ============================================================================

bool FootSolver::Solve(IKChain& chain, const IKSolverSettings& settings) {
    if (chain.GetJointCount() < 2 || !chain.HasTarget()) return false;

    // Simple foot IK: place foot at target, adjust ankle
    Vec3 footTarget = chain.GetTargetPosition();
    Vec3 anklePos = chain.GetJointPosition(0);
    Vec3 footPos = chain.GetJointPosition(chain.GetJointCount() - 1);

    // Compute ankle rotation to place foot at target
    Vec3 currentDir = (footPos - anklePos).Normalized();
    Vec3 targetDir = (footTarget - anklePos).Normalized();

    if (!currentDir.IsNearZero() && !targetDir.IsNearZero()) {
        Quat rotation = Quat::FromToRotation(currentDir, targetDir);
        chain.RotateJoint(0, rotation);
    }

    // Set foot position
    chain.SetJointPosition(chain.GetJointCount() - 1, footTarget);

    return true;
}

// ============================================================================
// IK Manager
// ============================================================================

IKManager::IKManager() : m_solver(nullptr), m_currentType(IKSolverType::CCD) {}

IKManager::~IKManager() {
    delete m_solver;
}

void IKManager::SetSolverType(IKSolverType type) {
    if (m_currentType == type && m_solver) return;

    delete m_solver;
    m_solver = nullptr;
    m_currentType = type;

    switch (type) {
        case IKSolverType::CCD: m_solver = new CCDSolver(); break;
        case IKSolverType::FABRIK: m_solver = new FABRIKSolver(); break;
        case IKSolverType::TwoBone: m_solver = new TwoBoneSolver(); break;
        case IKSolverType::Limb: m_solver = new LimbSolver(); break;
        case IKSolverType::LookAt: m_solver = new LookAtSolver(); break;
        case IKSolverType::Foot: m_solver = new FootSolver(); break;
        default: m_solver = new CCDSolver(); break;
    }
}

bool IKManager::Solve(IKChain& chain, const IKSolverSettings& settings) {
    if (!m_solver) SetSolverType(settings.type);
    return m_solver->Solve(chain, settings);
}

bool IKManager::SolveMultiple(Vector<IKChain>& chains, const IKSolverSettings& settings) {
    bool allSuccess = true;
    for (auto& chain : chains) {
        if (!Solve(chain, settings)) {
            allSuccess = false;
        }
    }
    return allSuccess;
}

// ============================================================================
// IKChain Helper Methods
// ============================================================================

void IKChain::RotateJoint(int32 index, const Quat& rotation) {
    if (index < 0 || index >= m_jointTransforms.Size()) return;
    m_jointTransforms[index].rotation = rotation * m_jointTransforms[index].rotation;
    m_jointTransforms[index].rotation.Normalize();
    UpdateJointPositions();
}

void IKChain::SetJointRotation(int32 index, const Quat& rotation) {
    if (index < 0 || index >= m_jointTransforms.Size()) return;
    m_jointTransforms[index].rotation = rotation;
    UpdateJointPositions();
}

void IKChain::SetJointPosition(int32 index, const Vec3& position) {
    if (index < 0 || index >= m_jointPositions.Size()) return;
    m_jointPositions[index] = position;
}

void IKChain::ApplyConstraint(int32 index) {
    if (index < 0 || index >= m_constraints.Size()) return;
    const JointConstraint& constraint = m_constraints[index];
    if (!constraint.enabled) return;

    Quat rot = m_jointTransforms[index].rotation;
    Vec3 euler = rot.ToEulerAngles();

    euler.x = Math::Clamp(euler.x, constraint.minAngleX, constraint.maxAngleX);
    euler.y = Math::Clamp(euler.y, constraint.minAngleY, constraint.maxAngleY);
    euler.z = Math::Clamp(euler.z, constraint.minAngleZ, constraint.maxAngleZ);

    m_jointTransforms[index].rotation = Quat::FromEulerAngles(euler);
}

void IKChain::UpdateJointPositions() {
    if (m_jointTransforms.Empty()) return;

    m_jointPositions[0] = m_baseTransform.position;
    for (size_type i = 1; i < m_jointTransforms.Size(); ++i) {
        Vec3 offset = m_jointTransforms[i - 1].rotation.Rotate(Vec3::Forward());
        // In a real implementation, this would use actual bone lengths
        m_jointPositions[i] = m_jointPositions[i - 1] + offset;
    }
}

MMV2_NAMESPACE_END
