// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Motion Warping Implementation
// ============================================================================

#include "MMV2/Animation/Warp/Warp.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Trajectory/Trajectory.h"
#include <algorithm>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// RootMotionExtractor
// ============================================================================

void RootMotionExtractor::ExtractRootMotion(const Vector<Pose>& poses,
                                             const Vector<float>& timestamps,
                                             Vector<RootMotionFrame>& outFrames)
{
    outFrames.Clear();
    if (poses.Size() < 2)
        return;

    outFrames.Reserve(poses.Size() - 1);

    for (uint32_t i = 1; i < poses.Size(); ++i)
    {
        RootMotionFrame frame;
        frame.deltaTime = timestamps[i] - timestamps[i - 1];

        // Compute delta from root bone (index 0)
        const Transform& prevRoot = poses[i - 1].GetBoneTransform(0);
        const Transform& currRoot = poses[i].GetBoneTransform(0);

        frame.deltaTransform.position = currRoot.position - prevRoot.position;
        frame.deltaTransform.rotation = currRoot.rotation * prevRoot.rotation.Inverse();
        frame.deltaTransform.scale = Vec3(1, 1, 1); // Scale usually doesn't change

        if (frame.deltaTime > 1e-6f)
        {
            frame.deltaVelocity = frame.deltaTransform.position / frame.deltaTime;

            Quat deltaRot = frame.deltaTransform.rotation;
            Vec3 axis;
            float angle;
            deltaRot.ToAxisAngle(axis, angle);
            frame.deltaAngularVelocity = axis * (angle / frame.deltaTime);
        }

        outFrames.PushBack(frame);
    }
}

void RootMotionExtractor::ExtractRootMotionFromSkeleton(const Vector<Pose>& poses, uint32_t rootBoneIndex,
                                                         Vector<RootMotionFrame>& outFrames)
{
    outFrames.Clear();
    if (poses.Size() < 2)
        return;

    outFrames.Reserve(poses.Size() - 1);

    for (uint32_t i = 1; i < poses.Size(); ++i)
    {
        RootMotionFrame frame;
        frame.deltaTime = 1.0f / 30.0f; // Assume 30fps if no timestamps

        const Transform& prev = poses[i - 1].GetBoneTransform(rootBoneIndex);
        const Transform& curr = poses[i].GetBoneTransform(rootBoneIndex);

        frame.deltaTransform.position = curr.position - prev.position;
        frame.deltaTransform.rotation = curr.rotation * prev.rotation.Inverse();

        outFrames.PushBack(frame);
    }
}

void RootMotionExtractor::ApplyRootMotion(Pose& pose, const RootMotionFrame& frame)
{
    Transform& root = pose.GetBoneTransform(0);
    root.position += frame.deltaTransform.position;
    root.rotation = frame.deltaTransform.rotation * root.rotation;
}

void RootMotionExtractor::RemoveRootMotion(Pose& pose, const RootMotionFrame& frame)
{
    Transform& root = pose.GetBoneTransform(0);
    root.position -= frame.deltaTransform.position;
    root.rotation = frame.deltaTransform.rotation.Inverse() * root.rotation;
}

void RootMotionExtractor::PredictTrajectory(const Vector<RootMotionFrame>& frames, float timeHorizon,
                                             Vector<Transform>& outTrajectory)
{
    outTrajectory.Clear();
    if (frames.Empty())
        return;

    Transform current;
    outTrajectory.PushBack(current);

    float accumulatedTime = 0.0f;
    uint32_t frameIdx = 0;

    while (accumulatedTime < timeHorizon && frameIdx < frames.Size())
    {
        current.position += frames[frameIdx].deltaTransform.position;
        current.rotation = frames[frameIdx].deltaTransform.rotation * current.rotation;

        accumulatedTime += frames[frameIdx].deltaTime;
        outTrajectory.PushBack(current);
        ++frameIdx;
    }
}

void RootMotionExtractor::SmoothRootMotion(Vector<RootMotionFrame>& frames, float smoothingFactor)
{
    if (frames.Size() < 3)
        return;

    Vector<Vec3> smoothedPositions;
    smoothedPositions.Reserve(frames.Size());

    for (uint32_t i = 0; i < frames.Size(); ++i)
    {
        Vec3 sum(0, 0, 0);
        float weightSum = 0.0f;

        for (int32_t j = -1; j <= 1; ++j)
        {
            int32_t idx = static_cast<int32_t>(i) + j;
            if (idx >= 0 && idx < static_cast<int32_t>(frames.Size()))
            {
                float weight = 1.0f - std::abs(j) * smoothingFactor;
                sum += frames[idx].deltaTransform.position * weight;
                weightSum += weight;
            }
        }

        smoothedPositions.PushBack(sum / weightSum);
    }

    for (uint32_t i = 0; i < frames.Size(); ++i)
    {
        frames[i].deltaTransform.position = smoothedPositions[i];
    }
}

Transform RootMotionExtractor::ComputeDeltaTransform(const Pose& from, const Pose& to, uint32_t rootBoneIndex)
{
    Transform delta;
    const Transform& a = from.GetBoneTransform(rootBoneIndex);
    const Transform& b = to.GetBoneTransform(rootBoneIndex);

    delta.position = b.position - a.position;
    delta.rotation = b.rotation * a.rotation.Inverse();
    delta.scale = Vec3(1, 1, 1);

    return delta;
}

// ============================================================================
// FootLocker
// ============================================================================

FootLocker::FootLocker()
    : m_lockVelocityThreshold(0.1f), m_lockHeightThreshold(0.05f),
      m_unlockVelocityThreshold(0.2f), m_ikSolver(nullptr)
{
}

void FootLocker::Initialize(uint32_t leftFootBone, uint32_t rightFootBone)
{
    m_leftFoot.footBoneIndex = leftFootBone;
    m_leftFoot.isLeftFoot = true;
    m_leftFoot.state = FootLockState::Unlocked;

    m_rightFoot.footBoneIndex = rightFootBone;
    m_rightFoot.isLeftFoot = false;
    m_rightFoot.state = FootLockState::Unlocked;
}

void FootLocker::Update(const Pose& currentPose, float currentTime, float deltaTime)
{
    const Transform& leftTransform = currentPose.GetBoneTransform(m_leftFoot.footBoneIndex);
    const Transform& rightTransform = currentPose.GetBoneTransform(m_rightFoot.footBoneIndex);

    UpdateFootLock(m_leftFoot, leftTransform, currentTime, deltaTime);
    UpdateFootLock(m_rightFoot, rightTransform, currentTime, deltaTime);
}

void FootLocker::Apply(Pose& outPose)
{
    if (m_leftFoot.state != FootLockState::Unlocked)
    {
        Transform blended = Transform::Lerp(outPose.GetBoneTransform(m_leftFoot.footBoneIndex),
                                             m_leftFoot.lockedTransform, m_leftFoot.lockWeight);
        outPose.SetBoneTransform(m_leftFoot.footBoneIndex, blended);
    }

    if (m_rightFoot.state != FootLockState::Unlocked)
    {
        Transform blended = Transform::Lerp(outPose.GetBoneTransform(m_rightFoot.footBoneIndex),
                                             m_rightFoot.lockedTransform, m_rightFoot.lockWeight);
        outPose.SetBoneTransform(m_rightFoot.footBoneIndex, blended);
    }

    // Apply IK if available
    if (m_ikSolver)
    {
        // Would solve IK to connect locked feet to body
    }
}

void FootLocker::LockFoot(uint32_t footBone, const Transform& lockTransform, float duration)
{
    FootLockInfo* foot = GetFootInfo(footBone);
    if (!foot) return;

    foot->lockedTransform = lockTransform;
    foot->lockDuration = duration;
    foot->state = FootLockState::Locking;
    foot->lockWeight = 0.0f;
}

void FootLocker::UnlockFoot(uint32_t footBone, float transitionTime)
{
    FootLockInfo* foot = GetFootInfo(footBone);
    if (!foot) return;

    foot->state = FootLockState::Unlocking;
    foot->lockDuration = transitionTime;
}

bool FootLocker::IsFootLocked(uint32_t footBone) const
{
    const FootLockInfo* foot = (footBone == m_leftFoot.footBoneIndex) ? &m_leftFoot :
                                (footBone == m_rightFoot.footBoneIndex) ? &m_rightFoot : nullptr;
    return foot && (foot->state == FootLockState::Locked || foot->state == FootLockState::Locking);
}

const Transform* FootLocker::GetLockedTransform(uint32_t footBone) const
{
    const FootLockInfo* foot = (footBone == m_leftFoot.footBoneIndex) ? &m_leftFoot :
                                (footBone == m_rightFoot.footBoneIndex) ? &m_rightFoot : nullptr;
    return (foot && foot->state != FootLockState::Unlocked) ? &foot->lockedTransform : nullptr;
}

void FootLocker::SetLockThreshold(float velocityThreshold, float heightThreshold)
{
    m_lockVelocityThreshold = velocityThreshold;
    m_lockHeightThreshold = heightThreshold;
}

void FootLocker::SetUnlockThreshold(float velocityThreshold)
{
    m_unlockVelocityThreshold = velocityThreshold;
}

void FootLocker::AutoDetectFootContacts(const Vector<Pose>& poses, const Vector<float>& timestamps,
                                         float groundHeight, float contactThreshold)
{
    // Analyze poses to detect when feet are near ground
    for (uint32_t i = 0; i < poses.Size(); ++i)
    {
        const Pose& pose = poses[i];

        float leftHeight = pose.GetBoneTransform(m_leftFoot.footBoneIndex).position.y - groundHeight;
        float rightHeight = pose.GetBoneTransform(m_rightFoot.footBoneIndex).position.y - groundHeight;

        if (std::abs(leftHeight) < contactThreshold)
        {
            // Left foot is in contact
        }
        if (std::abs(rightHeight) < contactThreshold)
        {
            // Right foot is in contact
        }
    }
}

FootLockInfo* FootLocker::GetFootInfo(uint32_t boneIndex)
{
    if (boneIndex == m_leftFoot.footBoneIndex) return &m_leftFoot;
    if (boneIndex == m_rightFoot.footBoneIndex) return &m_rightFoot;
    return nullptr;
}

void FootLocker::UpdateFootLock(FootLockInfo& foot, const Transform& currentTransform,
                                 float currentTime, float deltaTime)
{
    switch (foot.state)
    {
        case FootLockState::Unlocked:
            // Check if we should lock
            break;

        case FootLockState::Locking:
            foot.lockWeight += deltaTime / foot.lockDuration;
            if (foot.lockWeight >= 1.0f)
            {
                foot.lockWeight = 1.0f;
                foot.state = FootLockState::Locked;
            }
            break;

        case FootLockState::Locked:
            // Keep locked
            break;

        case FootLockState::Unlocking:
            foot.lockWeight -= deltaTime / foot.lockDuration;
            if (foot.lockWeight <= 0.0f)
            {
                foot.lockWeight = 0.0f;
                foot.state = FootLockState::Unlocked;
            }
            break;
    }
}

// ============================================================================
// MotionWarper
// ============================================================================

MotionWarper::MotionWarper() : m_currentTime(0), m_enabled(true), m_warpWeight(1.0f)
{
}

void MotionWarper::AddWarpPoint(const MotionWarpPoint& point)
{
    m_warpPoints.PushBack(point);

    // Sort by time
    std::sort(m_warpPoints.Begin(), m_warpPoints.End(),
              [](const MotionWarpPoint& a, const MotionWarpPoint& b)
    {
        return a.time < b.time;
    });
}

void MotionWarper::RemoveWarpPoint(const String& name)
{
    for (uint32_t i = 0; i < m_warpPoints.Size(); ++i)
    {
        if (m_warpPoints[i].name == name)
        {
            m_warpPoints.Erase(i);
            break;
        }
    }
}

void MotionWarper::ClearWarpPoints()
{
    m_warpPoints.Clear();
}

void MotionWarper::WarpPose(Pose& pose, float animationTime)
{
    if (!m_enabled || m_warpPoints.Empty())
        return;

    Transform warpDelta = InterpolateWarpPoint(animationTime);

    // Apply warp to root bone
    Transform& root = pose.GetBoneTransform(0);

    if (warpDelta.useTranslation)
        root.position += warpDelta.position * m_warpWeight;
    if (warpDelta.useRotation)
        root.rotation = warpDelta.rotation * root.rotation;
    if (warpDelta.useScale)
        root.scale = Vec3::Lerp(root.scale, root.scale * warpDelta.scale, m_warpWeight);
}

void MotionWarper::WarpTrajectory(Trajectory& trajectory, float animationTime)
{
    if (!m_enabled)
        return;

    Transform warpDelta = InterpolateWarpPoint(animationTime);

    for (uint32_t i = 0; i < trajectory.GetPointCount(); ++i)
    {
        Vec3 point = trajectory.GetPoint(i);
        point += warpDelta.position * m_warpWeight;
        trajectory.SetPoint(i, point);
    }
}

void MotionWarper::WarpRootMotion(Transform& rootTransform, float animationTime)
{
    if (!m_enabled)
        return;

    Transform warpDelta = InterpolateWarpPoint(animationTime);

    if (warpDelta.useTranslation)
        rootTransform.position += warpDelta.position * m_warpWeight;
    if (warpDelta.useRotation)
        rootTransform.rotation = warpDelta.rotation * rootTransform.rotation;
}

void MotionWarper::WarpFeetToGround(Pose& pose, const TerrainInfo& terrain)
{
    // Would adjust foot bones to match terrain height
    // Simplified: adjust root height based on terrain
    Transform& root = pose.GetBoneTransform(0);
    root.position.y = terrain.groundHeight;
}

void MotionWarper::WarpFeetToTargets(Pose& pose, const Vector<Transform>& footTargets)
{
    // Would use IK to warp feet to targets
    if (!m_ikSolver)
        return;

    for (uint32_t i = 0; i < footTargets.Size(); ++i)
    {
        // Apply IK for each foot target
    }
}

void MotionWarper::WarpBlendSpace(float& x, float& y, const Vector<MotionWarpPoint>& constraints)
{
    // Apply constraints to blend space coordinates
    for (const auto& constraint : constraints)
    {
        // Would adjust x/y based on constraint transforms
    }
}

Transform MotionWarper::InterpolateWarpPoint(float time) const
{
    if (m_warpPoints.Empty())
        return Transform();

    if (m_warpPoints.Size() == 1)
        return m_warpPoints[0].transform;

    // Find surrounding points
    uint32_t leftIdx = 0;
    for (uint32_t i = 0; i < m_warpPoints.Size(); ++i)
    {
        if (m_warpPoints[i].time <= time)
            leftIdx = i;
        else
            break;
    }

    if (leftIdx >= m_warpPoints.Size() - 1)
        return m_warpPoints.Back()->transform;

    uint32_t rightIdx = leftIdx + 1;

    float t = (time - m_warpPoints[leftIdx].time) /
              (m_warpPoints[rightIdx].time - m_warpPoints[leftIdx].time + 1e-6f);
    t = Math::Clamp(t, 0.0f, 1.0f);

    Transform result;
    result.position = Vec3::Lerp(m_warpPoints[leftIdx].transform.position,
                                  m_warpPoints[rightIdx].transform.position, t);
    result.rotation = Quat::Slerp(m_warpPoints[leftIdx].transform.rotation,
                                   m_warpPoints[rightIdx].transform.rotation, t);
    result.scale = Vec3::Lerp(m_warpPoints[leftIdx].transform.scale,
                               m_warpPoints[rightIdx].transform.scale, t);

    return result;
}

void MotionWarper::ApplyWarpToBone(Pose& pose, uint32_t boneIndex, const Transform& warpDelta)
{
    if (boneIndex >= pose.GetBoneCount())
        return;

    Transform& bone = pose.GetBoneTransform(boneIndex);
    bone.position += warpDelta.position * m_warpWeight;
    bone.rotation = warpDelta.rotation * bone.rotation;
}

// ============================================================================
// TrajectoryWarper
// ============================================================================

void TrajectoryWarper::SetDesiredTrajectory(const Trajectory& trajectory)
{
    m_desiredTrajectory = const_cast<Trajectory*>(&trajectory);
}

void TrajectoryWarper::SetCurrentTrajectory(const Trajectory& trajectory)
{
    m_currentTrajectory = const_cast<Trajectory*>(&trajectory);
}

void TrajectoryWarper::ComputeWarpTransform(Transform& outWarp)
{
    if (!m_desiredTrajectory || !m_currentTrajectory)
    {
        outWarp = Transform();
        return;
    }

    // Compute delta between desired and current at first point
    Vec3 desiredPos = m_desiredTrajectory->GetPoint(0);
    Vec3 currentPos = m_currentTrajectory->GetPoint(0);

    outWarp.position = desiredPos - currentPos;
    outWarp.rotation = Quat::Identity();
    outWarp.scale = Vec3(1, 1, 1);
}

void TrajectoryWarper::ApplyWarpToPose(Pose& pose, const Transform& warp)
{
    Transform& root = pose.GetBoneTransform(0);
    root.position += warp.position;
}

float TrajectoryWarper::ComputeTrajectoryError() const
{
    if (!m_desiredTrajectory || !m_currentTrajectory)
        return 0.0f;

    float error = 0.0f;
    uint32_t count = Math::Min(m_desiredTrajectory->GetPointCount(),
                                m_currentTrajectory->GetPointCount());

    for (uint32_t i = 0; i < count; ++i)
    {
        error += (m_desiredTrajectory->GetPoint(i) - m_currentTrajectory->GetPoint(i)).Magnitude();
    }

    return error / static_cast<float>(count);
}

bool TrajectoryWarper::IsWarpNeeded(float threshold) const
{
    return ComputeTrajectoryError() > threshold;
}

// ============================================================================
// MotionSync
// ============================================================================

void MotionSync::AddSyncPoint(const SyncPoint& point)
{
    m_syncPoints.PushBack(point);

    std::sort(m_syncPoints.Begin(), m_syncPoints.End(),
              [](const SyncPoint& a, const SyncPoint& b)
    {
        return a.sourceTime < b.sourceTime;
    });

    BuildSyncCurve();
}

void MotionSync::RemoveSyncPoint(const String& name)
{
    for (uint32_t i = 0; i < m_syncPoints.Size(); ++i)
    {
        if (m_syncPoints[i].name == name)
        {
            m_syncPoints.Erase(i);
            BuildSyncCurve();
            break;
        }
    }
}

float MotionSync::GetSyncedTime(float sourceTime) const
{
    if (m_syncPoints.Empty())
        return sourceTime;

    if (m_syncPoints.Size() == 1)
        return m_syncPoints[0].targetTime;

    // Find surrounding sync points
    uint32_t leftIdx = 0;
    for (uint32_t i = 0; i < m_syncPoints.Size(); ++i)
    {
        if (m_syncPoints[i].sourceTime <= sourceTime)
            leftIdx = i;
        else
            break;
    }

    if (leftIdx >= m_syncPoints.Size() - 1)
        return m_syncPoints.Back()->targetTime;

    uint32_t rightIdx = leftIdx + 1;

    float t = (sourceTime - m_syncPoints[leftIdx].sourceTime) /
              (m_syncPoints[rightIdx].sourceTime - m_syncPoints[leftIdx].sourceTime + 1e-6f);
    t = Math::Clamp(t, 0.0f, 1.0f);

    return Math::Lerp(m_syncPoints[leftIdx].targetTime, m_syncPoints[rightIdx].targetTime, t);
}

Transform MotionSync::GetSyncedTransform(float sourceTime) const
{
    if (m_syncPoints.Empty())
        return Transform();

    uint32_t leftIdx = 0;
    for (uint32_t i = 0; i < m_syncPoints.Size(); ++i)
    {
        if (m_syncPoints[i].sourceTime <= sourceTime)
            leftIdx = i;
        else
            break;
    }

    if (leftIdx >= m_syncPoints.Size() - 1)
        return m_syncPoints.Back()->targetTransform;

    uint32_t rightIdx = leftIdx + 1;

    float t = (sourceTime - m_syncPoints[leftIdx].sourceTime) /
              (m_syncPoints[rightIdx].sourceTime - m_syncPoints[leftIdx].sourceTime + 1e-6f);
    t = Math::Clamp(t, 0.0f, 1.0f);

    Transform result;
    result.position = Vec3::Lerp(m_syncPoints[leftIdx].targetTransform.position,
                                  m_syncPoints[rightIdx].targetTransform.position, t);
    result.rotation = Quat::Slerp(m_syncPoints[leftIdx].targetTransform.rotation,
                                   m_syncPoints[rightIdx].targetTransform.rotation, t);
    result.scale = Vec3::Lerp(m_syncPoints[leftIdx].targetTransform.scale,
                               m_syncPoints[rightIdx].targetTransform.scale, t);

    return result;
}

void MotionSync::BuildSyncCurve()
{
    m_syncCurve.Clear();

    for (const auto& point : m_syncPoints)
    {
        m_syncCurve.AddKeyframe(point.sourceTime, point.targetTime);
    }
}

MMV2_NAMESPACE_END
