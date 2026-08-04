// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Motion Warping System
// ============================================================================
// Provides foot locking, motion warping, and trajectory adjustment.
// Supports root motion extraction, foot IK, and terrain adaptation.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Core/Transform.h"
#include "MMV2/Animation/IK/IK.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Foot Lock State
// ============================================================================

enum class FootLockState : uint32_t
{
    Unlocked = 0,
    Locking,
    Locked,
    Unlocking
};

struct FootLockInfo
{
    uint32_t footBoneIndex;
    FootLockState state;
    Transform lockedTransform;
    Transform targetTransform;
    float lockWeight;
    float lockStartTime;
    float lockDuration;
    bool isLeftFoot;

    FootLockInfo() : footBoneIndex(0), state(FootLockState::Unlocked), lockWeight(0),
                     lockStartTime(0), lockDuration(0), isLeftFoot(false) {}
};

// ============================================================================
// Motion Warp Point
// ============================================================================

struct MotionWarpPoint
{
    String name;
    Transform transform;
    float time;
    bool useRotation;
    bool useTranslation;
    bool useScale;
    float weight;

    MotionWarpPoint() : time(0), useRotation(true), useTranslation(true),
                        useScale(false), weight(1.0f) {}
};

// ============================================================================
// Root Motion Extractor
// ============================================================================

struct RootMotionFrame
{
    Transform deltaTransform;
    Vec3 deltaVelocity;
    Vec3 deltaAngularVelocity;
    float deltaTime;
};

class MMV2_API RootMotionExtractor
{
public:
    void ExtractRootMotion(const Vector<Pose>& poses, const Vector<float>& timestamps,
                            Vector<RootMotionFrame>& outFrames);

    void ExtractRootMotionFromSkeleton(const Vector<Pose>& poses, uint32_t rootBoneIndex,
                                        Vector<RootMotionFrame>& outFrames);

    void ApplyRootMotion(Pose& pose, const RootMotionFrame& frame);
    void RemoveRootMotion(Pose& pose, const RootMotionFrame& frame);

    // Trajectory prediction from root motion
    void PredictTrajectory(const Vector<RootMotionFrame>& frames, float timeHorizon,
                            Vector<Transform>& outTrajectory);

    // Root motion smoothing
    void SmoothRootMotion(Vector<RootMotionFrame>& frames, float smoothingFactor);

private:
    Transform ComputeDeltaTransform(const Pose& from, const Pose& to, uint32_t rootBoneIndex);
};

// ============================================================================
// Foot Locker
// ============================================================================

class MMV2_API FootLocker
{
public:
    FootLocker();

    void Initialize(uint32_t leftFootBone, uint32_t rightFootBone);

    void Update(const Pose& currentPose, float currentTime, float deltaTime);
    void Apply(Pose& outPose);

    void LockFoot(uint32_t footBone, const Transform& lockTransform, float duration);
    void UnlockFoot(uint32_t footBone, float transitionTime);

    bool IsFootLocked(uint32_t footBone) const;
    const Transform* GetLockedTransform(uint32_t footBone) const;

    void SetLockThreshold(float velocityThreshold, float heightThreshold);
    void SetUnlockThreshold(float velocityThreshold);

    void AutoDetectFootContacts(const Vector<Pose>& poses, const Vector<float>& timestamps,
                                 float groundHeight, float contactThreshold);

    void SetIKSolver(IIKSolver* solver) { m_ikSolver = solver; }

private:
    FootLockInfo m_leftFoot;
    FootLockInfo m_rightFoot;

    float m_lockVelocityThreshold;
    float m_lockHeightThreshold;
    float m_unlockVelocityThreshold;

    IIKSolver* m_ikSolver;

    FootLockInfo* GetFootInfo(uint32_t boneIndex);
    void UpdateFootLock(FootLockInfo& foot, const Transform& currentTransform,
                        float currentTime, float deltaTime);
};

// ============================================================================
// Motion Warper
// ============================================================================

class MMV2_API MotionWarper
{
public:
    MotionWarper();

    void AddWarpPoint(const MotionWarpPoint& point);
    void RemoveWarpPoint(const String& name);
    void ClearWarpPoints();

    void SetCurrentTime(float time) { m_currentTime = time; }
    void SetTargetTransform(const Transform& transform) { m_targetTransform = transform; }

    // Apply warping to pose
    void WarpPose(Pose& pose, float animationTime);
    void WarpTrajectory(class Trajectory& trajectory, float animationTime);

    // Root motion warping
    void WarpRootMotion(Transform& rootTransform, float animationTime);

    // Foot IK-based warping
    void WarpFeetToGround(Pose& pose, const class TerrainInfo& terrain);
    void WarpFeetToTargets(Pose& pose, const Vector<Transform>& footTargets);

    // Blend space warping
    void WarpBlendSpace(float& x, float& y, const Vector<MotionWarpPoint>& constraints);

    // Enable/disable
    void SetEnabled(bool enabled) { m_enabled = enabled; }
    bool IsEnabled() const { return m_enabled; }

    void SetWarpWeight(float weight) { m_warpWeight = weight; }
    float GetWarpWeight() const { return m_warpWeight; }

private:
    Vector<MotionWarpPoint> m_warpPoints;
    float m_currentTime;
    Transform m_targetTransform;
    bool m_enabled;
    float m_warpWeight;

    Transform InterpolateWarpPoint(float time) const;
    void ApplyWarpToBone(Pose& pose, uint32_t boneIndex, const Transform& warpDelta);
};

// ============================================================================
// Terrain Info
// ============================================================================

struct TerrainInfo
{
    float groundHeight;
    Vec3 groundNormal;
    float slopeAngle;
    String surfaceType;
    float friction;

    TerrainInfo() : groundHeight(0), slopeAngle(0), friction(1.0f) {}
};

// ============================================================================
// Trajectory Warper
// ============================================================================

class MMV2_API TrajectoryWarper
{
public:
    void SetDesiredTrajectory(const class Trajectory& trajectory);
    void SetCurrentTrajectory(const class Trajectory& trajectory);

    void ComputeWarpTransform(Transform& outWarp);
    void ApplyWarpToPose(Pose& pose, const Transform& warp);

    float ComputeTrajectoryError() const;
    bool IsWarpNeeded(float threshold = 0.1f) const;

private:
    class Trajectory* m_desiredTrajectory;
    class Trajectory* m_currentTrajectory;
};

// ============================================================================
// Sync Point
// ============================================================================

struct SyncPoint
{
    String name;
    float sourceTime;
    float targetTime;
    Transform sourceTransform;
    Transform targetTransform;
    bool active;

    SyncPoint() : sourceTime(0), targetTime(0), active(false) {}
};

class MMV2_API MotionSync
{
public:
    void AddSyncPoint(const SyncPoint& point);
    void RemoveSyncPoint(const String& name);

    float GetSyncedTime(float sourceTime) const;
    Transform GetSyncedTransform(float sourceTime) const;

    void BuildSyncCurve();

private:
    Vector<SyncPoint> m_syncPoints;
    BlendCurve m_syncCurve;
};

MMV2_NAMESPACE_END
