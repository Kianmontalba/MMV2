// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Preview System - Editor Tool
// ============================================================================
// Provides real-time preview of animations, poses, and motion matching results.
// Supports 3D viewport rendering, bone visualization, trajectory overlay,
// and feature vector debugging.
// ============================================================================

#pragma once
#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Core/Transform.h"
#include "MMV2/Animation/Trajectory.h"
#include "MMV2/Features/Feature.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Preview Camera
// ============================================================================

struct PreviewCamera
{
    Vec3 position;
    Vec3 target;
    Vec3 up;
    float fov;
    float nearPlane;
    float farPlane;
    float orbitDistance;
    float orbitYaw;
    float orbitPitch;
    bool isOrthographic;
    float orthoSize;

    PreviewCamera()
        : position(Vec3(0, 2, -5)), target(Vec3::Zero()), up(Vec3::Up()),
          fov(60.0f), nearPlane(0.1f), farPlane(1000.0f),
          orbitDistance(5.0f), orbitYaw(0.0f), orbitPitch(30.0f),
          isOrthographic(false), orthoSize(10.0f) {}

    Mat4 GetViewMatrix() const;
    Mat4 GetProjectionMatrix(float aspectRatio) const;
    void Orbit(float deltaYaw, float deltaPitch);
    void Zoom(float delta);
    void Pan(float deltaX, float deltaY);
    void FocusOn(const Vec3& point);
    void Reset();
};

// ============================================================================
// Preview Render Settings
// ============================================================================

struct PreviewRenderSettings
{
    bool showSkeleton;
    bool showBones;
    bool showJoints;
    bool showMesh;
    bool showTrajectory;
    bool showFeatures;
    bool showGroundPlane;
    bool showGrid;
    bool showAxis;
    bool showBoundingBox;
    bool showVelocity;
    bool showContactPoints;
    bool showRootMotion;
    bool wireframe;
    bool shadows;
    float boneThickness;
    float jointSize;
    float trajectoryThickness;
    float gridSize;
    float gridSpacing;
    uint32_t boneColor;
    uint32_t jointColor;
    uint32_t trajectoryColor;
    uint32_t groundColor;
    uint32_t gridColor;
    uint32_t velocityColor;
    uint32_t contactColor;

    PreviewRenderSettings()
        : showSkeleton(true), showBones(true), showJoints(true),
          showMesh(true), showTrajectory(true), showFeatures(false),
          showGroundPlane(true), showGrid(true), showAxis(true),
          showBoundingBox(false), showVelocity(false), showContactPoints(false),
          showRootMotion(false), wireframe(false), shadows(true),
          boneThickness(2.0f), jointSize(0.05f), trajectoryThickness(3.0f),
          gridSize(10.0f), gridSpacing(1.0f),
          boneColor(0xFF00FF00), jointColor(0xFFFF0000),
          trajectoryColor(0xFF0000FF), groundColor(0xFF333333),
          gridColor(0xFF555555), velocityColor(0xFFFFFF00),
          contactColor(0xFF00FFFF) {}
};

// ============================================================================
// Preview Pose State
// ============================================================================

struct PreviewPoseState
{
    Pose pose;
    Trajectory trajectory;
    FeatureVector features;
    Transform rootTransform;
    Vec3 rootVelocity;
    Vec3 rootAngularVelocity;
    Vector<uint32_t> contactBones;
    Vector<Vec3> contactPoints;
    float timestamp;
    float deltaTime;
    uint32_t animationIndex;
    float animationTime;
    bool isValid;

    PreviewPoseState() : timestamp(0), deltaTime(0), animationIndex(0),
                         animationTime(0), isValid(false) {}
};

// ============================================================================
// Preview Animation State
// ============================================================================

struct PreviewAnimationState
{
    uint32_t animationIndex;
    float currentTime;
    float duration;
    float playbackSpeed;
    bool isPlaying;
    bool isLooping;
    bool isPaused;
    float blendWeight;
    float crossFadeTime;
    uint32_t crossFadeTargetIndex;
    float crossFadeProgress;

    PreviewAnimationState()
        : animationIndex(0), currentTime(0), duration(0),
          playbackSpeed(1.0f), isPlaying(false), isLooping(true),
          isPaused(false), blendWeight(1.0f), crossFadeTime(0.3f),
          crossFadeTargetIndex(0), crossFadeProgress(0.0f) {}
};

// ============================================================================
// Preview Overlay
// ============================================================================

enum class PreviewOverlayType : uint32_t
{
    None = 0,
    Text,
    Line,
    Circle,
    Sphere,
    Box,
    Arrow,
    TrajectoryPath,
    BoneLabel
};

struct PreviewOverlay
{
    PreviewOverlayType type;
    Vec3 position;
    Vec3 endPosition;
    Vec3 color;
    float size;
    float thickness;
    String text;
    float duration;
    float elapsed;
    bool persistent;

    PreviewOverlay()
        : type(PreviewOverlayType::None), position(Vec3::Zero()),
          endPosition(Vec3::Zero()), color(Vec3::One()),
          size(1.0f), thickness(1.0f), duration(1.0f),
          elapsed(0.0f), persistent(false) {}
};

// ============================================================================
// Preview System
// ============================================================================

class MMV2_API PreviewSystem
{
public:
    PreviewSystem();
    ~PreviewSystem();

    // === Initialization ===
    void Initialize();
    void Shutdown();

    // === Camera ===
    PreviewCamera& GetCamera() { return m_camera; }
    const PreviewCamera& GetCamera() const { return m_camera; }
    void ResetCamera();
    void FrameOnPose();
    void FrameOnBone(uint32_t boneIndex);

    // === Pose Display ===
    void SetCurrentPose(const Pose& pose);
    void SetCurrentPose(const PreviewPoseState& state);
    const Pose& GetCurrentPose() const { return m_currentPose; }

    // === Animation Playback ===
    void SetAnimation(uint32_t animationIndex, float duration);
    void Play();
    void Pause();
    void Stop();
    void SetPlaybackSpeed(float speed);
    void Seek(float time);
    void StepForward(float delta);
    void StepBackward(float delta);
    void SetLooping(bool loop);
    bool IsPlaying() const { return m_animState.isPlaying; }
    float GetCurrentTime() const { return m_animState.currentTime; }
    float GetDuration() const { return m_animState.duration; }

    // === Trajectory Display ===
    void SetTrajectory(const Trajectory& trajectory);
    void ClearTrajectory();

    // === Feature Display ===
    void SetFeatureVector(const FeatureVector& features);
    void ClearFeatureVector();
    void SetFeatureSchema(const FeatureSchema* schema);

    // === Render Settings ===
    PreviewRenderSettings& GetRenderSettings() { return m_renderSettings; }
    const PreviewRenderSettings& GetRenderSettings() const { return m_renderSettings; }
    void SetRenderSettings(const PreviewRenderSettings& settings);

    // === Overlays ===
    void AddOverlay(const PreviewOverlay& overlay);
    void ClearOverlays();
    void ClearTransientOverlays();
    const Vector<PreviewOverlay>& GetOverlays() const { return m_overlays; }

    // === Bone Selection ===
    void SelectBone(uint32_t boneIndex);
    void DeselectBone(uint32_t boneIndex);
    void ClearBoneSelection();
    bool IsBoneSelected(uint32_t boneIndex) const;
    const Vector<uint32_t>& GetSelectedBones() const { return m_selectedBones; }

    // === Update & Render ===
    void Update(float deltaTime);
    void Render(float viewportWidth, float viewportHeight);

    // === Comparison Mode ===
    void EnableComparison(bool enable);
    void SetComparisonPose(const Pose& pose);
    void SetComparisonPose(const PreviewPoseState& state);
    bool IsComparisonEnabled() const { return m_comparisonEnabled; }

    // === Screenshot ===
    bool CaptureScreenshot(const String& filePath);

    // === Recording ===
    void StartRecording(const String& outputPath, uint32_t fps);
    void StopRecording();
    bool IsRecording() const { return m_isRecording; }

    // === Debug ===
    void SetDebugEnabled(bool enabled);
    String GetDebugInfo() const;

private:
    // Camera
    PreviewCamera m_camera;

    // Pose
    Pose m_currentPose;
    PreviewPoseState m_currentState;
    bool m_hasPose;

    // Animation
    PreviewAnimationState m_animState;

    // Trajectory
    Trajectory m_trajectory;
    bool m_hasTrajectory;

    // Features
    FeatureVector m_featureVector;
    const FeatureSchema* m_featureSchema;
    bool m_hasFeatures;

    // Render settings
    PreviewRenderSettings m_renderSettings;

    // Overlays
    Vector<PreviewOverlay> m_overlays;

    // Bone selection
    Vector<uint32_t> m_selectedBones;

    // Comparison
    bool m_comparisonEnabled;
    Pose m_comparisonPose;
    PreviewPoseState m_comparisonState;

    // Recording
    bool m_isRecording;
    String m_recordOutputPath;
    uint32_t m_recordFps;
    float m_recordFrameTime;
    float m_recordAccumulator;
    uint32_t m_recordFrameCount;

    // State
    bool m_initialized;
    bool m_debugEnabled;

    // Internal
    void UpdateAnimation(float deltaTime);
    void UpdateOverlays(float deltaTime);
    void RenderSkeleton();
    void RenderBones();
    void RenderJoints();
    void RenderTrajectory();
    void RenderFeatures();
    void RenderGroundPlane();
    void RenderGrid();
    void RenderAxis();
    void RenderBoundingBox();
    void RenderVelocity();
    void RenderContactPoints();
    void RenderRootMotion();
    void RenderOverlays();
    void RenderComparison();
    void RenderBoneLabels();
    void RenderFeatureBars();
    Vec3 GetBoneColor(uint32_t boneIndex) const;
    Vec3 GetJointColor(uint32_t boneIndex) const;
};

MMV2_NAMESPACE_END
