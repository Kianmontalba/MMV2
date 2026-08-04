// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Preview System Implementation
// ============================================================================

#include "MMV2/Editor/Preview/Preview.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Bone.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// PreviewCamera
// ============================================================================

Mat4 PreviewCamera::GetViewMatrix() const
{
    return Mat4::LookAt(position, target, up);
}

Mat4 PreviewCamera::GetProjectionMatrix(float aspectRatio) const
{
    if (isOrthographic)
    {
        float halfH = orthoSize * 0.5f;
        float halfW = halfH * aspectRatio;
        return Mat4::Ortho(-halfW, halfW, -halfH, halfH, nearPlane, farPlane);
    }
    else
    {
        return Mat4::Perspective(fov * MMV2_DEG2RAD, aspectRatio, nearPlane, farPlane);
    }
}

void PreviewCamera::Orbit(float deltaYaw, float deltaPitch)
{
    orbitYaw += deltaYaw;
    orbitPitch = Math::Clamp(orbitPitch + deltaPitch, -89.0f, 89.0f);

    float yawRad = orbitYaw * MMV2_DEG2RAD;
    float pitchRad = orbitPitch * MMV2_DEG2RAD;

    Vec3 offset;
    offset.x = orbitDistance * std::cos(pitchRad) * std::sin(yawRad);
    offset.y = orbitDistance * std::sin(pitchRad);
    offset.z = orbitDistance * std::cos(pitchRad) * std::cos(yawRad);

    position = target + offset;
}

void PreviewCamera::Zoom(float delta)
{
    orbitDistance = Math::Clamp(orbitDistance - delta, 0.1f, 100.0f);

    float yawRad = orbitYaw * MMV2_DEG2RAD;
    float pitchRad = orbitPitch * MMV2_DEG2RAD;

    Vec3 offset;
    offset.x = orbitDistance * std::cos(pitchRad) * std::sin(yawRad);
    offset.y = orbitDistance * std::sin(pitchRad);
    offset.z = orbitDistance * std::cos(pitchRad) * std::cos(yawRad);

    position = target + offset;
}

void PreviewCamera::Pan(float deltaX, float deltaY)
{
    Vec3 right = (target - position).Cross(up).Normalized();
    Vec3 localUp = right.Cross(target - position).Normalized();

    target += right * deltaX + localUp * deltaY;
    position += right * deltaX + localUp * deltaY;
}

void PreviewCamera::FocusOn(const Vec3& point)
{
    target = point;

    float yawRad = orbitYaw * MMV2_DEG2RAD;
    float pitchRad = orbitPitch * MMV2_DEG2RAD;

    Vec3 offset;
    offset.x = orbitDistance * std::cos(pitchRad) * std::sin(yawRad);
    offset.y = orbitDistance * std::sin(pitchRad);
    offset.z = orbitDistance * std::cos(pitchRad) * std::cos(yawRad);

    position = target + offset;
}

void PreviewCamera::Reset()
{
    position = Vec3(0, 2, -5);
    target = Vec3::Zero();
    up = Vec3::Up();
    orbitDistance = 5.0f;
    orbitYaw = 0.0f;
    orbitPitch = 30.0f;
    isOrthographic = false;
    orthoSize = 10.0f;
}

// ============================================================================
// PreviewSystem
// ============================================================================

PreviewSystem::PreviewSystem()
    : m_featureSchema(nullptr)
    , m_hasPose(false)
    , m_hasTrajectory(false)
    , m_hasFeatures(false)
    , m_comparisonEnabled(false)
    , m_isRecording(false)
    , m_recordFps(30)
    , m_recordFrameTime(1.0f / 30.0f)
    , m_recordAccumulator(0.0f)
    , m_recordFrameCount(0)
    , m_initialized(false)
    , m_debugEnabled(false)
{
}

PreviewSystem::~PreviewSystem()
{
    Shutdown();
}

void PreviewSystem::Initialize()
{
    if (m_initialized)
        return;

    m_camera.Reset();
    m_initialized = true;
}

void PreviewSystem::Shutdown()
{
    if (!m_initialized)
        return;

    if (m_isRecording)
    {
        StopRecording();
    }

    m_overlays.Clear();
    m_selectedBones.Clear();
    m_initialized = false;
}

// ============================================================================
// Camera Operations
// ============================================================================

void PreviewSystem::ResetCamera()
{
    m_camera.Reset();
}

void PreviewSystem::FrameOnPose()
{
    if (!m_hasPose || m_currentPose.boneCount == 0)
        return;

    Vec3 center = Vec3::Zero();
    uint32_t count = 0;

    for (int32 i = 0; i < m_currentPose.boneCount; ++i)
    {
        center += m_currentPose.worldPositions[i];
        count++;
    }

    if (count > 0)
    {
        center /= static_cast<float>(count);
        m_camera.FocusOn(center);
    }
}

void PreviewSystem::FrameOnBone(uint32_t boneIndex)
{
    if (!m_hasPose || boneIndex >= static_cast<uint32_t>(m_currentPose.boneCount))
        return;

    m_camera.FocusOn(m_currentPose.worldPositions[boneIndex]);
}

// ============================================================================
// Pose Display
// ============================================================================

void PreviewSystem::SetCurrentPose(const Pose& pose)
{
    m_currentPose = pose;
    m_hasPose = true;
}

void PreviewSystem::SetCurrentPose(const PreviewPoseState& state)
{
    m_currentState = state;
    m_currentPose = state.pose;
    m_hasPose = state.isValid;

    if (state.isValid)
    {
        SetTrajectory(state.trajectory);
        SetFeatureVector(state.features);
    }
}

// ============================================================================
// Animation Playback
// ============================================================================

void PreviewSystem::SetAnimation(uint32_t animationIndex, float duration)
{
    m_animState.animationIndex = animationIndex;
    m_animState.duration = duration;
    m_animState.currentTime = 0.0f;
    m_animState.isPlaying = false;
    m_animState.isPaused = false;
}

void PreviewSystem::Play()
{
    m_animState.isPlaying = true;
    m_animState.isPaused = false;
}

void PreviewSystem::Pause()
{
    m_animState.isPaused = true;
}

void PreviewSystem::Stop()
{
    m_animState.isPlaying = false;
    m_animState.isPaused = false;
    m_animState.currentTime = 0.0f;
}

void PreviewSystem::SetPlaybackSpeed(float speed)
{
    m_animState.playbackSpeed = speed;
}

void PreviewSystem::Seek(float time)
{
    m_animState.currentTime = Math::Clamp(time, 0.0f, m_animState.duration);
}

void PreviewSystem::StepForward(float delta)
{
    m_animState.currentTime = Math::Min(m_animState.currentTime + delta, m_animState.duration);
}

void PreviewSystem::StepBackward(float delta)
{
    m_animState.currentTime = Math::Max(m_animState.currentTime - delta, 0.0f);
}

void PreviewSystem::SetLooping(bool loop)
{
    m_animState.isLooping = loop;
}

// ============================================================================
// Trajectory Display
// ============================================================================

void PreviewSystem::SetTrajectory(const Trajectory& trajectory)
{
    m_trajectory = trajectory;
    m_hasTrajectory = true;
}

void PreviewSystem::ClearTrajectory()
{
    m_hasTrajectory = false;
}

// ============================================================================
// Feature Display
// ============================================================================

void PreviewSystem::SetFeatureVector(const FeatureVector& features)
{
    m_featureVector = features;
    m_hasFeatures = true;
}

void PreviewSystem::ClearFeatureVector()
{
    m_hasFeatures = false;
}

void PreviewSystem::SetFeatureSchema(const FeatureSchema* schema)
{
    m_featureSchema = schema;
}

// ============================================================================
// Render Settings
// ============================================================================

void PreviewSystem::SetRenderSettings(const PreviewRenderSettings& settings)
{
    m_renderSettings = settings;
}

// ============================================================================
// Overlays
// ============================================================================

void PreviewSystem::AddOverlay(const PreviewOverlay& overlay)
{
    m_overlays.PushBack(overlay);
}

void PreviewSystem::ClearOverlays()
{
    m_overlays.Clear();
}

void PreviewSystem::ClearTransientOverlays()
{
    for (int32 i = static_cast<int32>(m_overlays.Size()) - 1; i >= 0; --i)
    {
        if (!m_overlays[i].persistent)
        {
            m_overlays.Erase(i);
        }
    }
}

// ============================================================================
// Bone Selection
// ============================================================================

void PreviewSystem::SelectBone(uint32_t boneIndex)
{
    for (const auto& selected : m_selectedBones)
    {
        if (selected == boneIndex)
            return;
    }
    m_selectedBones.PushBack(boneIndex);
}

void PreviewSystem::DeselectBone(uint32_t boneIndex)
{
    for (size_type i = 0; i < m_selectedBones.Size(); ++i)
    {
        if (m_selectedBones[i] == boneIndex)
        {
            m_selectedBones.Erase(i);
            break;
        }
    }
}

void PreviewSystem::ClearBoneSelection()
{
    m_selectedBones.Clear();
}

bool PreviewSystem::IsBoneSelected(uint32_t boneIndex) const
{
    for (const auto& selected : m_selectedBones)
    {
        if (selected == boneIndex)
            return true;
    }
    return false;
}

// ============================================================================
// Comparison Mode
// ============================================================================

void PreviewSystem::EnableComparison(bool enable)
{
    m_comparisonEnabled = enable;
}

void PreviewSystem::SetComparisonPose(const Pose& pose)
{
    m_comparisonPose = pose;
}

void PreviewSystem::SetComparisonPose(const PreviewPoseState& state)
{
    m_comparisonState = state;
    m_comparisonPose = state.pose;
}

// ============================================================================
// Screenshot & Recording
// ============================================================================

bool PreviewSystem::CaptureScreenshot(const String& filePath)
{
    // TODO: Implement screenshot capture
    if (m_debugEnabled)
    {
        Log::Debug("PreviewSystem: Screenshot saved to %s", filePath.CStr());
    }
    return false;
}

void PreviewSystem::StartRecording(const String& outputPath, uint32_t fps)
{
    if (m_isRecording)
        return;

    m_recordOutputPath = outputPath;
    m_recordFps = fps;
    m_recordFrameTime = 1.0f / static_cast<float>(fps);
    m_recordAccumulator = 0.0f;
    m_recordFrameCount = 0;
    m_isRecording = true;

    if (m_debugEnabled)
    {
        Log::Debug("PreviewSystem: Recording started at %u FPS", fps);
    }
}

void PreviewSystem::StopRecording()
{
    if (!m_isRecording)
        return;

    m_isRecording = false;

    if (m_debugEnabled)
    {
        Log::Debug("PreviewSystem: Recording stopped. %u frames captured.", m_recordFrameCount);
    }
}

// ============================================================================
// Update
// ============================================================================

void PreviewSystem::Update(float deltaTime)
{
    if (!m_initialized)
        return;

    UpdateAnimation(deltaTime);
    UpdateOverlays(deltaTime);

    if (m_isRecording)
    {
        m_recordAccumulator += deltaTime;
        while (m_recordAccumulator >= m_recordFrameTime)
        {
            m_recordAccumulator -= m_recordFrameTime;
            m_recordFrameCount++;
            // TODO: Capture frame for recording
        }
    }
}

void PreviewSystem::UpdateAnimation(float deltaTime)
{
    if (!m_animState.isPlaying || m_animState.isPaused)
        return;

    m_animState.currentTime += deltaTime * m_animState.playbackSpeed;

    if (m_animState.currentTime >= m_animState.duration)
    {
        if (m_animState.isLooping)
        {
            m_animState.currentTime = std::fmod(m_animState.currentTime, m_animState.duration);
        }
        else
        {
            m_animState.currentTime = m_animState.duration;
            m_animState.isPlaying = false;
        }
    }

    // Handle crossfade
    if (m_animState.crossFadeProgress < 1.0f && m_animState.crossFadeTime > 0.0f)
    {
        m_animState.crossFadeProgress += deltaTime / m_animState.crossFadeTime;
        m_animState.crossFadeProgress = Math::Min(m_animState.crossFadeProgress, 1.0f);
    }
}

void PreviewSystem::UpdateOverlays(float deltaTime)
{
    for (int32 i = static_cast<int32>(m_overlays.Size()) - 1; i >= 0; --i)
    {
        if (!m_overlays[i].persistent)
        {
            m_overlays[i].elapsed += deltaTime;
            if (m_overlays[i].elapsed >= m_overlays[i].duration)
            {
                m_overlays.Erase(i);
            }
        }
    }
}

// ============================================================================
// Render
// ============================================================================

void PreviewSystem::Render(float viewportWidth, float viewportHeight)
{
    if (!m_initialized)
        return;

    float aspectRatio = viewportWidth / viewportHeight;

    // Setup camera matrices
    Mat4 viewMatrix = m_camera.GetViewMatrix();
    Mat4 projMatrix = m_camera.GetProjectionMatrix(aspectRatio);

    // Render components
    if (m_renderSettings.showGroundPlane)
        RenderGroundPlane();

    if (m_renderSettings.showGrid)
        RenderGrid();

    if (m_renderSettings.showAxis)
        RenderAxis();

    if (m_hasPose)
    {
        if (m_renderSettings.showSkeleton || m_renderSettings.showBones)
            RenderSkeleton();

        if (m_renderSettings.showJoints)
            RenderJoints();

        if (m_renderSettings.showBoundingBox)
            RenderBoundingBox();

        if (m_renderSettings.showVelocity)
            RenderVelocity();

        if (m_renderSettings.showContactPoints)
            RenderContactPoints();

        if (m_renderSettings.showRootMotion)
            RenderRootMotion();

        RenderBoneLabels();
    }

    if (m_hasTrajectory && m_renderSettings.showTrajectory)
        RenderTrajectory();

    if (m_hasFeatures && m_renderSettings.showFeatures)
        RenderFeatures();

    if (m_comparisonEnabled)
        RenderComparison();

    RenderOverlays();
}

// ============================================================================
// Render Components
// ============================================================================

void PreviewSystem::RenderSkeleton()
{
    // TODO: Implement skeleton rendering using line primitives
    // Draw lines between parent and child bones
}

void PreviewSystem::RenderBones()
{
    // TODO: Implement bone rendering
    // Draw oriented boxes or capsules for each bone
}

void PreviewSystem::RenderJoints()
{
    // TODO: Implement joint rendering
    // Draw spheres at joint positions
}

void PreviewSystem::RenderTrajectory()
{
    if (m_trajectory.sampleCount < 2)
        return;

    // TODO: Implement trajectory path rendering
    // Draw line through trajectory points with color gradient
    for (int32 i = 0; i < m_trajectory.sampleCount - 1; ++i)
    {
        const auto& p0 = m_trajectory.points[i];
        const auto& p1 = m_trajectory.points[i + 1];

        if (p0.isValid && p1.isValid)
        {
            // Render line from p0.position to p1.position
            // Color based on speed or time
        }
    }
}

void PreviewSystem::RenderFeatures()
{
    // TODO: Implement feature visualization
    // Draw feature vector as bars or overlay
}

void PreviewSystem::RenderGroundPlane()
{
    // TODO: Implement ground plane rendering
    // Draw a flat plane at y=0
}

void PreviewSystem::RenderGrid()
{
    // TODO: Implement grid rendering
    // Draw grid lines on ground plane
    float halfSize = m_renderSettings.gridSize * 0.5f;
    int32_t lines = static_cast<int32_t>(m_renderSettings.gridSize / m_renderSettings.gridSpacing);

    for (int32_t i = -lines; i <= lines; ++i)
    {
        float pos = i * m_renderSettings.gridSpacing;
        // Draw horizontal line
        // Draw vertical line
    }
}

void PreviewSystem::RenderAxis()
{
    // TODO: Implement axis rendering
    // Draw X (red), Y (green), Z (blue) axes at origin
}

void PreviewSystem::RenderBoundingBox()
{
    if (!m_hasPose || m_currentPose.boneCount == 0)
        return;

    // Compute bounding box
    Vec3 minBounds = Vec3(FLT_MAX);
    Vec3 maxBounds = Vec3(-FLT_MAX);

    for (int32 i = 0; i < m_currentPose.boneCount; ++i)
    {
        minBounds = Vec3::Min(minBounds, m_currentPose.worldPositions[i]);
        maxBounds = Vec3::Max(maxBounds, m_currentPose.worldPositions[i]);
    }

    // TODO: Render bounding box wireframe
}

void PreviewSystem::RenderVelocity()
{
    if (!m_hasPose || m_currentPose.boneCount == 0)
        return;

    // TODO: Render velocity vectors for bones
}

void PreviewSystem::RenderContactPoints()
{
    if (!m_currentState.isValid)
        return;

    // TODO: Render contact point markers
    for (const auto& point : m_currentState.contactPoints)
    {
        // Render sphere or marker at contact point
    }
}

void PreviewSystem::RenderRootMotion()
{
    // TODO: Implement root motion trail rendering
    // Draw path of root bone over time
}

void PreviewSystem::RenderOverlays()
{
    for (const auto& overlay : m_overlays)
    {
        switch (overlay.type)
        {
            case PreviewOverlayType::Text:
                // TODO: Render text at position
                break;
            case PreviewOverlayType::Line:
                // TODO: Render line from position to endPosition
                break;
            case PreviewOverlayType::Circle:
                // TODO: Render circle at position
                break;
            case PreviewOverlayType::Sphere:
                // TODO: Render sphere at position
                break;
            case PreviewOverlayType::Box:
                // TODO: Render box at position
                break;
            case PreviewOverlayType::Arrow:
                // TODO: Render arrow from position to endPosition
                break;
            case PreviewOverlayType::TrajectoryPath:
                // TODO: Render trajectory path
                break;
            case PreviewOverlayType::BoneLabel:
                // TODO: Render bone name label
                break;
            default:
                break;
        }
    }
}

void PreviewSystem::RenderComparison()
{
    if (!m_comparisonEnabled)
        return;

    // TODO: Render comparison pose with transparency or offset
    // Show both current and comparison poses side by side or overlaid
}

void PreviewSystem::RenderBoneLabels()
{
    if (!m_renderSettings.showSkeleton)
        return;

    // TODO: Render bone name labels near each bone
}

void PreviewSystem::RenderFeatureBars()
{
    if (!m_hasFeatures)
        return;

    // TODO: Render feature values as bar chart overlay
}

// ============================================================================
// Color Helpers
// ============================================================================

Vec3 PreviewSystem::GetBoneColor(uint32_t boneIndex) const
{
    // Return color based on bone properties or selection state
    if (IsBoneSelected(boneIndex))
        return Vec3(1.0f, 1.0f, 0.0f); // Yellow for selected

    // Default bone color from settings
    uint32_t color = m_renderSettings.boneColor;
    return Vec3(
        ((color >> 16) & 0xFF) / 255.0f,
        ((color >> 8) & 0xFF) / 255.0f,
        (color & 0xFF) / 255.0f
    );
}

Vec3 PreviewSystem::GetJointColor(uint32_t boneIndex) const
{
    if (IsBoneSelected(boneIndex))
        return Vec3(1.0f, 0.5f, 0.0f); // Orange for selected

    uint32_t color = m_renderSettings.jointColor;
    return Vec3(
        ((color >> 16) & 0xFF) / 255.0f,
        ((color >> 8) & 0xFF) / 255.0f,
        (color & 0xFF) / 255.0f
    );
}

// ============================================================================
// Debug
// ============================================================================

void PreviewSystem::SetDebugEnabled(bool enabled)
{
    m_debugEnabled = enabled;
}

String PreviewSystem::GetDebugInfo() const
{
    String info;
    info += String::Format("Pose: %s\n", m_hasPose ? "Valid" : "None");
    info += String::Format("Bones: %d\n", m_currentPose.boneCount);
    info += String::Format("Trajectory: %s\n", m_hasTrajectory ? "Yes" : "No");
    info += String::Format("Features: %s\n", m_hasFeatures ? "Yes" : "No");
    info += String::Format("Animation Time: %.3f / %.3f\n",
                           m_animState.currentTime, m_animState.duration);
    info += String::Format("Playing: %s\n", m_animState.isPlaying ? "Yes" : "No");
    info += String::Format("Selected Bones: %zu\n", m_selectedBones.Size());
    info += String::Format("Overlays: %zu\n", m_overlays.Size());
    info += String::Format("Comparison: %s\n", m_comparisonEnabled ? "On" : "Off");
    info += String::Format("Recording: %s (%u frames)\n",
                           m_isRecording ? "Yes" : "No", m_recordFrameCount);
    return info;
}

MMV2_NAMESPACE_END
