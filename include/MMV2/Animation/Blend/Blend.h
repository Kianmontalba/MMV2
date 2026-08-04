// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Blend Framework
// ============================================================================
// Provides pose blending, cross-fading, blend spaces, and smooth transitions.
// Supports 1D/2D blend spaces, layered blending, and additive poses.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Core/Transform.h"
#include "MMV2/Core/HashMap.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Blend Types
// ============================================================================

enum class BlendType : uint32_t
{
    Linear = 0,         // Standard linear interpolation
    Spherical,          // Spherical linear interpolation (SLERP) for rotations
    Cubic,              // Cubic interpolation (smooth)
    EaseIn,             // Ease in
    EaseOut,            // Ease out
    EaseInOut,          // Ease in-out
    Spring,             // Spring physics-based blend
    Damped,             // Damped spring
    CustomCurve         // Custom blend curve
};

// ============================================================================
// Blend Curve
// ============================================================================

class MMV2_API BlendCurve
{
public:
    BlendCurve();

    void AddKeyframe(float time, float value, float tangent = 0.0f);
    void Clear();

    float Evaluate(float t) const;
    float EvaluateDerivative(float t) const;

    static BlendCurve CreateLinear();
    static BlendCurve CreateEaseIn();
    static BlendCurve CreateEaseOut();
    static BlendCurve CreateEaseInOut();
    static BlendCurve CreateSmoothStep();
    static BlendCurve CreateSpring(float damping = 0.5f, float frequency = 5.0f);

private:
    struct Keyframe
    {
        float time;
        float value;
        float tangent;
        float coefficients[4]; // Cubic Hermite coefficients
    };

    Vector<Keyframe> m_keyframes;

    void ComputeCoefficients();
    int32_t FindSegment(float t) const;
};

// ============================================================================
// Blend Request
// ============================================================================

struct BlendRequest
{
    const Pose* sourcePose;
    const Pose* targetPose;
    float blendWeight;
    BlendType blendType;
    const BlendCurve* customCurve;
    float deltaTime;
    Vector<bool> boneMask;      // Which bones to blend (empty = all)
    float blendSpeed;           // How fast to blend (for inertial blending)
    bool useInertialBlending;   // Use inertial blending (velocity-preserving)

    BlendRequest() : sourcePose(nullptr), targetPose(nullptr), blendWeight(0),
                     blendType(BlendType::Linear), customCurve(nullptr), deltaTime(0),
                     blendSpeed(1.0f), useInertialBlending(false) {}
};

// ============================================================================
// Blend Result
// ============================================================================

struct BlendResult
{
    Pose pose;
    bool completed;             // Blend is complete (weight = 1.0)
    float remainingWeight;      // How much more blending needed
};

// ============================================================================
// Pose Blender
// ============================================================================

class MMV2_API PoseBlender
{
public:
    static void Blend(const BlendRequest& request, BlendResult& result);
    static void BlendPoses(Pose& result, const Pose& a, const Pose& b, float t, BlendType type = BlendType::Linear);
    static void BlendTransforms(Transform& result, const Transform& a, const Transform& b, float t, BlendType type = BlendType::Linear);

    // Specialized blends
    static void BlendWithMask(Pose& result, const Pose& a, const Pose& b, float t, const Vector<bool>& boneMask);
    static void BlendUpperBody(Pose& result, const Pose& fullBody, const Pose& upperBody, float t, uint32_t spineBoneIndex);
    static void BlendLowerBody(Pose& result, const Pose& fullBody, const Pose& lowerBody, float t, uint32_t spineBoneIndex);

    // Inertial blending (velocity-preserving)
    static void InertialBlend(Pose& result, const Pose& current, const Pose& target,
                               const Vector<Vec3>& boneVelocities, const Vector<Vec3>& boneAngularVelocities,
                               float deltaTime, float halfLife);

    // Additive blending
    static void ApplyAdditive(Pose& result, const Pose& base, const Pose& additive, float weight);
    static void ApplyAdditiveTransform(Transform& result, const Transform& base, const Transform& additive, float weight);

    // Override blending
    static void BlendOverride(Pose& result, const Pose& base, const Pose& override, float weight, const Vector<bool>& boneMask);
};

// ============================================================================
// Blend Space
// ============================================================================

struct BlendSpacePoint
{
    float x, y;                 // Coordinates in blend space
    uint32_t animationIndex;    // Animation at this point
    float animationTime;        // Time offset
    float weight;               // Computed weight
};

class MMV2_API BlendSpace1D
{
public:
    BlendSpace1D();

    void SetAxisName(const String& name) { m_axisName = name; }
    String GetAxisName() const { return m_axisName; }

    void SetAxisRange(float min, float max) { m_minX = min; m_maxX = max; }
    float GetMinX() const { return m_minX; }
    float GetMaxX() const { return m_maxX; }

    void AddPoint(const BlendSpacePoint& point);
    void RemovePoint(uint32_t index);
    void ClearPoints();
    uint32_t GetPointCount() const { return static_cast<uint32_t>(m_points.Size()); }

    // Evaluate blend weights for given coordinate
    void Evaluate(float x, Vector<BlendSpacePoint>& outPoints) const;

    // Get animation indices and weights
    void GetWeights(float x, Vector<uint32_t>& outIndices, Vector<float>& outWeights) const;

    // Grid-based optimization
    void BuildGrid(uint32_t gridSize);
    void EvaluateGrid(float x, Vector<BlendSpacePoint>& outPoints) const;

private:
    String m_axisName;
    float m_minX, m_maxX;
    Vector<BlendSpacePoint> m_points;

    // Grid acceleration
    Vector<Vector<uint32_t>> m_grid;
    uint32_t m_gridSize;
    float m_gridStep;
};

class MMV2_API BlendSpace2D
{
public:
    BlendSpace2D();

    void SetAxisNames(const String& xName, const String& yName) { m_xAxisName = xName; m_yAxisName = yName; }
    String GetXAxisName() const { return m_xAxisName; }
    String GetYAxisName() const { return m_yAxisName; }

    void SetXRange(float min, float max) { m_minX = min; m_maxX = max; }
    void SetYRange(float min, float max) { m_minY = min; m_maxY = max; }

    void AddPoint(const BlendSpacePoint& point);
    void RemovePoint(uint32_t index);
    void ClearPoints();
    uint32_t GetPointCount() const { return static_cast<uint32_t>(m_points.Size()); }

    // Triangulation-based evaluation
    void Evaluate(float x, float y, Vector<BlendSpacePoint>& outPoints) const;
    void GetWeights(float x, float y, Vector<uint32_t>& outIndices, Vector<float>& outWeights) const;

    // Delaunay triangulation
    void Triangulate();
    bool IsTriangulated() const { return !m_triangles.Empty(); }

private:
    String m_xAxisName, m_yAxisName;
    float m_minX, m_maxX, m_minY, m_maxY;
    Vector<BlendSpacePoint> m_points;

    struct Triangle
    {
        uint32_t indices[3];
    };
    Vector<Triangle> m_triangles;

    bool PointInTriangle(float x, float y, const Triangle& tri) const;
    void ComputeBarycentric(float x, float y, const Triangle& tri, float& u, float& v, float& w) const;
};

// ============================================================================
// Cross-Fade
// ============================================================================

class MMV2_API CrossFader
{
public:
    CrossFader();

    void StartFade(const Pose& fromPose, const Pose& toPose, float duration, BlendType type = BlendType::EaseInOut);
    void StartFadeIn(const Pose& toPose, float duration, BlendType type = BlendType::EaseOut);
    void StartFadeOut(const Pose& fromPose, float duration, BlendType type = BlendType::EaseIn);

    bool Update(float deltaTime, Pose& outPose);
    bool IsComplete() const { return m_progress >= 1.0f; }
    float GetProgress() const { return m_progress; }

    void Stop() { m_progress = 1.0f; }
    void Reset() { m_progress = 0.0f; m_duration = 0.0f; }

private:
    Pose m_fromPose;
    Pose m_toPose;
    float m_duration;
    float m_progress;
    BlendType m_blendType;
    bool m_active;
};

// ============================================================================
// Layered Blend
// ============================================================================

struct BlendLayer
{
    String name;
    const Pose* pose;
    float weight;
    Vector<bool> boneMask;
    BlendType blendType;
    int32_t priority;
    bool active;

    BlendLayer() : pose(nullptr), weight(0), blendType(BlendType::Linear), priority(0), active(true) {}
};

class MMV2_API LayeredPoseBlender
{
public:
    void AddLayer(const BlendLayer& layer);
    void RemoveLayer(const String& name);
    void SetLayerWeight(const String& name, float weight);
    void SetLayerActive(const String& name, bool active);

    BlendLayer* GetLayer(const String& name);
    const BlendLayer* GetLayer(const String& name) const;

    void ComputeFinalPose(Pose& outPose) const;
    void ComputeFinalPoseWithBase(Pose& outPose, const Pose& basePose) const;

    void ClearLayers();
    uint32_t GetLayerCount() const { return static_cast<uint32_t>(m_layers.Size()); }

private:
    Vector<BlendLayer> m_layers;

    void SortLayersByPriority();
};

// ============================================================================
// Inertial Blending State
// ============================================================================

struct InertialBlendState
{
    Vector<Vec3> bonePositions;
    Vector<Vec3> boneVelocities;
    Vector<Quat> boneRotations;
    Vector<Vec3> boneAngularVelocities;
    Vector<Vec3> boneScales;
    Vector<Vec3> boneScaleVelocities;
    float halfLife;
    float elapsedTime;
    bool active;

    InertialBlendState() : halfLife(0.1f), elapsedTime(0.0f), active(false) {}
};

class MMV2_API InertialBlender
{
public:
    void Initialize(uint32_t boneCount);

    void Start(const Pose& fromPose, const Pose& toPose, float halfLife);
    void Update(float deltaTime, Pose& outPose);
    bool IsActive() const { return m_state.active; }
    void Stop() { m_state.active = false; }

    void SetHalfLife(float halfLife) { m_state.halfLife = halfLife; }
    float GetHalfLife() const { return m_state.halfLife; }

private:
    InertialBlendState m_state;
    Pose m_targetPose;

    static float DampingFactor(float halfLife, float deltaTime);
    static float SpringDamping(float halfLife);
};

MMV2_NAMESPACE_END
