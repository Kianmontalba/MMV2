// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Blend Framework Implementation
// ============================================================================

#include "MMV2/Animation/Blend/Blend.h"
#include "MMV2/Core/Math.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// BlendCurve
// ============================================================================

BlendCurve::BlendCurve()
{
}

void BlendCurve::AddKeyframe(float time, float value, float tangent)
{
    Keyframe kf;
    kf.time = time;
    kf.value = value;
    kf.tangent = tangent;
    m_keyframes.PushBack(kf);
    ComputeCoefficients();
}

void BlendCurve::Clear()
{
    m_keyframes.Clear();
}

float BlendCurve::Evaluate(float t) const
{
    if (m_keyframes.Empty())
        return t;
    if (m_keyframes.Size() == 1)
        return m_keyframes[0].value;

    int32_t segment = FindSegment(t);
    if (segment < 0)
        return m_keyframes[0].value;
    if (segment >= static_cast<int32_t>(m_keyframes.Size()) - 1)
        return m_keyframes.Back()->value;

    const Keyframe& a = m_keyframes[segment];
    const Keyframe& b = m_keyframes[segment + 1];

    float dt = b.time - a.time;
    if (dt < 1e-6f)
        return a.value;

    float localT = (t - a.time) / dt;
    float t2 = localT * localT;
    float t3 = t2 * localT;

    return a.coefficients[0] + a.coefficients[1] * localT +
           a.coefficients[2] * t2 + a.coefficients[3] * t3;
}

float BlendCurve::EvaluateDerivative(float t) const
{
    int32_t segment = FindSegment(t);
    if (segment < 0 || segment >= static_cast<int32_t>(m_keyframes.Size()) - 1)
        return 0.0f;

    const Keyframe& a = m_keyframes[segment];
    const Keyframe& b = m_keyframes[segment + 1];

    float dt = b.time - a.time;
    if (dt < 1e-6f)
        return 0.0f;

    float localT = (t - a.time) / dt;
    return (a.coefficients[1] + 2.0f * a.coefficients[2] * localT +
            3.0f * a.coefficients[3] * localT * localT) / dt;
}

BlendCurve BlendCurve::CreateLinear()
{
    BlendCurve curve;
    curve.AddKeyframe(0.0f, 0.0f);
    curve.AddKeyframe(1.0f, 1.0f);
    return curve;
}

BlendCurve BlendCurve::CreateEaseIn()
{
    BlendCurve curve;
    curve.AddKeyframe(0.0f, 0.0f, 0.0f);
    curve.AddKeyframe(1.0f, 1.0f, 2.0f);
    return curve;
}

BlendCurve BlendCurve::CreateEaseOut()
{
    BlendCurve curve;
    curve.AddKeyframe(0.0f, 0.0f, 2.0f);
    curve.AddKeyframe(1.0f, 1.0f, 0.0f);
    return curve;
}

BlendCurve BlendCurve::CreateEaseInOut()
{
    BlendCurve curve;
    curve.AddKeyframe(0.0f, 0.0f, 0.0f);
    curve.AddKeyframe(0.5f, 0.5f, 1.0f);
    curve.AddKeyframe(1.0f, 1.0f, 0.0f);
    return curve;
}

BlendCurve BlendCurve::CreateSmoothStep()
{
    BlendCurve curve;
    curve.AddKeyframe(0.0f, 0.0f);
    curve.AddKeyframe(1.0f, 1.0f);
    // Override with smoothstep function
    return curve;
}

BlendCurve BlendCurve::CreateSpring(float damping, float frequency)
{
    BlendCurve curve;
    // Spring curve would be generated procedurally
    curve.AddKeyframe(0.0f, 0.0f);
    curve.AddKeyframe(1.0f, 1.0f);
    return curve;
}

void BlendCurve::ComputeCoefficients()
{
    for (uint32_t i = 0; i + 1 < m_keyframes.Size(); ++i)
    {
        Keyframe& a = m_keyframes[i];
        Keyframe& b = m_keyframes[i + 1];

        float dt = b.time - a.time;
        if (dt < 1e-6f) dt = 1.0f;

        float m0 = a.tangent * dt;
        float m1 = b.tangent * dt;

        a.coefficients[0] = a.value;
        a.coefficients[1] = m0;
        a.coefficients[2] = 3.0f * (b.value - a.value) - 2.0f * m0 - m1;
        a.coefficients[3] = 2.0f * (a.value - b.value) + m0 + m1;
    }
}

int32_t BlendCurve::FindSegment(float t) const
{
    int32_t low = 0;
    int32_t high = static_cast<int32_t>(m_keyframes.Size()) - 1;

    while (low < high)
    {
        int32_t mid = (low + high) / 2;
        if (m_keyframes[mid].time <= t)
            low = mid + 1;
        else
            high = mid;
    }

    return low - 1;
}

// ============================================================================
// PoseBlender
// ============================================================================

void PoseBlender::Blend(const BlendRequest& request, BlendResult& result)
{
    if (!request.sourcePose || !request.targetPose)
    {
        result.completed = true;
        result.remainingWeight = 0.0f;
        return;
    }

    float t = request.blendWeight;

    // Apply blend curve
    if (request.customCurve)
    {
        t = request.customCurve->Evaluate(t);
    }
    else
    {
        switch (request.blendType)
        {
            case BlendType::EaseIn:     t = t * t; break;
            case BlendType::EaseOut:    t = 1.0f - (1.0f - t) * (1.0f - t); break;
            case BlendType::EaseInOut:  t = t * t * (3.0f - 2.0f * t); break;
            case BlendType::Cubic:      t = t * t * (3.0f - 2.0f * t); break;
            default: break;
        }
    }

    if (!request.boneMask.Empty())
    {
        BlendWithMask(result.pose, *request.sourcePose, *request.targetPose, t, request.boneMask);
    }
    else
    {
        BlendPoses(result.pose, *request.sourcePose, *request.targetPose, t, request.blendType);
    }

    result.completed = (request.blendWeight >= 1.0f);
    result.remainingWeight = 1.0f - request.blendWeight;
}

void PoseBlender::BlendPoses(Pose& result, const Pose& a, const Pose& b, float t, BlendType type)
{
    uint32_t boneCount = a.GetBoneCount();
    result.Resize(boneCount);

    for (uint32_t i = 0; i < boneCount; ++i)
    {
        BlendTransforms(result.GetBoneTransform(i),
                        a.GetBoneTransform(i), b.GetBoneTransform(i), t, type);
    }
}

void PoseBlender::BlendTransforms(Transform& result, const Transform& a, const Transform& b, float t, BlendType type)
{
    // Position - always linear
    result.position = Vec3::Lerp(a.position, b.position, t);

    // Rotation - spherical for smooth rotation
    if (type == BlendType::Spherical || type == BlendType::Cubic)
    {
        result.rotation = Quat::Slerp(a.rotation, b.rotation, t);
    }
    else
    {
        result.rotation = Quat::Lerp(a.rotation, b.rotation, t);
        result.rotation = result.rotation.Normalized();
    }

    // Scale - linear
    result.scale = Vec3::Lerp(a.scale, b.scale, t);
}

void PoseBlender::BlendWithMask(Pose& result, const Pose& a, const Pose& b, float t, const Vector<bool>& boneMask)
{
    uint32_t boneCount = a.GetBoneCount();
    result.Resize(boneCount);

    for (uint32_t i = 0; i < boneCount; ++i)
    {
        if (i < boneMask.Size() && boneMask[i])
        {
            BlendTransforms(result.GetBoneTransform(i),
                            a.GetBoneTransform(i), b.GetBoneTransform(i), t);
        }
        else
        {
            result.SetBoneTransform(i, a.GetBoneTransform(i));
        }
    }
}

void PoseBlender::BlendUpperBody(Pose& result, const Pose& fullBody, const Pose& upperBody, float t, uint32_t spineBoneIndex)
{
    // Blend upper body from upperBody pose, keep lower body from fullBody
    result = fullBody;

    // Would traverse bone hierarchy to find all children of spine
    // For now, blend from spine bone onwards
    uint32_t boneCount = result.GetBoneCount();
    for (uint32_t i = spineBoneIndex; i < boneCount; ++i)
    {
        BlendTransforms(result.GetBoneTransform(i),
                        fullBody.GetBoneTransform(i), upperBody.GetBoneTransform(i), t);
    }
}

void PoseBlender::BlendLowerBody(Pose& result, const Pose& fullBody, const Pose& lowerBody, float t, uint32_t spineBoneIndex)
{
    result = fullBody;

    for (uint32_t i = 0; i < spineBoneIndex && i < result.GetBoneCount(); ++i)
    {
        BlendTransforms(result.GetBoneTransform(i),
                        fullBody.GetBoneTransform(i), lowerBody.GetBoneTransform(i), t);
    }
}

void PoseBlender::InertialBlend(Pose& result, const Pose& current, const Pose& target,
                                 const Vector<Vec3>& boneVelocities, const Vector<Vec3>& boneAngularVelocities,
                                 float deltaTime, float halfLife)
{
    // Simplified inertial blending
    // Would preserve bone velocities and decay over time
    float damping = std::exp(-deltaTime * 5.0f / halfLife);

    uint32_t boneCount = current.GetBoneCount();
    result.Resize(boneCount);

    for (uint32_t i = 0; i < boneCount; ++i)
    {
        const Transform& cur = current.GetBoneTransform(i);
        const Transform& tgt = target.GetBoneTransform(i);

        // Position with velocity preservation
        Vec3 velocity = (i < boneVelocities.Size()) ? boneVelocities[i] : Vec3(0, 0, 0);
        result.GetBoneTransform(i).position = Vec3::Lerp(cur.position + velocity * deltaTime, tgt.position, 1.0f - damping);

        // Rotation
        result.GetBoneTransform(i).rotation = Quat::Slerp(cur.rotation, tgt.rotation, 1.0f - damping);

        // Scale
        result.GetBoneTransform(i).scale = Vec3::Lerp(cur.scale, tgt.scale, 1.0f - damping);
    }
}

void PoseBlender::ApplyAdditive(Pose& result, const Pose& base, const Pose& additive, float weight)
{
    uint32_t boneCount = base.GetBoneCount();
    result.Resize(boneCount);

    for (uint32_t i = 0; i < boneCount; ++i)
    {
        const Transform& b = base.GetBoneTransform(i);
        const Transform& a = additive.GetBoneTransform(i);

        result.GetBoneTransform(i).position = b.position + a.position * weight;

        // Additive rotation
        Quat additiveRot = Quat::Slerp(Quat::Identity(), a.rotation, weight);
        result.GetBoneTransform(i).rotation = (additiveRot * b.rotation).Normalized();

        result.GetBoneTransform(i).scale = b.scale + (a.scale - Vec3(1, 1, 1)) * weight;
    }
}

void PoseBlender::ApplyAdditiveTransform(Transform& result, const Transform& base, const Transform& additive, float weight)
{
    result.position = base.position + additive.position * weight;
    Quat additiveRot = Quat::Slerp(Quat::Identity(), additive.rotation, weight);
    result.rotation = (additiveRot * base.rotation).Normalized();
    result.scale = base.scale + (additive.scale - Vec3(1, 1, 1)) * weight;
}

void PoseBlender::BlendOverride(Pose& result, const Pose& base, const Pose& override, float weight, const Vector<bool>& boneMask)
{
    uint32_t boneCount = base.GetBoneCount();
    result.Resize(boneCount);

    for (uint32_t i = 0; i < boneCount; ++i)
    {
        if (i < boneMask.Size() && boneMask[i])
        {
            BlendTransforms(result.GetBoneTransform(i),
                            base.GetBoneTransform(i), override.GetBoneTransform(i), weight);
        }
        else
        {
            result.SetBoneTransform(i, base.GetBoneTransform(i));
        }
    }
}

// ============================================================================
// BlendSpace1D
// ============================================================================

BlendSpace1D::BlendSpace1D() : m_minX(0.0f), m_maxX(1.0f), m_gridSize(0), m_gridStep(0.0f)
{
}

void BlendSpace1D::AddPoint(const BlendSpacePoint& point)
{
    m_points.PushBack(point);
}

void BlendSpace1D::RemovePoint(uint32_t index)
{
    if (index < m_points.Size())
        m_points.Erase(index);
}

void BlendSpace1D::ClearPoints()
{
    m_points.Clear();
}

void BlendSpace1D::Evaluate(float x, Vector<BlendSpacePoint>& outPoints) const
{
    outPoints.Clear();

    if (m_points.Size() < 2)
    {
        outPoints = m_points;
        return;
    }

    // Clamp x to range
    x = Math::Clamp(x, m_minX, m_maxX);

    // Find two closest points
    int32_t leftIdx = -1;
    int32_t rightIdx = -1;
    float minDistLeft = FLT_MAX;
    float minDistRight = FLT_MAX;

    for (uint32_t i = 0; i < m_points.Size(); ++i)
    {
        float dist = m_points[i].x - x;
        if (dist <= 0 && std::abs(dist) < minDistLeft)
        {
            minDistLeft = std::abs(dist);
            leftIdx = static_cast<int32_t>(i);
        }
        if (dist >= 0 && dist < minDistRight)
        {
            minDistRight = dist;
            rightIdx = static_cast<int32_t>(i);
        }
    }

    if (leftIdx < 0) leftIdx = rightIdx;
    if (rightIdx < 0) rightIdx = leftIdx;
    if (leftIdx < 0) return;

    // Compute weights
    float totalDist = m_points[rightIdx].x - m_points[leftIdx].x;
    if (totalDist < 1e-6f)
    {
        BlendSpacePoint p = m_points[leftIdx];
        p.weight = 1.0f;
        outPoints.PushBack(p);
        return;
    }

    float t = (x - m_points[leftIdx].x) / totalDist;

    BlendSpacePoint left = m_points[leftIdx];
    left.weight = 1.0f - t;
    outPoints.PushBack(left);

    BlendSpacePoint right = m_points[rightIdx];
    right.weight = t;
    outPoints.PushBack(right);
}

void BlendSpace1D::GetWeights(float x, Vector<uint32_t>& outIndices, Vector<float>& outWeights) const
{
    Vector<BlendSpacePoint> points;
    Evaluate(x, points);

    outIndices.Clear();
    outWeights.Clear();

    for (const auto& p : points)
    {
        outIndices.PushBack(p.animationIndex);
        outWeights.PushBack(p.weight);
    }
}

void BlendSpace1D::BuildGrid(uint32_t gridSize)
{
    m_gridSize = gridSize;
    m_gridStep = (m_maxX - m_minX) / static_cast<float>(gridSize);
    m_grid.Resize(gridSize);

    for (uint32_t i = 0; i < gridSize; ++i)
    {
        float x = m_minX + i * m_gridStep;
        Vector<BlendSpacePoint> points;
        Evaluate(x, points);

        m_grid[i].Clear();
        for (const auto& p : points)
        {
            m_grid[i].PushBack(p.animationIndex);
        }
    }
}

void BlendSpace1D::EvaluateGrid(float x, Vector<BlendSpacePoint>& outPoints) const
{
    if (m_grid.Empty())
    {
        Evaluate(x, outPoints);
        return;
    }

    uint32_t gridIdx = static_cast<uint32_t>((x - m_minX) / m_gridStep);
    gridIdx = Math::Min(gridIdx, m_gridSize - 1);

    // Return points from grid cell
    outPoints.Clear();
    for (uint32_t animIdx : m_grid[gridIdx])
    {
        for (const auto& p : m_points)
        {
            if (p.animationIndex == animIdx)
            {
                outPoints.PushBack(p);
                break;
            }
        }
    }
}

// ============================================================================
// BlendSpace2D
// ============================================================================

BlendSpace2D::BlendSpace2D() : m_minX(0.0f), m_maxX(1.0f), m_minY(0.0f), m_maxY(1.0f)
{
}

void BlendSpace2D::AddPoint(const BlendSpacePoint& point)
{
    m_points.PushBack(point);
}

void BlendSpace2D::RemovePoint(uint32_t index)
{
    if (index < m_points.Size())
        m_points.Erase(index);
}

void BlendSpace2D::ClearPoints()
{
    m_points.Clear();
    m_triangles.Clear();
}

void BlendSpace2D::Evaluate(float x, float y, Vector<BlendSpacePoint>& outPoints) const
{
    outPoints.Clear();

    if (m_points.Size() < 3)
    {
        outPoints = m_points;
        for (auto& p : outPoints)
            p.weight = 1.0f / static_cast<float>(outPoints.Size());
        return;
    }

    x = Math::Clamp(x, m_minX, m_maxX);
    y = Math::Clamp(y, m_minY, m_maxY);

    if (!m_triangles.Empty())
    {
        // Find containing triangle
        for (const auto& tri : m_triangles)
        {
            if (PointInTriangle(x, y, tri))
            {
                float u, v, w;
                ComputeBarycentric(x, y, tri, u, v, w);

                BlendSpacePoint p0 = m_points[tri.indices[0]];
                p0.weight = u;
                outPoints.PushBack(p0);

                BlendSpacePoint p1 = m_points[tri.indices[1]];
                p1.weight = v;
                outPoints.PushBack(p1);

                BlendSpacePoint p2 = m_points[tri.indices[2]];
                p2.weight = w;
                outPoints.PushBack(p2);

                return;
            }
        }
    }

    // Fallback: nearest neighbor
    float minDist = FLT_MAX;
    uint32_t nearestIdx = 0;

    for (uint32_t i = 0; i < m_points.Size(); ++i)
    {
        float dx = m_points[i].x - x;
        float dy = m_points[i].y - y;
        float dist = dx * dx + dy * dy;

        if (dist < minDist)
        {
            minDist = dist;
            nearestIdx = i;
        }
    }

    BlendSpacePoint p = m_points[nearestIdx];
    p.weight = 1.0f;
    outPoints.PushBack(p);
}

void BlendSpace2D::GetWeights(float x, float y, Vector<uint32_t>& outIndices, Vector<float>& outWeights) const
{
    Vector<BlendSpacePoint> points;
    Evaluate(x, y, points);

    outIndices.Clear();
    outWeights.Clear();

    for (const auto& p : points)
    {
        outIndices.PushBack(p.animationIndex);
        outWeights.PushBack(p.weight);
    }
}

void BlendSpace2D::Triangulate()
{
    m_triangles.Clear();

    if (m_points.Size() < 3)
        return;

    // Simplified triangulation: fan from first point
    for (uint32_t i = 1; i + 1 < m_points.Size(); ++i)
    {
        Triangle tri;
        tri.indices[0] = 0;
        tri.indices[1] = i;
        tri.indices[2] = i + 1;
        m_triangles.PushBack(tri);
    }
}

bool BlendSpace2D::PointInTriangle(float x, float y, const Triangle& tri) const
{
    const BlendSpacePoint& a = m_points[tri.indices[0]];
    const BlendSpacePoint& b = m_points[tri.indices[1]];
    const BlendSpacePoint& c = m_points[tri.indices[2]];

    float denom = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if (std::abs(denom) < 1e-6f)
        return false;

    float u = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / denom;
    float v = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / denom;
    float w = 1.0f - u - v;

    return (u >= 0.0f && v >= 0.0f && w >= 0.0f);
}

void BlendSpace2D::ComputeBarycentric(float x, float y, const Triangle& tri, float& u, float& v, float& w) const
{
    const BlendSpacePoint& a = m_points[tri.indices[0]];
    const BlendSpacePoint& b = m_points[tri.indices[1]];
    const BlendSpacePoint& c = m_points[tri.indices[2]];

    float denom = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if (std::abs(denom) < 1e-6f)
    {
        u = v = w = 1.0f / 3.0f;
        return;
    }

    u = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / denom;
    v = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / denom;
    w = 1.0f - u - v;
}

// ============================================================================
// CrossFader
// ============================================================================

CrossFader::CrossFader() : m_duration(0.0f), m_progress(0.0f), m_blendType(BlendType::Linear), m_active(false)
{
}

void CrossFader::StartFade(const Pose& fromPose, const Pose& toPose, float duration, BlendType type)
{
    m_fromPose = fromPose;
    m_toPose = toPose;
    m_duration = duration;
    m_progress = 0.0f;
    m_blendType = type;
    m_active = true;
}

void CrossFader::StartFadeIn(const Pose& toPose, float duration, BlendType type)
{
    m_fromPose.Resize(toPose.GetBoneCount());
    m_toPose = toPose;
    m_duration = duration;
    m_progress = 0.0f;
    m_blendType = type;
    m_active = true;
}

void CrossFader::StartFadeOut(const Pose& fromPose, float duration, BlendType type)
{
    m_fromPose = fromPose;
    m_toPose.Resize(fromPose.GetBoneCount());
    m_duration = duration;
    m_progress = 0.0f;
    m_blendType = type;
    m_active = true;
}

bool CrossFader::Update(float deltaTime, Pose& outPose)
{
    if (!m_active || m_duration <= 0.0f)
    {
        outPose = m_toPose;
        return true;
    }

    m_progress += deltaTime / m_duration;
    m_progress = Math::Clamp(m_progress, 0.0f, 1.0f);

    float t = m_progress;
    switch (m_blendType)
    {
        case BlendType::EaseIn:     t = t * t; break;
        case BlendType::EaseOut:    t = 1.0f - (1.0f - t) * (1.0f - t); break;
        case BlendType::EaseInOut:  t = t * t * (3.0f - 2.0f * t); break;
        default: break;
    }

    PoseBlender::BlendPoses(outPose, m_fromPose, m_toPose, t);

    if (m_progress >= 1.0f)
    {
        m_active = false;
        return true;
    }

    return false;
}

// ============================================================================
// LayeredPoseBlender
// ============================================================================

void LayeredPoseBlender::AddLayer(const BlendLayer& layer)
{
    m_layers.PushBack(layer);
    SortLayersByPriority();
}

void LayeredPoseBlender::RemoveLayer(const String& name)
{
    for (uint32_t i = 0; i < m_layers.Size(); ++i)
    {
        if (m_layers[i].name == name)
        {
            m_layers.Erase(i);
            break;
        }
    }
}

void LayeredPoseBlender::SetLayerWeight(const String& name, float weight)
{
    for (auto& layer : m_layers)
    {
        if (layer.name == name)
        {
            layer.weight = weight;
            break;
        }
    }
}

void LayeredPoseBlender::SetLayerActive(const String& name, bool active)
{
    for (auto& layer : m_layers)
    {
        if (layer.name == name)
        {
            layer.active = active;
            break;
        }
    }
}

BlendLayer* LayeredPoseBlender::GetLayer(const String& name)
{
    for (auto& layer : m_layers)
    {
        if (layer.name == name)
            return &layer;
    }
    return nullptr;
}

const BlendLayer* LayeredPoseBlender::GetLayer(const String& name) const
{
    for (const auto& layer : m_layers)
    {
        if (layer.name == name)
            return &layer;
    }
    return nullptr;
}

void LayeredPoseBlender::ComputeFinalPose(Pose& outPose) const
{
    if (m_layers.Empty())
        return;

    // Start with highest priority layer
    outPose = *m_layers[0].pose;

    // Blend in remaining layers
    for (uint32_t i = 1; i < m_layers.Size(); ++i)
    {
        const BlendLayer& layer = m_layers[i];
        if (!layer.active || !layer.pose || layer.weight <= 0.0f)
            continue;

        if (layer.boneMask.Empty())
        {
            PoseBlender::BlendPoses(outPose, outPose, *layer.pose, layer.weight, layer.blendType);
        }
        else
        {
            PoseBlender::BlendWithMask(outPose, outPose, *layer.pose, layer.weight, layer.boneMask);
        }
    }
}

void LayeredPoseBlender::ComputeFinalPoseWithBase(Pose& outPose, const Pose& basePose) const
{
    outPose = basePose;

    for (const auto& layer : m_layers)
    {
        if (!layer.active || !layer.pose || layer.weight <= 0.0f)
            continue;

        if (layer.boneMask.Empty())
        {
            PoseBlender::BlendPoses(outPose, outPose, *layer.pose, layer.weight, layer.blendType);
        }
        else
        {
            PoseBlender::BlendWithMask(outPose, outPose, *layer.pose, layer.weight, layer.boneMask);
        }
    }
}

void LayeredPoseBlender::ClearLayers()
{
    m_layers.Clear();
}

void LayeredPoseBlender::SortLayersByPriority()
{
    std::sort(m_layers.Begin(), m_layers.End(), [](const BlendLayer& a, const BlendLayer& b)
    {
        return a.priority > b.priority;
    });
}

// ============================================================================
// InertialBlender
// ============================================================================

void InertialBlender::Initialize(uint32_t boneCount)
{
    m_state.bonePositions.Resize(boneCount);
    m_state.boneVelocities.Resize(boneCount, Vec3(0, 0, 0));
    m_state.boneRotations.Resize(boneCount);
    m_state.boneAngularVelocities.Resize(boneCount, Vec3(0, 0, 0));
    m_state.boneScales.Resize(boneCount);
    m_state.boneScaleVelocities.Resize(boneCount, Vec3(0, 0, 0));
    m_state.active = false;
}

void InertialBlender::Start(const Pose& fromPose, const Pose& toPose, float halfLife)
{
    uint32_t boneCount = fromPose.GetBoneCount();
    Initialize(boneCount);

    m_targetPose = toPose;
    m_state.halfLife = halfLife;
    m_state.elapsedTime = 0.0f;
    m_state.active = true;

    // Capture initial state
    for (uint32_t i = 0; i < boneCount; ++i)
    {
        m_state.bonePositions[i] = fromPose.GetBoneTransform(i).position;
        m_state.boneRotations[i] = fromPose.GetBoneTransform(i).rotation;
        m_state.boneScales[i] = fromPose.GetBoneTransform(i).scale;
    }
}

void InertialBlender::Update(float deltaTime, Pose& outPose)
{
    if (!m_state.active)
    {
        outPose = m_targetPose;
        return;
    }

    m_state.elapsedTime += deltaTime;

    float damping = DampingFactor(m_state.halfLife, deltaTime);
    float springDamp = SpringDamping(m_state.halfLife);

    uint32_t boneCount = m_state.bonePositions.Size();
    outPose.Resize(boneCount);

    for (uint32_t i = 0; i < boneCount; ++i)
    {
        const Transform& target = m_targetPose.GetBoneTransform(i);

        // Position spring
        Vec3 positionError = target.position - m_state.bonePositions[i];
        m_state.boneVelocities[i] = m_state.boneVelocities[i] * damping + positionError * springDamp * deltaTime;
        m_state.bonePositions[i] += m_state.boneVelocities[i] * deltaTime;
        outPose.GetBoneTransform(i).position = m_state.bonePositions[i];

        // Rotation spring (simplified)
        outPose.GetBoneTransform(i).rotation = Quat::Slerp(m_state.boneRotations[i], target.rotation, 1.0f - damping);
        m_state.boneRotations[i] = outPose.GetBoneTransform(i).rotation;

        // Scale
        outPose.GetBoneTransform(i).scale = Vec3::Lerp(m_state.boneScales[i], target.scale, 1.0f - damping);
        m_state.boneScales[i] = outPose.GetBoneTransform(i).scale;
    }

    // Check if blend is essentially complete
    if (damping < 0.001f)
    {
        m_state.active = false;
        outPose = m_targetPose;
    }
}

float InertialBlender::DampingFactor(float halfLife, float deltaTime)
{
    return std::exp(-0.693147f * deltaTime / (halfLife + 1e-6f));
}

float InertialBlender::SpringDamping(float halfLife)
{
    return 5.0f / (halfLife + 1e-6f);
}

MMV2_NAMESPACE_END
