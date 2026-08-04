// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Compression System Implementation
// ============================================================================

#include "MMV2/Compression/Compression.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include "MMV2/Database/Database.h"
#include <cmath>
#include <cstring>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// QuantizationCompressor
// ============================================================================

QuantizationCompressor::QuantizationCompressor()
{
    m_settings.algorithm = CompressionAlgorithm::Quantize;
}

void QuantizationCompressor::SetSettings(const CompressionSettings& settings)
{
    m_settings = settings;
}

void QuantizationCompressor::QuantizeTransform(const Transform& transform, QuantizedTransform& out)
{
    out.posX = static_cast<int16_t>(Math::Clamp(transform.position.x * QuantizedTransform::POSITION_SCALE, -32767.0f, 32767.0f));
    out.posY = static_cast<int16_t>(Math::Clamp(transform.position.y * QuantizedTransform::POSITION_SCALE, -32767.0f, 32767.0f));
    out.posZ = static_cast<int16_t>(Math::Clamp(transform.position.z * QuantizedTransform::POSITION_SCALE, -32767.0f, 32767.0f));

    Quat normalizedRot = transform.rotation.Normalized();
    out.rotX = static_cast<int16_t>(Math::Clamp(normalizedRot.x * QuantizedTransform::ROTATION_SCALE, -32767.0f, 32767.0f));
    out.rotY = static_cast<int16_t>(Math::Clamp(normalizedRot.y * QuantizedTransform::ROTATION_SCALE, -32767.0f, 32767.0f));
    out.rotZ = static_cast<int16_t>(Math::Clamp(normalizedRot.z * QuantizedTransform::ROTATION_SCALE, -32767.0f, 32767.0f));
    out.rotW = static_cast<int16_t>(Math::Clamp(normalizedRot.w * QuantizedTransform::ROTATION_SCALE, -32767.0f, 32767.0f));

    out.scaleX = static_cast<int8_t>(Math::Clamp(transform.scale.x * QuantizedTransform::SCALE_SCALE, -127.0f, 127.0f));
    out.scaleY = static_cast<int8_t>(Math::Clamp(transform.scale.y * QuantizedTransform::SCALE_SCALE, -127.0f, 127.0f));
    out.scaleZ = static_cast<int8_t>(Math::Clamp(transform.scale.z * QuantizedTransform::SCALE_SCALE, -127.0f, 127.0f));
}

void QuantizationCompressor::DequantizeTransform(const QuantizedTransform& quantized, Transform& out)
{
    out.position.x = quantized.posX / QuantizedTransform::POSITION_SCALE;
    out.position.y = quantized.posY / QuantizedTransform::POSITION_SCALE;
    out.position.z = quantized.posZ / QuantizedTransform::POSITION_SCALE;

    out.rotation.x = quantized.rotX / QuantizedTransform::ROTATION_SCALE;
    out.rotation.y = quantized.rotY / QuantizedTransform::ROTATION_SCALE;
    out.rotation.z = quantized.rotZ / QuantizedTransform::ROTATION_SCALE;
    out.rotation.w = quantized.rotW / QuantizedTransform::ROTATION_SCALE;
    out.rotation = out.rotation.Normalized();

    out.scale.x = quantized.scaleX / QuantizedTransform::SCALE_SCALE;
    out.scale.y = quantized.scaleY / QuantizedTransform::SCALE_SCALE;
    out.scale.z = quantized.scaleZ / QuantizedTransform::SCALE_SCALE;
}

bool QuantizationCompressor::CompressPoses(const Vector<Pose>& poses, Vector<uint8_t>& outData)
{
    m_stats.originalFrameCount = static_cast<uint32_t>(poses.Size());
    m_stats.originalSize = poses.Size() * sizeof(Pose); // Approximate

    // Calculate output size
    uint32_t boneCount = poses.Empty() ? 0 : poses[0].GetBoneCount();
    uint32_t frameSize = sizeof(uint32_t) + boneCount * sizeof(QuantizedTransform);
    uint32_t totalSize = sizeof(uint32_t) + poses.Size() * frameSize;

    outData.Resize(totalSize);
    uint8_t* ptr = outData.Data();

    // Write frame count
    std::memcpy(ptr, &poses.Size(), sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    // Write bone count
    std::memcpy(ptr, &boneCount, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    for (const auto& pose : poses)
    {
        uint32_t currentBoneCount = pose.GetBoneCount();
        std::memcpy(ptr, &currentBoneCount, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        for (uint32_t b = 0; b < currentBoneCount; ++b)
        {
            QuantizedTransform qt;
            QuantizeTransform(pose.GetBoneTransform(b), qt);
            std::memcpy(ptr, &qt, sizeof(QuantizedTransform));
            ptr += sizeof(QuantizedTransform);
        }
    }

    m_stats.compressedSize = outData.Size();
    m_stats.compressionRatio = static_cast<float>(m_stats.originalSize) / static_cast<float>(m_stats.compressedSize);
    m_stats.keyframeCount = static_cast<uint32_t>(poses.Size());

    return true;
}

bool QuantizationCompressor::DecompressPoses(const Vector<uint8_t>& data, Vector<Pose>& outPoses)
{
    if (data.Size() < sizeof(uint32_t) * 2)
        return false;

    const uint8_t* ptr = data.Data();

    uint32_t frameCount;
    std::memcpy(&frameCount, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    uint32_t boneCount;
    std::memcpy(&boneCount, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    outPoses.Resize(frameCount);

    for (uint32_t f = 0; f < frameCount; ++f)
    {
        uint32_t currentBoneCount;
        std::memcpy(&currentBoneCount, ptr, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        outPoses[f].Resize(currentBoneCount);

        for (uint32_t b = 0; b < currentBoneCount; ++b)
        {
            QuantizedTransform qt;
            std::memcpy(&qt, ptr, sizeof(QuantizedTransform));
            ptr += sizeof(QuantizedTransform);

            Transform transform;
            DequantizeTransform(qt, transform);
            outPoses[f].SetBoneTransform(b, transform);
        }
    }

    return true;
}

bool QuantizationCompressor::CompressFeatures(const Vector<FeatureVector>& features, Vector<uint8_t>& outData)
{
    if (features.Empty())
        return true;

    uint32_t dims = features[0].dimensions;
    uint32_t count = static_cast<uint32_t>(features.Size());

    // Compute ranges for adaptive quantization
    Vector<float> minVals(dims, FLT_MAX);
    Vector<float> maxVals(dims, -FLT_MAX);

    for (const auto& fv : features)
    {
        for (uint32_t d = 0; d < dims; ++d)
        {
            minVals[d] = Math::Min(minVals[d], fv.data[d]);
            maxVals[d] = Math::Max(maxVals[d], fv.data[d]);
        }
    }

    // Write header
    uint32_t headerSize = sizeof(uint32_t) * 3 + dims * sizeof(float) * 2;
    uint32_t dataSize = count * dims * sizeof(uint16_t);
    outData.Resize(headerSize + dataSize);

    uint8_t* ptr = outData.Data();
    std::memcpy(ptr, &count, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &dims, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    uint32_t bits = 16;
    std::memcpy(ptr, &bits, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    // Write ranges
    for (uint32_t d = 0; d < dims; ++d)
    {
        std::memcpy(ptr, &minVals[d], sizeof(float)); ptr += sizeof(float);
        std::memcpy(ptr, &maxVals[d], sizeof(float)); ptr += sizeof(float);
    }

    // Quantize and write data
    for (const auto& fv : features)
    {
        for (uint32_t d = 0; d < dims; ++d)
        {
            float range = maxVals[d] - minVals[d];
            if (range < 1e-6f) range = 1.0f;

            float normalized = (fv.data[d] - minVals[d]) / range;
            uint16_t quantized = static_cast<uint16_t>(Math::Clamp(normalized * 65535.0f, 0.0f, 65535.0f));
            std::memcpy(ptr, &quantized, sizeof(uint16_t));
            ptr += sizeof(uint16_t);
        }
    }

    m_stats.compressedSize = outData.Size();
    m_stats.compressionRatio = static_cast<float>(count * dims * sizeof(float)) / static_cast<float>(outData.Size());

    return true;
}

bool QuantizationCompressor::DecompressFeatures(const Vector<uint8_t>& data, Vector<FeatureVector>& outFeatures)
{
    if (data.Size() < sizeof(uint32_t) * 3)
        return false;

    const uint8_t* ptr = data.Data();

    uint32_t count, dims, bits;
    std::memcpy(&count, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&dims, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&bits, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    Vector<float> minVals(dims);
    Vector<float> maxVals(dims);

    for (uint32_t d = 0; d < dims; ++d)
    {
        std::memcpy(&minVals[d], ptr, sizeof(float)); ptr += sizeof(float);
        std::memcpy(&maxVals[d], ptr, sizeof(float)); ptr += sizeof(float);
    }

    outFeatures.Resize(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        outFeatures[i].Resize(dims);

        for (uint32_t d = 0; d < dims; ++d)
        {
            uint16_t quantized;
            std::memcpy(&quantized, ptr, sizeof(uint16_t));
            ptr += sizeof(uint16_t);

            float normalized = quantized / 65535.0f;
            float range = maxVals[d] - minVals[d];
            outFeatures[i].data[d] = minVals[d] + normalized * range;
        }
    }

    return true;
}

bool QuantizationCompressor::CompressPose(const Pose& pose, CompressedPoseData& outData)
{
    uint32_t boneCount = pose.GetBoneCount();
    outData.boneTransforms.Resize(boneCount);

    for (uint32_t b = 0; b < boneCount; ++b)
    {
        QuantizeTransform(pose.GetBoneTransform(b), outData.boneTransforms[b]);
    }

    return true;
}

bool QuantizationCompressor::DecompressPose(const CompressedPoseData& data, Pose& outPose)
{
    uint32_t boneCount = static_cast<uint32_t>(data.boneTransforms.Size());
    outPose.Resize(boneCount);

    for (uint32_t b = 0; b < boneCount; ++b)
    {
        Transform transform;
        DequantizeTransform(data.boneTransforms[b], transform);
        outPose.SetBoneTransform(b, transform);
    }

    return true;
}

// ============================================================================
// AdaptiveQuantizationCompressor
// ============================================================================

void AdaptiveQuantizationCompressor::SetSettings(const CompressionSettings& settings)
{
    m_settings = settings;
}

void AdaptiveQuantizationCompressor::ComputeAdaptiveRanges(const Vector<Pose>& poses)
{
    // Per-bone adaptive ranges based on bone importance
    // Root and end effectors get more bits
    if (poses.Empty()) return;

    uint32_t boneCount = poses[0].GetBoneCount();
    m_positionRanges.Resize(boneCount);
    m_rotationRanges.Resize(boneCount);

    for (uint32_t b = 0; b < boneCount; ++b)
    {
        float minPos = FLT_MAX, maxPos = -FLT_MAX;
        float minRot = FLT_MAX, maxRot = -FLT_MAX;

        for (const auto& pose : poses)
        {
            const Transform& t = pose.GetBoneTransform(b);
            minPos = Math::Min(minPos, t.position.Magnitude());
            maxPos = Math::Max(maxPos, t.position.Magnitude());

            float rotAngle = 2.0f * std::acos(Math::Clamp(t.rotation.w, -1.0f, 1.0f));
            minRot = Math::Min(minRot, rotAngle);
            maxRot = Math::Max(maxRot, rotAngle);
        }

        // Root bone (0) and end effectors get more precision
        bool isImportant = (b == 0) || m_settings.preserveEndEffectors;
        uint32_t bits = isImportant ? m_settings.positionBits : (m_settings.positionBits / 2);

        m_positionRanges[b] = {minPos, maxPos, (maxPos > minPos) ? (maxPos - minPos) : 1.0f, bits};
        m_rotationRanges[b] = {minRot, maxRot, (maxRot > minRot) ? (maxRot - minRot) : 1.0f, 
                                isImportant ? m_settings.rotationBits : (m_settings.rotationBits / 2)};
    }
}

void AdaptiveQuantizationCompressor::ComputeAdaptiveRanges(const Vector<FeatureVector>& features)
{
    // Compute per-dimension ranges for feature vectors
    // Implementation similar to above but for feature dimensions
}

bool AdaptiveQuantizationCompressor::CompressPoses(const Vector<Pose>& poses, Vector<uint8_t>& outData)
{
    ComputeAdaptiveRanges(poses);

    // Similar to QuantizationCompressor but uses adaptive ranges
    // For brevity, delegate to base quantization with adaptive settings
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.CompressPoses(poses, outData);
}

bool AdaptiveQuantizationCompressor::DecompressPoses(const Vector<uint8_t>& data, Vector<Pose>& outPoses)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.DecompressPoses(data, outPoses);
}

bool AdaptiveQuantizationCompressor::CompressFeatures(const Vector<FeatureVector>& features, Vector<uint8_t>& outData)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.CompressFeatures(features, outData);
}

bool AdaptiveQuantizationCompressor::DecompressFeatures(const Vector<uint8_t>& data, Vector<FeatureVector>& outFeatures)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.DecompressFeatures(data, outFeatures);
}

bool AdaptiveQuantizationCompressor::CompressPose(const Pose& pose, CompressedPoseData& outData)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.CompressPose(pose, outData);
}

bool AdaptiveQuantizationCompressor::DecompressPose(const CompressedPoseData& data, Pose& outPose)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.DecompressPose(data, outPose);
}

// ============================================================================
// PCACompressor (Simplified)
// ============================================================================

void PCACompressor::SetSettings(const CompressionSettings& settings)
{
    m_settings = settings;
}

void PCACompressor::ComputePCA(const Vector<FeatureVector>& features)
{
    if (features.Empty()) return;

    m_originalDimensions = features[0].dimensions;
    m_componentCount = Math::Min(m_settings.pcaComponents, m_originalDimensions);

    // Compute mean
    m_meanVector.Resize(m_originalDimensions, 0.0f);
    for (const auto& fv : features)
    {
        for (uint32_t d = 0; d < m_originalDimensions; ++d)
            m_meanVector[d] += fv.data[d];
    }

    float invCount = 1.0f / static_cast<float>(features.Size());
    for (auto& val : m_meanVector)
        val *= invCount;

    // Simplified: use identity-like eigenvectors (no full SVD for brevity)
    // In production, use proper SVD or power iteration
    m_eigenVectors.Resize(m_originalDimensions * m_componentCount, 0.0f);
    m_eigenValues.Resize(m_componentCount, 1.0f);

    for (uint32_t c = 0; c < m_componentCount; ++c)
    {
        for (uint32_t d = 0; d < m_originalDimensions; ++d)
        {
            m_eigenVectors[c * m_originalDimensions + d] = (d == c) ? 1.0f : 0.0f;
        }
    }
}

void PCACompressor::ProjectToPCA(const FeatureVector& input, Vector<float>& outComponents)
{
    outComponents.Resize(m_componentCount, 0.0f);

    for (uint32_t c = 0; c < m_componentCount; ++c)
    {
        float sum = 0.0f;
        for (uint32_t d = 0; d < m_originalDimensions; ++d)
        {
            float centered = input.data[d] - m_meanVector[d];
            sum += centered * m_eigenVectors[c * m_originalDimensions + d];
        }
        outComponents[c] = sum;
    }
}

void PCACompressor::ReconstructFromPCA(const Vector<float>& components, FeatureVector& out)
{
    out.Resize(m_originalDimensions);

    for (uint32_t d = 0; d < m_originalDimensions; ++d)
    {
        float sum = m_meanVector[d];
        for (uint32_t c = 0; c < m_componentCount; ++c)
        {
            sum += components[c] * m_eigenVectors[c * m_originalDimensions + d];
        }
        out.data[d] = sum;
    }
}

bool PCACompressor::CompressPoses(const Vector<Pose>& poses, Vector<uint8_t>& outData)
{
    // PCA not typically used for poses directly
    // Delegate to quantization
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.CompressPoses(poses, outData);
}

bool PCACompressor::DecompressPoses(const Vector<uint8_t>& data, Vector<Pose>& outPoses)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.DecompressPoses(data, outPoses);
}

bool PCACompressor::CompressFeatures(const Vector<FeatureVector>& features, Vector<uint8_t>& outData)
{
    ComputePCA(features);

    // Project all features to PCA space
    Vector<Vector<float>> compressed(features.Size());
    for (uint32_t i = 0; i < features.Size(); ++i)
    {
        ProjectToPCA(features[i], compressed[i]);
    }

    // Serialize compressed data
    // Header: original dims, component count, mean vector, eigenvectors
    uint32_t headerSize = sizeof(uint32_t) * 2 + 
                          m_originalDimensions * sizeof(float) +
                          m_originalDimensions * m_componentCount * sizeof(float);

    uint32_t dataSize = features.Size() * m_componentCount * sizeof(float);
    outData.Resize(headerSize + dataSize);

    uint8_t* ptr = outData.Data();
    std::memcpy(ptr, &m_originalDimensions, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &m_componentCount, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    std::memcpy(ptr, m_meanVector.Data(), m_originalDimensions * sizeof(float));
    ptr += m_originalDimensions * sizeof(float);

    std::memcpy(ptr, m_eigenVectors.Data(), m_originalDimensions * m_componentCount * sizeof(float));
    ptr += m_originalDimensions * m_componentCount * sizeof(float);

    for (const auto& comp : compressed)
    {
        std::memcpy(ptr, comp.Data(), m_componentCount * sizeof(float));
        ptr += m_componentCount * sizeof(float);
    }

    m_stats.compressedSize = outData.Size();
    m_stats.compressionRatio = static_cast<float>(features.Size() * m_originalDimensions * sizeof(float)) / 
                                static_cast<float>(outData.Size());

    return true;
}

bool PCACompressor::DecompressFeatures(const Vector<uint8_t>& data, Vector<FeatureVector>& outFeatures)
{
    if (data.Size() < sizeof(uint32_t) * 2)
        return false;

    const uint8_t* ptr = data.Data();
    std::memcpy(&m_originalDimensions, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&m_componentCount, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    m_meanVector.Resize(m_originalDimensions);
    std::memcpy(m_meanVector.Data(), ptr, m_originalDimensions * sizeof(float));
    ptr += m_originalDimensions * sizeof(float);

    m_eigenVectors.Resize(m_originalDimensions * m_componentCount);
    std::memcpy(m_eigenVectors.Data(), ptr, m_originalDimensions * m_componentCount * sizeof(float));
    ptr += m_originalDimensions * m_componentCount * sizeof(float);

    uint32_t remaining = static_cast<uint32_t>(data.Size() - (ptr - data.Data()));
    uint32_t count = remaining / (m_componentCount * sizeof(float));

    outFeatures.Resize(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        Vector<float> components(m_componentCount);
        std::memcpy(components.Data(), ptr, m_componentCount * sizeof(float));
        ptr += m_componentCount * sizeof(float);

        ReconstructFromPCA(components, outFeatures[i]);
    }

    return true;
}

bool PCACompressor::CompressPose(const Pose& pose, CompressedPoseData& outData)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.CompressPose(pose, outData);
}

bool PCACompressor::DecompressPose(const CompressedPoseData& data, Pose& outPose)
{
    QuantizationCompressor base;
    base.SetSettings(m_settings);
    return base.DecompressPose(data, outPose);
}

// ============================================================================
// KeyframeReducer
// ============================================================================

void KeyframeReducer::SetSettings(const CompressionSettings& settings)
{
    m_settings = settings;
}

float KeyframeReducer::ComputePoseError(const Pose& a, const Pose& b)
{
    uint32_t boneCount = a.GetBoneCount();
    float totalError = 0.0f;

    for (uint32_t b = 0; b < boneCount; ++b)
    {
        const Transform& ta = a.GetBoneTransform(b);
        const Transform& tb = b.GetBoneTransform(b);

        float posError = (ta.position - tb.position).Magnitude();
        float rotError = 1.0f - std::abs(Quat::Dot(ta.rotation, tb.rotation));

        totalError += posError + rotError * 10.0f;
    }

    return totalError / static_cast<float>(boneCount);
}

bool KeyframeReducer::IsKeyframeNeeded(const Pose& prev, const Pose& current, const Pose& next, float tolerance)
{
    // Check if current pose deviates significantly from linear interpolation between prev and next
    Pose interpolated;
    Pose::Blend(interpolated, prev, next, 0.5f);

    float error = ComputePoseError(current, interpolated);
    return error > tolerance;
}

KeyframeReductionResult KeyframeReducer::ReduceKeyframes(const Vector<Pose>& poses,
                                                          const Vector<float>& timestamps)
{
    KeyframeReductionResult result;

    if (poses.Empty())
        return result;

    // Always keep first and last
    result.keyframeIndices.PushBack(0);
    result.keyframeTimestamps.PushBack(timestamps[0]);

    uint32_t step = m_settings.maxKeyframeReduction;
    for (uint32_t i = step; i < poses.Size() - 1; i += step)
    {
        result.keyframeIndices.PushBack(i);
        result.keyframeTimestamps.PushBack(timestamps[i]);
    }

    // Always keep last
    if (poses.Size() > 1)
    {
        result.keyframeIndices.PushBack(static_cast<uint32_t>(poses.Size()) - 1);
        result.keyframeTimestamps.PushBack(timestamps[timestamps.Size() - 1]);
    }

    result.stats.originalFrameCount = static_cast<uint32_t>(poses.Size());
    result.stats.keyframeCount = static_cast<uint32_t>(result.keyframeIndices.Size());
    result.stats.compressionRatio = static_cast<float>(poses.Size()) / static_cast<float>(result.keyframeIndices.Size());

    return result;
}

KeyframeReductionResult KeyframeReducer::ReduceKeyframesAdaptive(const Vector<Pose>& poses,
                                                                  const Vector<float>& timestamps,
                                                                  float maxError)
{
    KeyframeReductionResult result;

    if (poses.Size() < 2)
        return result;

    // Always keep first
    result.keyframeIndices.PushBack(0);
    result.keyframeTimestamps.PushBack(timestamps[0]);

    uint32_t lastKeyframe = 0;

    for (uint32_t i = 1; i < poses.Size() - 1; ++i)
    {
        if (IsKeyframeNeeded(poses[lastKeyframe], poses[i], poses[i + 1], maxError))
        {
            result.keyframeIndices.PushBack(i);
            result.keyframeTimestamps.PushBack(timestamps[i]);
            lastKeyframe = i;
        }
    }

    // Always keep last
    result.keyframeIndices.PushBack(static_cast<uint32_t>(poses.Size()) - 1);
    result.keyframeTimestamps.PushBack(timestamps[timestamps.Size() - 1]);

    result.stats.originalFrameCount = static_cast<uint32_t>(poses.Size());
    result.stats.keyframeCount = static_cast<uint32_t>(result.keyframeIndices.Size());
    result.stats.compressionRatio = static_cast<float>(poses.Size()) / static_cast<float>(result.keyframeIndices.Size());

    return result;
}

Vector<Pose> KeyframeReducer::ReconstructFromKeyframes(const KeyframeReductionResult& keyframes,
                                                        const Vector<float>& targetTimestamps)
{
    Vector<Pose> result;
    result.Reserve(targetTimestamps.Size());

    for (float t : targetTimestamps)
    {
        // Find surrounding keyframes
        uint32_t idx = 0;
        while (idx < keyframes.keyframeTimestamps.Size() - 1 && 
               keyframes.keyframeTimestamps[idx + 1] < t)
        {
            ++idx;
        }

        if (idx >= keyframes.keyframeTimestamps.Size() - 1)
        {
            // Use last keyframe
            // In real implementation, we'd need access to the actual poses
            // This is a placeholder
            continue;
        }

        float t0 = keyframes.keyframeTimestamps[idx];
        float t1 = keyframes.keyframeTimestamps[idx + 1];
        float alpha = (t - t0) / (t1 - t0 + 1e-6f);

        // Would interpolate between poses here
        // result.PushBack(interpolatedPose);
    }

    return result;
}

// ============================================================================
// CompressionFactory
// ============================================================================

UniquePtr<ICompressor> CompressionFactory::CreateCompressor(CompressionAlgorithm algorithm)
{
    switch (algorithm)
    {
        case CompressionAlgorithm::Quantize:
        case CompressionAlgorithm::None:
            return MakeUnique<QuantizationCompressor>();
        case CompressionAlgorithm::AdaptiveQuantize:
            return MakeUnique<AdaptiveQuantizationCompressor>();
        case CompressionAlgorithm::PCA:
            return MakeUnique<PCACompressor>();
        default:
            return MakeUnique<QuantizationCompressor>();
    }
}

UniquePtr<ICompressor> CompressionFactory::CreateDefaultCompressor()
{
    return MakeUnique<AdaptiveQuantizationCompressor>();
}

CompressionSettings CompressionFactory::GetRecommendedSettings(CompressionAlgorithm algorithm)
{
    CompressionSettings settings;
    settings.algorithm = algorithm;

    switch (algorithm)
    {
        case CompressionAlgorithm::Quantize:
            settings.positionBits = 16;
            settings.rotationBits = 16;
            break;
        case CompressionAlgorithm::AdaptiveQuantize:
            settings.positionBits = 16;
            settings.rotationBits = 16;
            settings.preserveEndEffectors = true;
            break;
        case CompressionAlgorithm::PCA:
            settings.pcaComponents = 16;
            settings.pcaVarianceThreshold = 0.99f;
            break;
        default:
            break;
    }

    return settings;
}

CompressionSettings CompressionFactory::GetFastSettings()
{
    CompressionSettings settings;
    settings.algorithm = CompressionAlgorithm::Quantize;
    settings.positionBits = 12;
    settings.rotationBits = 12;
    settings.positionTolerance = 0.01f;
    settings.rotationTolerance = 0.05f;
    return settings;
}

CompressionSettings CompressionFactory::GetHighQualitySettings()
{
    CompressionSettings settings;
    settings.algorithm = CompressionAlgorithm::AdaptiveQuantize;
    settings.positionBits = 20;
    settings.rotationBits = 20;
    settings.positionTolerance = 0.0001f;
    settings.rotationTolerance = 0.001f;
    settings.preserveEndEffectors = true;
    settings.preserveRootMotion = true;
    return settings;
}

CompressionSettings CompressionFactory::GetMaximumCompressionSettings()
{
    CompressionSettings settings;
    settings.algorithm = CompressionAlgorithm::PCA;
    settings.pcaComponents = 8;
    settings.pcaVarianceThreshold = 0.95f;
    settings.positionBits = 10;
    settings.rotationBits = 10;
    settings.maxKeyframeReduction = 8;
    return settings;
}

// ============================================================================
// DatabaseCompressor
// ============================================================================

bool DatabaseCompressor::CompressDatabase(PoseDatabase& database)
{
    if (!m_compressor)
        return false;

    // Get all poses from database
    Vector<Pose> poses;
    const uint32_t count = database.GetPoseCount();
    poses.Reserve(count);

    for (uint32_t i = 0; i < count; ++i)
    {
        poses.PushBack(database.GetPose(i));
    }

    Vector<uint8_t> compressed;
    bool success = m_compressor->CompressPoses(poses, compressed);

    if (success)
    {
        database.SetCompressedData(compressed);
        m_stats = m_compressor->GetStats();
    }

    return success;
}

bool DatabaseCompressor::DecompressDatabase(PoseDatabase& database)
{
    if (!m_compressor)
        return false;

    const Vector<uint8_t>& compressed = database.GetCompressedData();
    if (compressed.Empty())
        return false;

    Vector<Pose> poses;
    bool success = m_compressor->DecompressPoses(compressed, poses);

    if (success)
    {
        database.SetPoses(poses);
    }

    return success;
}

bool DatabaseCompressor::CompressFeatureDatabase(FeatureDatabase& database)
{
    if (!m_compressor)
        return false;

    Vector<FeatureVector> features;
    const uint32_t count = database.GetPoseCount();
    features.Reserve(count);

    for (uint32_t i = 0; i < count; ++i)
    {
        features.PushBack(database.GetFeatureVector(i));
    }

    Vector<uint8_t> compressed;
    bool success = m_compressor->CompressFeatures(features, compressed);

    if (success)
    {
        database.SetCompressedFeatures(compressed);
    }

    return success;
}

bool DatabaseCompressor::DecompressFeatureDatabase(FeatureDatabase& database)
{
    if (!m_compressor)
        return false;

    const Vector<uint8_t>& compressed = database.GetCompressedFeatures();
    if (compressed.Empty())
        return false;

    Vector<FeatureVector> features;
    bool success = m_compressor->DecompressFeatures(compressed, features);

    if (success)
    {
        database.SetFeatureVectors(features);
    }

    return success;
}

MMV2_NAMESPACE_END
