#include "MMV2/Features/Feature.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Animation/Trajectory.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

int32 FeatureSchema::AddChannel(const FeatureChannelDesc& desc) {
    channels.PushBack(desc);
    RebuildOffsets();
    return static_cast<int32>(channels.Size()) - 1;
}

int32 FeatureSchema::FindChannel(const char* name) const {
    for (size_type i = 0; i < channels.Size(); ++i) {
        if (channels[i].name == name) return static_cast<int32>(i);
    }
    return -1;
}

int32 FeatureSchema::FindChannel(FeatureChannelType type, int32 boneIndex) const {
    for (size_type i = 0; i < channels.Size(); ++i) {
        if (channels[i].type == type && (boneIndex < 0 || channels[i].boneIndex == boneIndex)) {
            return static_cast<int32>(i);
        }
    }
    return -1;
}

void FeatureSchema::RemoveChannel(int32 index) {
    if (index < 0 || index >= static_cast<int32>(channels.Size())) return;
    channels.Erase(channels.begin() + index);
    RebuildOffsets();
}

void FeatureSchema::SetChannelWeight(int32 index, float32 weight) {
    if (index >= 0 && index < static_cast<int32>(channels.Size())) {
        channels[index].weight = weight;
    }
}

void FeatureSchema::SetChannelEnabled(int32 index, bool enabled) {
    if (index >= 0 && index < static_cast<int32>(channels.Size())) {
        channels[index].enabled = enabled;
        RebuildOffsets();
    }
}

void FeatureSchema::RebuildOffsets() {
    totalDimension = 0;
    enabledDimension = 0;
    for (size_type i = 0; i < channels.Size(); ++i) {
        channels[i].offset = totalDimension;
        totalDimension += channels[i].dimension;
        if (channels[i].enabled) enabledDimension += channels[i].dimension;
    }
}

bool FeatureSchema::IsValid() const {
    if (channels.IsEmpty()) return false;
    for (const auto& ch : channels) {
        if (ch.type == FeatureChannelType::Invalid) return false;
        if (ch.dimension <= 0) return false;
    }
    return true;
}

FeatureSchema FeatureSchema::Clone() const {
    FeatureSchema copy;
    copy.channels = channels;
    copy.totalDimension = totalDimension;
    copy.enabledDimension = enabledDimension;
    copy.name = name;
    return copy;
}

// FeatureVector
void FeatureVector::Resize(int32 newDim) {
    dimension = newDim;
    data.Resize(newDim, 0.0f);
}

void FeatureVector::SetZero() {
    for (int32 i = 0; i < dimension; ++i) data[i] = 0.0f;
}

void FeatureVector::SetFrom(const float32* src, int32 count) {
    int32 copyCount = std::min(count, dimension);
    for (int32 i = 0; i < copyCount; ++i) data[i] = src[i];
}

void FeatureVector::CopyFrom(const FeatureVector& other) {
    Resize(other.dimension);
    for (int32 i = 0; i < dimension; ++i) data[i] = other.data[i];
}

void FeatureVector::Normalize(const FeatureSchema& schema) {
    int32 offset = 0;
    for (const auto& ch : schema.channels) {
        if (!ch.enabled) continue;
        float32 range = ch.maxValue - ch.minValue;
        if (range > MMV2_EPSILON) {
            for (int32 i = 0; i < ch.dimension; ++i) {
                data[offset + i] = (data[offset + i] - ch.minValue) / range;
            }
        }
        offset += ch.dimension;
    }
}

void FeatureVector::Denormalize(const FeatureSchema& schema) {
    int32 offset = 0;
    for (const auto& ch : schema.channels) {
        if (!ch.enabled) continue;
        float32 range = ch.maxValue - ch.minValue;
        for (int32 i = 0; i < ch.dimension; ++i) {
            data[offset + i] = data[offset + i] * range + ch.minValue;
        }
        offset += ch.dimension;
    }
}

float32 FeatureVector::DistanceTo(const FeatureVector& other, const FeatureSchema& schema) const {
    return std::sqrt(DistanceToSq(other, schema));
}

float32 FeatureVector::DistanceToSq(const FeatureVector& other, const FeatureSchema& schema) const {
    float32 distSq = 0.0f;
    int32 offset = 0;
    for (const auto& ch : schema.channels) {
        if (!ch.enabled) continue;
        for (int32 i = 0; i < ch.dimension; ++i) {
            float32 diff = data[offset + i] - other.data[offset + i];
            distSq += diff * diff;
        }
        offset += ch.dimension;
    }
    return distSq;
}

float32 FeatureVector::DistanceToWeighted(const FeatureVector& other, const FeatureSchema& schema) const {
    float32 distSq = 0.0f;
    int32 offset = 0;
    for (const auto& ch : schema.channels) {
        if (!ch.enabled) continue;
        float32 channelDistSq = 0.0f;
        for (int32 i = 0; i < ch.dimension; ++i) {
            float32 diff = data[offset + i] - other.data[offset + i];
            channelDistSq += diff * diff;
        }
        distSq += channelDistSq * ch.weight * ch.weight;
        offset += ch.dimension;
    }
    return std::sqrt(distSq);
}

float32 FeatureVector::Dot(const FeatureVector& other) const {
    float32 result = 0.0f;
    int32 count = std::min(dimension, other.dimension);
    for (int32 i = 0; i < count; ++i) result += data[i] * other.data[i];
    return result;
}

float32 FeatureVector::Length() const {
    return std::sqrt(LengthSq());
}

float32 FeatureVector::LengthSq() const {
    float32 result = 0.0f;
    for (int32 i = 0; i < dimension; ++i) result += data[i] * data[i];
    return result;
}

void FeatureVector::Scale(float32 s) {
    for (int32 i = 0; i < dimension; ++i) data[i] *= s;
}

void FeatureVector::Add(const FeatureVector& other) {
    int32 count = std::min(dimension, other.dimension);
    for (int32 i = 0; i < count; ++i) data[i] += other.data[i];
}

void FeatureVector::Subtract(const FeatureVector& other) {
    int32 count = std::min(dimension, other.dimension);
    for (int32 i = 0; i < count; ++i) data[i] -= other.data[i];
}

void FeatureVector::Lerp(const FeatureVector& a, const FeatureVector& b, float32 t) {
    int32 count = std::min(a.dimension, b.dimension);
    Resize(count);
    for (int32 i = 0; i < count; ++i) data[i] = a.data[i] + (b.data[i] - a.data[i]) * t;
}

bool FeatureVector::IsValid() const {
    for (int32 i = 0; i < dimension; ++i) {
        if (std::isnan(data[i]) || std::isinf(data[i])) return false;
    }
    return dimension > 0;
}

bool FeatureVector::IsNearZero() const {
    return LengthSq() < MMV2_EPSILON_SQ;
}

// Extractors
void BonePositionExtractor::Extract(const Pose& pose, const PoseVelocity& velocity,
                                    const Trajectory& trajectory, float32* outData, int32 offset) const {
    if (m_boneIndex < 0 || m_boneIndex >= pose.boneCount) return;
    Vec3 pos = m_localSpace ? pose.localPositions[m_boneIndex] : pose.worldPositions[m_boneIndex];
    outData[offset + 0] = pos.x;
    outData[offset + 1] = pos.y;
    outData[offset + 2] = pos.z;
}

void BonePositionExtractor::ExtractNormalized(const Pose& pose, const PoseVelocity& velocity,
                                                const Trajectory& trajectory, float32* outData, int32 offset,
                                                const FeatureSchema& schema) const {
    Extract(pose, velocity, trajectory, outData, offset);
}

bool BonePositionExtractor::Validate(const FeatureSchema& schema, int32 channelIndex) const {
    return m_boneIndex >= 0;
}

void BoneRotationExtractor::Extract(const Pose& pose, const PoseVelocity& velocity,
                                    const Trajectory& trajectory, float32* outData, int32 offset) const {
    if (m_boneIndex < 0 || m_boneIndex >= pose.boneCount) return;
    Quat rot = m_localSpace ? pose.localRotations[m_boneIndex] : pose.worldRotations[m_boneIndex];
    outData[offset + 0] = rot.x;
    outData[offset + 1] = rot.y;
    outData[offset + 2] = rot.z;
    outData[offset + 3] = rot.w;
}

void BoneRotationExtractor::ExtractNormalized(const Pose& pose, const PoseVelocity& velocity,
                                              const Trajectory& trajectory, float32* outData, int32 offset,
                                              const FeatureSchema& schema) const {
    Extract(pose, velocity, trajectory, outData, offset);
}

bool BoneRotationExtractor::Validate(const FeatureSchema& schema, int32 channelIndex) const {
    return m_boneIndex >= 0;
}

void BoneVelocityExtractor::Extract(const Pose& pose, const PoseVelocity& velocity,
                                    const Trajectory& trajectory, float32* outData, int32 offset) const {
    if (m_boneIndex < 0 || m_boneIndex >= velocity.boneCount) return;
    Vec3 vel = velocity.linearVelocities[m_boneIndex];
    outData[offset + 0] = vel.x;
    outData[offset + 1] = vel.y;
    outData[offset + 2] = vel.z;
}

void BoneVelocityExtractor::ExtractNormalized(const Pose& pose, const PoseVelocity& velocity,
                                              const Trajectory& trajectory, float32* outData, int32 offset,
                                              const FeatureSchema& schema) const {
    Extract(pose, velocity, trajectory, outData, offset);
}

bool BoneVelocityExtractor::Validate(const FeatureSchema& schema, int32 channelIndex) const {
    return m_boneIndex >= 0;
}

void TrajectoryPositionExtractor::Extract(const Pose& pose, const PoseVelocity& velocity,
                                          const Trajectory& trajectory, float32* outData, int32 offset) const {
    if (m_trajectoryIndex < 0 || m_trajectoryIndex >= static_cast<int32>(trajectory.points.Size())) return;
    Vec3 pos = trajectory.points[m_trajectoryIndex].position;
    outData[offset + 0] = pos.x;
    outData[offset + 1] = pos.y;
    outData[offset + 2] = pos.z;
}

void TrajectoryPositionExtractor::ExtractNormalized(const Pose& pose, const PoseVelocity& velocity,
                                                    const Trajectory& trajectory, float32* outData, int32 offset,
                                                    const FeatureSchema& schema) const {
    Extract(pose, velocity, trajectory, outData, offset);
}

bool TrajectoryPositionExtractor::Validate(const FeatureSchema& schema, int32 channelIndex) const {
    return m_trajectoryIndex >= 0;
}

void TrajectoryVelocityExtractor::Extract(const Pose& pose, const PoseVelocity& velocity,
                                          const Trajectory& trajectory, float32* outData, int32 offset) const {
    if (m_trajectoryIndex < 0 || m_trajectoryIndex >= static_cast<int32>(trajectory.points.Size())) return;
    Vec3 vel = trajectory.points[m_trajectoryIndex].velocity;
    outData[offset + 0] = vel.x;
    outData[offset + 1] = vel.y;
    outData[offset + 2] = vel.z;
}

void TrajectoryVelocityExtractor::ExtractNormalized(const Pose& pose, const PoseVelocity& velocity,
                                                  const Trajectory& trajectory, float32* outData, int32 offset,
                                                  const FeatureSchema& schema) const {
    Extract(pose, velocity, trajectory, outData, offset);
}

bool TrajectoryVelocityExtractor::Validate(const FeatureSchema& schema, int32 channelIndex) const {
    return m_trajectoryIndex >= 0;
}

void RootVelocityExtractor::Extract(const Pose& pose, const PoseVelocity& velocity,
                                    const Trajectory& trajectory, float32* outData, int32 offset) const {
    if (velocity.boneCount <= 0) return;
    Vec3 vel = velocity.linearVelocities[0];
    outData[offset + 0] = vel.x;
    outData[offset + 1] = vel.y;
    outData[offset + 2] = vel.z;
}

void RootVelocityExtractor::ExtractNormalized(const Pose& pose, const PoseVelocity& velocity,
                                              const Trajectory& trajectory, float32* outData, int32 offset,
                                              const FeatureSchema& schema) const {
    Extract(pose, velocity, trajectory, outData, offset);
}

bool RootVelocityExtractor::Validate(const FeatureSchema& schema, int32 channelIndex) const {
    return true;
}

void FootContactExtractor::Extract(const Pose& pose, const PoseVelocity& velocity,
                                   const Trajectory& trajectory, float32* outData, int32 offset) const {
    float32 leftContact = 0.0f, rightContact = 0.0f;
    if (m_leftFootIndex >= 0 && m_leftFootIndex < velocity.boneCount) {
        float32 leftVelY = std::abs(velocity.linearVelocities[m_leftFootIndex].y);
        float32 leftPosY = pose.worldPositions[m_leftFootIndex].y;
        leftContact = (leftVelY < m_threshold && leftPosY < m_threshold) ? 1.0f : 0.0f;
    }
    if (m_rightFootIndex >= 0 && m_rightFootIndex < velocity.boneCount) {
        float32 rightVelY = std::abs(velocity.linearVelocities[m_rightFootIndex].y);
        float32 rightPosY = pose.worldPositions[m_rightFootIndex].y;
        rightContact = (rightVelY < m_threshold && rightPosY < m_threshold) ? 1.0f : 0.0f;
    }
    outData[offset + 0] = leftContact;
    outData[offset + 1] = rightContact;
}

void FootContactExtractor::ExtractNormalized(const Pose& pose, const PoseVelocity& velocity,
                                             const Trajectory& trajectory, float32* outData, int32 offset,
                                             const FeatureSchema& schema) const {
    Extract(pose, velocity, trajectory, outData, offset);
}

bool FootContactExtractor::Validate(const FeatureSchema& schema, int32 channelIndex) const {
    return m_leftFootIndex >= 0 && m_rightFootIndex >= 0;
}

// Registry
FeatureExtractorRegistry& FeatureExtractorRegistry::Instance() {
    static FeatureExtractorRegistry instance;
    return instance;
}

void FeatureExtractorRegistry::RegisterExtractor(FeatureChannelType type, IFeatureExtractor* extractor) {
    m_extractors.Insert(static_cast<uint32>(type), extractor);
}

IFeatureExtractor* FeatureExtractorRegistry::GetExtractor(FeatureChannelType type) {
    return m_extractors.Find(static_cast<uint32>(type));
}

const IFeatureExtractor* FeatureExtractorRegistry::GetExtractor(FeatureChannelType type) const {
    return m_extractors.Find(static_cast<uint32>(type));
}

void FeatureExtractorRegistry::Clear() {
    m_extractors.Clear();
}

// Manager
FeatureExtractorManager::FeatureExtractorManager() : m_allocator(GetDefaultAllocator()) {}
FeatureExtractorManager::~FeatureExtractorManager() {
    for (auto* ex : m_extractors) {
        Delete(m_allocator, ex);
    }
}

void FeatureExtractorManager::SetSchema(const FeatureSchema& schema) {
    m_schema = schema;
    BuildExtractors();
}

void FeatureExtractorManager::ExtractFeatures(const Pose& pose, const PoseVelocity& velocity,
                                              const Trajectory& trajectory, FeatureVector& outFeatures) const {
    outFeatures.Resize(m_schema.GetDimension());
    outFeatures.SetZero();
    int32 offset = 0;
    for (const auto& ch : m_schema.channels) {
        if (!ch.enabled) continue;
        for (const auto* ex : m_extractors) {
            if (ex->GetType() == ch.type) {
                ex->Extract(pose, velocity, trajectory, outFeatures.Data(), offset);
                break;
            }
        }
        offset += ch.dimension;
    }
}

void FeatureExtractorManager::ExtractFeaturesNormalized(const Pose& pose, const PoseVelocity& velocity,
                                                        const Trajectory& trajectory, FeatureVector& outFeatures) const {
    outFeatures.Resize(m_schema.GetDimension());
    outFeatures.SetZero();
    int32 offset = 0;
    for (const auto& ch : m_schema.channels) {
        if (!ch.enabled) continue;
        for (const auto* ex : m_extractors) {
            if (ex->GetType() == ch.type) {
                ex->ExtractNormalized(pose, velocity, trajectory, outFeatures.Data(), offset, m_schema);
                break;
            }
        }
        offset += ch.dimension;
    }
}

void FeatureExtractorManager::BuildExtractors() {
    for (auto* ex : m_extractors) Delete(m_allocator, ex);
    m_extractors.Clear();

    for (const auto& ch : m_schema.channels) {
        if (!ch.enabled) continue;
        IFeatureExtractor* extractor = nullptr;
        switch (ch.type) {
            case FeatureChannelType::BonePosition:
                extractor = New<BonePositionExtractor>(m_allocator, ch.boneIndex);
                break;
            case FeatureChannelType::BoneRotation:
                extractor = New<BoneRotationExtractor>(m_allocator, ch.boneIndex);
                break;
            case FeatureChannelType::BoneVelocity:
                extractor = New<BoneVelocityExtractor>(m_allocator, ch.boneIndex);
                break;
            case FeatureChannelType::TrajectoryPosition:
                extractor = New<TrajectoryPositionExtractor>(m_allocator, ch.trajectoryIndex);
                break;
            case FeatureChannelType::TrajectoryVelocity:
                extractor = New<TrajectoryVelocityExtractor>(m_allocator, ch.trajectoryIndex);
                break;
            case FeatureChannelType::RootVelocity:
                extractor = New<RootVelocityExtractor>(m_allocator);
                break;
            case FeatureChannelType::FootContact:
                extractor = New<FootContactExtractor>(m_allocator, ch.boneIndex, ch.boneIndex + 1);
                break;
            default:
                break;
        }
        if (extractor) m_extractors.PushBack(extractor);
    }
}

bool FeatureExtractorManager::ValidateSchema() const {
    if (!m_schema.IsValid()) return false;
    for (size_type i = 0; i < m_schema.channels.Size(); ++i) {
        const auto& ch = m_schema.channels[i];
        if (!ch.enabled) continue;
        for (const auto* ex : m_extractors) {
            if (ex->GetType() == ch.type && ex->Validate(m_schema, static_cast<int32>(i))) {
                return true;
            }
        }
    }
    return !m_extractors.IsEmpty();
}

MMV2_NAMESPACE_END
