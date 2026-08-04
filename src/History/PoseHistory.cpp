// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Pose History System Implementation
// ============================================================================

#include "MMV2/History/PoseHistory.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// PoseHistory
// ============================================================================

PoseHistory::PoseHistory() : m_currentTime(0.0f), m_mirroringEnabled(false)
{
}

PoseHistory::PoseHistory(const PoseHistoryConfig& config)
    : m_config(config), m_currentTime(0.0f), m_mirroringEnabled(false)
{
    Initialize(config);
}

void PoseHistory::Initialize(const PoseHistoryConfig& config)
{
    m_config = config;
    m_entries.Resize(config.maxHistoryFrames);
    m_entryAttributes.Resize(config.maxHistoryFrames);
    m_currentTime = 0.0f;

    for (uint32_t i = 0; i < config.maxHistoryFrames; ++i)
        m_entryAttributes[i] = config.defaultAttributes;
}

void PoseHistory::Reset()
{
    m_entries.Clear();
    m_currentTime = 0.0f;

    for (auto& attr : m_entryAttributes)
        attr = m_config.defaultAttributes;
}

void PoseHistory::Record(const Pose& pose, float timestamp, float deltaTime,
                          uint32_t animIndex, float animTime, const Transform& rootTransform)
{
    if (!m_config.recordRootMotion && !m_config.recordVelocities)
        return;

    PoseHistoryEntry entry;
    entry.pose = pose;
    entry.timestamp = timestamp;
    entry.deltaTime = deltaTime;
    entry.animationIndex = animIndex;
    entry.animationTime = animTime;
    entry.frameNumber = static_cast<uint32_t>(m_entries.GetCount());
    entry.rootTransform = rootTransform;
    entry.isMirrored = m_mirroringEnabled;

    // Compute velocities from previous entry
    if (m_entries.GetCount() > 0)
    {
        const PoseHistoryEntry& prev = *m_entries.Back();
        float dt = timestamp - prev.timestamp;
        if (dt > 1e-6f)
        {
            entry.rootVelocity = (rootTransform.position - prev.rootTransform.position) / dt;

            Quat deltaRot = rootTransform.rotation * prev.rootTransform.rotation.Inverse();
            Vec3 axis;
            float angle;
            deltaRot.ToAxisAngle(axis, angle);
            entry.rootAngularVelocity = axis * (angle / dt);
        }
    }

    if (m_config.enableCompression)
        CompressEntry(entry);

    m_entries.PushBack(entry);
    m_currentTime = timestamp;
}

void PoseHistory::Record(const PoseHistoryEntry& entry)
{
    PoseHistoryEntry copy = entry;
    if (m_config.enableCompression)
        CompressEntry(copy);
    m_entries.PushBack(copy);
    m_currentTime = entry.timestamp;
}

const PoseHistoryEntry* PoseHistory::GetHistoryEntry(int32_t framesAgo) const
{
    if (framesAgo < 0 || static_cast<uint32_t>(framesAgo) >= m_entries.GetCount())
        return nullptr;

    uint32_t index = m_entries.GetCount() - 1 - static_cast<uint32_t>(framesAgo);
    return &m_entries[index];
}

const PoseHistoryEntry* PoseHistory::GetHistoryEntryAtTime(float timeAgo) const
{
    if (m_entries.GetCount() == 0)
        return nullptr;

    float targetTime = m_currentTime - timeAgo;

    // Binary search for closest entry
    int32_t low = 0;
    int32_t high = static_cast<int32_t>(m_entries.GetCount()) - 1;

    while (low < high)
    {
        int32_t mid = (low + high + 1) / 2;
        if (m_entries[mid].timestamp <= targetTime)
            low = mid;
        else
            high = mid - 1;
    }

    return &m_entries[low];
}

float PoseHistory::GetHistoryDuration() const
{
    if (m_entries.GetCount() < 2)
        return 0.0f;

    return m_entries.Back()->timestamp - m_entries.Front()->timestamp;
}

void PoseHistory::GetHistoryFeatureVector(FeatureVector& outVector,
                                           const PoseHistoryConfig& config) const
{
    // Encode history into feature vector
    // Include: past velocities, past positions, acceleration patterns
    const uint32_t historySamples = Math::Min(static_cast<uint32_t>(m_entries.GetCount()), 
                                               config.maxHistoryFrames / config.historySampleRate);

    uint32_t featureIndex = 0;

    for (uint32_t i = 0; i < historySamples; ++i)
    {
        uint32_t entryIndex = m_entries.GetCount() - 1 - (i * config.historySampleRate);
        if (entryIndex >= m_entries.GetCount())
            break;

        const PoseHistoryEntry& entry = m_entries[entryIndex];

        if (config.recordVelocities && featureIndex + 3 <= outVector.dimensions)
        {
            outVector.data[featureIndex++] = entry.rootVelocity.x;
            outVector.data[featureIndex++] = entry.rootVelocity.y;
            outVector.data[featureIndex++] = entry.rootVelocity.z;
        }

        if (config.recordRootMotion && featureIndex + 3 <= outVector.dimensions)
        {
            outVector.data[featureIndex++] = entry.rootTransform.position.x;
            outVector.data[featureIndex++] = entry.rootTransform.position.y;
            outVector.data[featureIndex++] = entry.rootTransform.position.z;
        }
    }

    // Zero-pad remaining dimensions
    while (featureIndex < outVector.dimensions)
        outVector.data[featureIndex++] = 0.0f;
}

void PoseHistory::GetHistoryTrajectory(Vector<Vec3>& outTrajectory, float timeHorizon) const
{
    outTrajectory.Clear();

    if (m_entries.GetCount() == 0)
        return;

    const float sampleInterval = 0.1f; // 100ms samples
    uint32_t numSamples = static_cast<uint32_t>(timeHorizon / sampleInterval) + 1;
    outTrajectory.Reserve(numSamples);

    for (uint32_t i = 0; i < numSamples; ++i)
    {
        float timeAgo = i * sampleInterval;
        const PoseHistoryEntry* entry = GetHistoryEntryAtTime(timeAgo);
        if (entry)
            outTrajectory.PushBack(entry->rootTransform.position);
        else
            break;
    }
}

float PoseHistory::GetAverageVelocityOverTime(float timeWindow) const
{
    if (m_entries.GetCount() < 2)
        return 0.0f;

    Vec3 totalVelocity(0, 0, 0);
    uint32_t count = 0;
    float cutoffTime = m_currentTime - timeWindow;

    for (const auto& entry : m_entries)
    {
        if (entry.timestamp < cutoffTime)
            continue;

        totalVelocity += entry.rootVelocity;
        ++count;
    }

    if (count == 0)
        return 0.0f;

    return totalVelocity.Magnitude() / static_cast<float>(count);
}

Vec3 PoseHistory::GetAverageDirectionOverTime(float timeWindow) const
{
    if (m_entries.GetCount() < 2)
        return Vec3(0, 0, 1);

    Vec3 totalDirection(0, 0, 0);
    uint32_t count = 0;
    float cutoffTime = m_currentTime - timeWindow;

    for (const auto& entry : m_entries)
    {
        if (entry.timestamp < cutoffTime)
            continue;

        Vec3 forward = entry.rootTransform.rotation * Vec3(0, 0, 1);
        totalDirection += forward;
        ++count;
    }

    if (count == 0)
        return Vec3(0, 0, 1);

    totalDirection /= static_cast<float>(count);
    return totalDirection.Normalized();
}

float PoseHistory::GetSpeedVarianceOverTime(float timeWindow) const
{
    if (m_entries.GetCount() < 2)
        return 0.0f;

    float cutoffTime = m_currentTime - timeWindow;
    Vector<float> speeds;

    for (const auto& entry : m_entries)
    {
        if (entry.timestamp < cutoffTime)
            continue;
        speeds.PushBack(entry.rootVelocity.Magnitude());
    }

    if (speeds.Size() < 2)
        return 0.0f;

    float mean = 0.0f;
    for (float s : speeds)
        mean += s;
    mean /= static_cast<float>(speeds.Size());

    float variance = 0.0f;
    for (float s : speeds)
        variance += (s - mean) * (s - mean);
    variance /= static_cast<float>(speeds.Size());

    return variance;
}

float PoseHistory::ComputeContinuityScore(const Pose& candidatePose, float candidateTime) const
{
    if (m_entries.GetCount() == 0)
        return 1.0f;

    const PoseHistoryEntry& lastEntry = *m_entries.Back();

    // Compute pose difference between last recorded pose and candidate
    float poseDiff = Pose::ComputeDifference(lastEntry.pose, candidatePose);

    // Time-based decay factor (older history = less weight)
    float timeDiff = candidateTime - lastEntry.timestamp;
    float timeDecay = Math::Exp(-timeDiff * 2.0f);

    // Velocity continuity
    Vec3 predictedVelocity = lastEntry.rootVelocity;
    // ... (could integrate trajectory prediction here)

    float continuityScore = 1.0f - Math::Clamp(poseDiff * timeDecay, 0.0f, 1.0f);
    return continuityScore;
}

float PoseHistory::ComputeVelocityContinuity(const Vec3& candidateVelocity) const
{
    if (m_entries.GetCount() == 0)
        return 1.0f;

    const Vec3& lastVelocity = m_entries.Back()->rootVelocity;
    float velocityDiff = (candidateVelocity - lastVelocity).Magnitude();
    float maxExpectedDiff = 10.0f; // Configurable

    return 1.0f - Math::Clamp(velocityDiff / maxExpectedDiff, 0.0f, 1.0f);
}

void PoseHistory::SetEntryAttribute(uint32_t index, PoseHistoryAttribute attr)
{
    if (index < m_entryAttributes.Size())
        m_entryAttributes[index] = attr;
}

bool PoseHistory::HasEntryAttribute(uint32_t index, PoseHistoryAttribute attr) const
{
    if (index >= m_entryAttributes.Size())
        return false;
    return (static_cast<uint32_t>(m_entryAttributes[index]) & static_cast<uint32_t>(attr)) != 0;
}

void PoseHistory::CompressEntry(PoseHistoryEntry& entry)
{
    // Simple quantization compression
    // Round positions and rotations to reduce precision
    const float quantScale = 1.0f / m_config.compressionTolerance;

    entry.rootTransform.position.x = std::round(entry.rootTransform.position.x * quantScale) / quantScale;
    entry.rootTransform.position.y = std::round(entry.rootTransform.position.y * quantScale) / quantScale;
    entry.rootTransform.position.z = std::round(entry.rootTransform.position.z * quantScale) / quantScale;

    entry.rootVelocity.x = std::round(entry.rootVelocity.x * quantScale) / quantScale;
    entry.rootVelocity.y = std::round(entry.rootVelocity.y * quantScale) / quantScale;
    entry.rootVelocity.z = std::round(entry.rootVelocity.z * quantScale) / quantScale;
}

// ============================================================================
// PoseHistory Iterator
// ============================================================================

PoseHistory::Iterator::Iterator(const PoseHistory& history, uint32_t index)
    : m_history(history), m_index(index)
{
}

const PoseHistoryEntry& PoseHistory::Iterator::operator*() const
{
    return m_history.m_entries[m_index];
}

const PoseHistoryEntry* PoseHistory::Iterator::operator->() const
{
    return &m_history.m_entries[m_index];
}

PoseHistory::Iterator& PoseHistory::Iterator::operator++()
{
    ++m_index;
    return *this;
}

bool PoseHistory::Iterator::operator!=(const Iterator& other) const
{
    return m_index != other.m_index;
}

PoseHistory::Iterator PoseHistory::Begin() const
{
    return Iterator(*this, 0);
}

PoseHistory::Iterator PoseHistory::End() const
{
    return Iterator(*this, m_entries.GetCount());
}

// ============================================================================
// PoseHistoryCollector
// ============================================================================

PoseHistoryCollector::PoseHistoryCollector()
    : m_targetHistory(nullptr), m_enabled(true), m_sampleRate(1), m_frameCounter(0), m_accumulatedTime(0)
{
}

void PoseHistoryCollector::Update(const Pose& currentPose, float deltaTime,
                                   uint32_t animIndex, float animTime, const Transform& rootTransform)
{
    if (!m_enabled || !m_targetHistory)
        return;

    m_accumulatedTime += deltaTime;
    ++m_frameCounter;

    if (m_frameCounter % m_sampleRate != 0)
        return;

    m_targetHistory->Record(currentPose, m_accumulatedTime, deltaTime, animIndex, animTime, rootTransform);
}

// ============================================================================
// PoseHistoryUtils
// ============================================================================

void PoseHistoryUtils::BlendHistoryEntry(PoseHistoryEntry& result,
                                          const PoseHistoryEntry& a,
                                          const PoseHistoryEntry& b,
                                          float t)
{
    result.timestamp = Math::Lerp(a.timestamp, b.timestamp, t);
    result.deltaTime = Math::Lerp(a.deltaTime, b.deltaTime, t);
    result.animationIndex = a.animationIndex;
    result.animationTime = Math::Lerp(a.animationTime, b.animationTime, t);
    result.frameNumber = a.frameNumber;

    result.rootTransform.position = Vec3::Lerp(a.rootTransform.position, b.rootTransform.position, t);
    result.rootTransform.rotation = Quat::Slerp(a.rootTransform.rotation, b.rootTransform.rotation, t);
    result.rootTransform.scale = Vec3::Lerp(a.rootTransform.scale, b.rootTransform.scale, t);

    result.rootVelocity = Vec3::Lerp(a.rootVelocity, b.rootVelocity, t);
    result.rootAngularVelocity = Vec3::Lerp(a.rootAngularVelocity, b.rootAngularVelocity, t);

    result.isMirrored = a.isMirrored;

    // Blend poses
    Pose::Blend(result.pose, a.pose, b.pose, t);
}

float PoseHistoryUtils::ComputePoseDifference(const PoseHistoryEntry& a,
                                               const PoseHistoryEntry& b)
{
    float posDiff = (a.rootTransform.position - b.rootTransform.position).Magnitude();
    float rotDiff = 1.0f - std::abs(Quat::Dot(a.rootTransform.rotation, b.rootTransform.rotation));
    float velDiff = (a.rootVelocity - b.rootVelocity).Magnitude();

    return posDiff + rotDiff * 10.0f + velDiff * 0.1f;
}

void PoseHistoryUtils::ExtractFeatureWindow(const PoseHistory& history,
                                             float startTime, float endTime,
                                             Vector<float>& outFeatures)
{
    outFeatures.Clear();

    const uint32_t count = history.GetHistoryCount();
    for (uint32_t i = 0; i < count; ++i)
    {
        const PoseHistoryEntry* entry = history.GetHistoryEntry(static_cast<int32_t>(count - 1 - i));
        if (!entry)
            break;

        if (entry->timestamp >= startTime && entry->timestamp <= endTime)
        {
            outFeatures.PushBack(entry->rootVelocity.x);
            outFeatures.PushBack(entry->rootVelocity.y);
            outFeatures.PushBack(entry->rootVelocity.z);
            outFeatures.PushBack(entry->rootTransform.position.x);
            outFeatures.PushBack(entry->rootTransform.position.y);
            outFeatures.PushBack(entry->rootTransform.position.z);
        }
    }
}

bool PoseHistoryUtils::FindBestHistoryMatch(const PoseHistory& history,
                                             const FeatureVector& query,
                                             uint32_t& outBestIndex,
                                             float& outBestCost)
{
    outBestCost = FLT_MAX;
    outBestIndex = 0;
    bool found = false;

    const uint32_t count = history.GetHistoryCount();
    for (uint32_t i = 0; i < count; ++i)
    {
        // Simplified: compare based on velocity and position
        const PoseHistoryEntry* entry = history.GetHistoryEntry(static_cast<int32_t>(i));
        if (!entry)
            continue;

        float cost = 0.0f;
        for (uint32_t d = 0; d < query.dimensions && d < 6; ++d)
        {
            float value = (d < 3) ? entry->rootVelocity[d] : entry->rootTransform.position[d - 3];
            float diff = query.data[d] - value;
            cost += diff * diff;
        }

        if (cost < outBestCost)
        {
            outBestCost = cost;
            outBestIndex = i;
            found = true;
        }
    }

    return found;
}

void PoseHistoryUtils::GenerateHistoryFeatureChannels(const PoseHistory& history,
                                                       FeatureChannelConfig& outConfig)
{
    // Generate feature channel configuration based on history data
    outConfig.channels.Clear();

    // Add velocity history channel
    FeatureChannelConfig::Channel velChannel;
    velChannel.type = FeatureChannelType::Velocity;
    velChannel.dimensions = 3;
    velChannel.weight = 1.0f;
    outConfig.channels.PushBack(velChannel);

    // Add position history channel
    FeatureChannelConfig::Channel posChannel;
    posChannel.type = FeatureChannelType::Position;
    posChannel.dimensions = 3;
    posChannel.weight = 0.5f;
    outConfig.channels.PushBack(posChannel);

    // Add trajectory channel
    FeatureChannelConfig::Channel trajChannel;
    trajChannel.type = FeatureChannelType::Trajectory;
    trajChannel.dimensions = 6; // 2 future points * 3D
    trajChannel.weight = 2.0f;
    outConfig.channels.PushBack(trajChannel);
}

MMV2_NAMESPACE_END
