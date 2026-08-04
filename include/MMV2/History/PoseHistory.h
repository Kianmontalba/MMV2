// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Pose History System
// ============================================================================
// Records past poses for continuity matching and history-based scoring.
// Supports circular buffer, configurable history length, and attribute tagging.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/RingBuffer.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Core/Transform.h"
#include "MMV2/Features/Feature.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Pose History Entry
// ============================================================================

struct PoseHistoryEntry
{
    Pose pose;
    float timestamp;
    float deltaTime;
    uint32_t animationIndex;
    float animationTime;
    uint32_t frameNumber;
    Transform rootTransform;
    Vec3 rootVelocity;
    Vec3 rootAngularVelocity;
    bool isMirrored;

    PoseHistoryEntry() : timestamp(0), deltaTime(0), animationIndex(0),
                         animationTime(0), frameNumber(0), isMirrored(false) {}
};

// ============================================================================
// Pose History Attribute
// ============================================================================

enum class PoseHistoryAttribute : uint32_t
{
    None            = 0,
    RootMotion      = 1 << 0,
    FullBody        = 1 << 1,
    UpperBody       = 1 << 2,
    LowerBody       = 1 << 3,
    LeftSide        = 1 << 4,
    RightSide       = 1 << 5,
    Mirrored        = 1 << 6,
    Interaction     = 1 << 7,
    Custom1         = 1 << 8,
    Custom2         = 1 << 9,
    Custom3         = 1 << 10,
    Custom4         = 1 << 11
};

MMV2_ENUM_CLASS_FLAGS(PoseHistoryAttribute)

// ============================================================================
// Pose History Configuration
// ============================================================================

struct PoseHistoryConfig
{
    uint32_t maxHistoryFrames = 120;        // ~2 seconds at 60fps
    uint32_t historySampleRate = 1;         // Sample every N frames
    float historyTimeWindow = 2.0f;         // Maximum time window in seconds
    bool recordRootMotion = true;
    bool recordVelocities = true;
    bool recordFeatures = true;
    bool enableCompression = false;
    float compressionTolerance = 0.001f;
    PoseHistoryAttribute defaultAttributes = PoseHistoryAttribute::FullBody;
};

// ============================================================================
// Pose History Provider Interface
// ============================================================================

class MMV2_API IPoseHistoryProvider
{
public:
    virtual ~IPoseHistoryProvider() = default;

    virtual const PoseHistoryEntry* GetHistoryEntry(int32_t framesAgo) const = 0;
    virtual const PoseHistoryEntry* GetHistoryEntryAtTime(float timeAgo) const = 0;
    virtual uint32_t GetHistoryCount() const = 0;
    virtual float GetHistoryDuration() const = 0;

    virtual void GetHistoryFeatureVector(FeatureVector& outVector, 
                                          const PoseHistoryConfig& config) const = 0;
    virtual void GetHistoryTrajectory(Vector<Vec3>& outTrajectory, float timeHorizon) const = 0;
};

// ============================================================================
// Pose History
// ============================================================================

class MMV2_API PoseHistory : public IPoseHistoryProvider
{
public:
    PoseHistory();
    explicit PoseHistory(const PoseHistoryConfig& config);

    void Initialize(const PoseHistoryConfig& config);
    void Reset();

    // Recording
    void Record(const Pose& pose, float timestamp, float deltaTime,
                uint32_t animIndex, float animTime, const Transform& rootTransform);
    void Record(const PoseHistoryEntry& entry);

    // Query
    const PoseHistoryEntry* GetHistoryEntry(int32_t framesAgo) const override;
    const PoseHistoryEntry* GetHistoryEntryAtTime(float timeAgo) const override;
    uint32_t GetHistoryCount() const override { return m_entries.GetCount(); }
    float GetHistoryDuration() const override;

    // Feature extraction from history
    void GetHistoryFeatureVector(FeatureVector& outVector,
                                  const PoseHistoryConfig& config) const override;
    void GetHistoryTrajectory(Vector<Vec3>& outTrajectory, float timeHorizon) const override;

    // History-based queries
    float GetAverageVelocityOverTime(float timeWindow) const;
    Vec3 GetAverageDirectionOverTime(float timeWindow) const;
    float GetSpeedVarianceOverTime(float timeWindow) const;

    // Continuity scoring
    float ComputeContinuityScore(const Pose& candidatePose, float candidateTime) const;
    float ComputeVelocityContinuity(const Vec3& candidateVelocity) const;

    // Mirroring support
    void SetMirroringEnabled(bool enabled) { m_mirroringEnabled = enabled; }
    bool IsMirroringEnabled() const { return m_mirroringEnabled; }

    // Attributes
    void SetEntryAttribute(uint32_t index, PoseHistoryAttribute attr);
    bool HasEntryAttribute(uint32_t index, PoseHistoryAttribute attr) const;

    // Serialization
    void Serialize(class BinarySerializer& serializer) const;
    void Deserialize(class BinarySerializer& serializer);

    // Iterator
    class Iterator
    {
    public:
        Iterator(const PoseHistory& history, uint32_t index);

        const PoseHistoryEntry& operator*() const;
        const PoseHistoryEntry* operator->() const;
        Iterator& operator++();
        bool operator!=(const Iterator& other) const;

    private:
        const PoseHistory& m_history;
        uint32_t m_index;
    };

    Iterator Begin() const;
    Iterator End() const;

private:
    PoseHistoryConfig m_config;
    RingBuffer<PoseHistoryEntry> m_entries;
    Vector<PoseHistoryAttribute> m_entryAttributes;
    float m_currentTime;
    bool m_mirroringEnabled;

    void CompressEntry(PoseHistoryEntry& entry);
};

// ============================================================================
// Pose History Collector (Animation Node equivalent)
// ============================================================================

class MMV2_API PoseHistoryCollector
{
public:
    PoseHistoryCollector();

    void SetTargetHistory(PoseHistory* history) { m_targetHistory = history; }
    PoseHistory* GetTargetHistory() const { return m_targetHistory; }

    void Update(const Pose& currentPose, float deltaTime,
                uint32_t animIndex, float animTime, const Transform& rootTransform);

    void SetEnabled(bool enabled) { m_enabled = enabled; }
    bool IsEnabled() const { return m_enabled; }

    void SetSampleRate(uint32_t rate) { m_sampleRate = rate; }
    uint32_t GetSampleRate() const { return m_sampleRate; }

private:
    PoseHistory* m_targetHistory;
    bool m_enabled;
    uint32_t m_sampleRate;
    uint32_t m_frameCounter;
    float m_accumulatedTime;
};

// ============================================================================
// Pose History Utilities
// ============================================================================

class MMV2_API PoseHistoryUtils
{
public:
    static void BlendHistoryEntry(PoseHistoryEntry& result,
                                   const PoseHistoryEntry& a,
                                   const PoseHistoryEntry& b,
                                   float t);

    static float ComputePoseDifference(const PoseHistoryEntry& a,
                                        const PoseHistoryEntry& b);

    static void ExtractFeatureWindow(const PoseHistory& history,
                                      float startTime, float endTime,
                                      Vector<float>& outFeatures);

    static bool FindBestHistoryMatch(const PoseHistory& history,
                                      const FeatureVector& query,
                                      uint32_t& outBestIndex,
                                      float& outBestCost);

    static void GenerateHistoryFeatureChannels(const PoseHistory& history,
                                                FeatureChannelConfig& outConfig);
};

MMV2_NAMESPACE_END
