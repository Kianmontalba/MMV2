// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Runtime Profiler & Analytics
// ============================================================================
// Provides detailed performance profiling, timing analysis, and statistical
// reporting for all MMV2 subsystems. Supports hierarchical profiling,
// memory tracking, and real-time performance visualization.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/HashMap.h"
#include "MMV2/Core/String.h"
#include "MMV2/Core/RingBuffer.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Profiler Configuration
// ============================================================================

struct ProfilerConfig
{
    bool enabled = true;
    bool captureHierarchy = true;
    bool captureMemory = true;
    bool captureGPU = false;
    uint32_t historyFrames = 300;           // 5 seconds at 60fps
    uint32_t maxZones = 256;
    float slowFrameThreshold = 16.67f;      // ms (60fps)
    bool autoCaptureSlowFrames = true;
    bool exportToFile = false;
    String exportPath;
};

// ============================================================================
// Profile Zone
// ============================================================================

struct ProfileZone
{
    String name;
    String category;
    uint64_t startTime;
    uint64_t endTime;
    uint64_t duration;          // nanoseconds
    uint32_t threadId;
    uint32_t depth;
    uint64_t memoryAllocated;
    uint64_t memoryFreed;
    uint32_t callCount;

    ProfileZone() : startTime(0), endTime(0), duration(0), threadId(0),
                    depth(0), memoryAllocated(0), memoryFreed(0), callCount(1) {}
};

// ============================================================================
// Profile Frame
// ============================================================================

struct ProfileFrame
{
    uint64_t frameNumber;
    float deltaTime;
    float totalTime;
    Vector<ProfileZone> zones;
    uint64_t totalMemoryUsed;
    uint64_t totalMemoryAllocated;
    uint64_t totalMemoryFreed;
    bool isSlowFrame;

    ProfileFrame() : frameNumber(0), deltaTime(0), totalTime(0),
                     totalMemoryUsed(0), totalMemoryAllocated(0),
                     totalMemoryFreed(0), isSlowFrame(false) {}
};

// ============================================================================
// Profile Statistics
// ============================================================================

struct ProfileStats
{
    String name;
    float minTime;
    float maxTime;
    float avgTime;
    float medianTime;
    float p95Time;
    float p99Time;
    uint32_t totalCalls;
    float totalTime;
    float percentageOfFrame;

    ProfileStats() : minTime(FLT_MAX), maxTime(0), avgTime(0), medianTime(0),
                     p95Time(0), p99Time(0), totalCalls(0), totalTime(0),
                     percentageOfFrame(0) {}
};

// ============================================================================
// Performance Budget
// ============================================================================

struct PerformanceBudget
{
    String category;
    float targetTimeMs;
    float warningThreshold;     // 0.8 = 80% of target
    float criticalThreshold;    // 1.0 = 100% of target
    uint32_t violationCount;

    PerformanceBudget() : targetTimeMs(16.67f), warningThreshold(0.8f),
                          criticalThreshold(1.0f), violationCount(0) {}
};

// ============================================================================
// Profiler
// ============================================================================

class MMV2_API Profiler
{
public:
    static Profiler& GetInstance();

    void Initialize(const ProfilerConfig& config);
    void Shutdown();

    // Frame management
    void BeginFrame();
    void EndFrame();

    // Zone profiling (RAII helper below)
    void BeginZone(const char* name, const char* category = nullptr);
    void EndZone();

    // Memory tracking
    void TrackAllocation(uint64_t size, const char* category = nullptr);
    void TrackDeallocation(uint64_t size, const char* category = nullptr);

    // Queries
    const ProfileFrame* GetCurrentFrame() const;
    const ProfileFrame* GetFrame(uint32_t framesAgo) const;
    uint32_t GetFrameHistoryCount() const;

    ProfileStats GetZoneStats(const String& name, uint32_t frameWindow = 60) const;
    ProfileStats GetCategoryStats(const String& category, uint32_t frameWindow = 60) const;

    Vector<ProfileStats> GetAllZoneStats(uint32_t frameWindow = 60) const;
    Vector<ProfileStats> GetAllCategoryStats(uint32_t frameWindow = 60) const;

    // Performance budgets
    void SetBudget(const String& category, float targetTimeMs);
    bool IsBudgetViolated(const String& category) const;
    Vector<String> GetViolatedBudgets() const;

    // Analytics
    float GetAverageFPS(uint32_t frameWindow = 60) const;
    float GetFrameTimeVariance(uint32_t frameWindow = 60) const;
    uint32_t GetSlowFrameCount(uint32_t frameWindow = 300) const;
    float GetSlowFramePercentage(uint32_t frameWindow = 300) const;

    // Export
    bool ExportToJSON(const String& filepath) const;
    bool ExportToCSV(const String& filepath) const;
    bool ExportToChromeTracing(const String& filepath) const;

    // Real-time data
    float GetCurrentFrameTime() const;
    float GetCurrentSearchTime() const;
    float GetCurrentBlendTime() const;
    float GetCurrentIKTime() const;

    // Reset
    void Reset();
    void ResetStats();

    bool IsEnabled() const { return m_config.enabled; }
    void SetEnabled(bool enabled) { m_config.enabled = enabled; }

private:
    Profiler();
    ~Profiler();

    ProfilerConfig m_config;
    RingBuffer<ProfileFrame> m_frameHistory;
    ProfileFrame* m_currentFrame;

    Vector<ProfileZone*> m_zoneStack;
    HashMap<String, PerformanceBudget> m_budgets;

    uint64_t m_frameCounter;
    uint64_t m_zoneCounter;

    mutable class SpinLock* m_lock;

    uint64_t GetTimestamp() const;
    float NanosecondsToMilliseconds(uint64_t ns) const;
    void ComputeStats(Vector<float>& times, ProfileStats& outStats) const;
};

// ============================================================================
// RAII Zone Scoper
// ============================================================================

class MMV2_API ProfileScope
{
public:
    ProfileScope(const char* name, const char* category = nullptr);
    ~ProfileScope();

private:
    bool m_active;
};

// ============================================================================
// Convenience Macros
// ============================================================================

#if MMV2_ENABLE_PROFILER
    #define MMV2_PROFILE_ZONE(name) ProfileScope _mmv2_zone_##__LINE__(name)
    #define MMV2_PROFILE_ZONE_CAT(name, cat) ProfileScope _mmv2_zone_##__LINE__(name, cat)
    #define MMV2_PROFILE_FUNCTION() MMV2_PROFILE_ZONE(__FUNCTION__)
    #define MMV2_PROFILE_FUNCTION_CAT(cat) MMV2_PROFILE_ZONE_CAT(__FUNCTION__, cat)
    #define MMV2_TRACK_ALLOC(size) Profiler::GetInstance().TrackAllocation(size)
    #define MMV2_TRACK_FREE(size) Profiler::GetInstance().TrackDeallocation(size)
#else
    #define MMV2_PROFILE_ZONE(name) ((void)0)
    #define MMV2_PROFILE_ZONE_CAT(name, cat) ((void)0)
    #define MMV2_PROFILE_FUNCTION() ((void)0)
    #define MMV2_PROFILE_FUNCTION_CAT(cat) ((void)0)
    #define MMV2_TRACK_ALLOC(size) ((void)0)
    #define MMV2_TRACK_FREE(size) ((void)0)
#endif

// ============================================================================
// Database Analytics
// ============================================================================

struct DatabaseAnalytics
{
    uint32_t totalPoses;
    uint32_t totalAnimations;
    uint32_t totalFeatures;
    uint32_t featureDimensions;
    uint64_t memoryUsage;
    float averageSearchTime;
    float averageBlendTime;
    float cacheHitRate;
    float averageCost;
    float minCost;
    float maxCost;
    float costVariance;
    uint32_t searchCount;
    uint32_t cacheHitCount;
    uint32_t cacheMissCount;

    DatabaseAnalytics() : totalPoses(0), totalAnimations(0), totalFeatures(0),
                          featureDimensions(0), memoryUsage(0), averageSearchTime(0),
                          averageBlendTime(0), cacheHitRate(0), averageCost(0),
                          minCost(FLT_MAX), maxCost(0), costVariance(0),
                          searchCount(0), cacheHitCount(0), cacheMissCount(0) {}
};

class MMV2_API DatabaseProfiler
{
public:
    void RecordSearch(float searchTime, float cost, bool cacheHit);
    void RecordBlend(float blendTime);
    void RecordCacheHit() { ++m_analytics.cacheHitCount; }
    void RecordCacheMiss() { ++m_analytics.cacheMissCount; }

    void UpdateDatabaseStats(const class PoseDatabase& database);

    const DatabaseAnalytics& GetAnalytics() const { return m_analytics; }
    DatabaseAnalytics ComputeRollingAnalytics(uint32_t windowSize) const;

    void Reset();

    bool ExportAnalytics(const String& filepath) const;

private:
    DatabaseAnalytics m_analytics;
    RingBuffer<float> m_searchTimes;
    RingBuffer<float> m_blendTimes;
    RingBuffer<float> m_costs;
};

// ============================================================================
// Search Quality Metrics
// ============================================================================

struct SearchQualityMetrics
{
    float averagePoseError;
    float maxPoseError;
    float trajectoryDeviation;
    float velocityDeviation;
    float footSkatingAmount;
    float rootMotionDrift;
    float continuityScore;
    float responsiveness;
    float naturalnessScore;

    SearchQualityMetrics() : averagePoseError(0), maxPoseError(0), trajectoryDeviation(0),
                             velocityDeviation(0), footSkatingAmount(0), rootMotionDrift(0),
                             continuityScore(0), responsiveness(0), naturalnessScore(0) {}
};

class MMV2_API SearchQualityAnalyzer
{
public:
    void RecordSearchResult(const class Pose& selectedPose, const class Pose& actualPose,
                            const class Trajectory& desiredTrajectory,
                            const class Trajectory& actualTrajectory);

    SearchQualityMetrics ComputeMetrics() const;
    void Reset();

private:
    Vector<float> m_poseErrors;
    Vector<float> m_trajectoryDeviations;
    Vector<float> m_velocityDeviations;
};

MMV2_NAMESPACE_END
