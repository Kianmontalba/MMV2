// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Debugger System - Editor Tool
// ============================================================================
// Provides comprehensive runtime debugging, visualization, and analysis
// for motion matching. Supports pose search debugging, cost visualization,
// trajectory comparison, and performance profiling.
// ============================================================================

#pragma once
#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/RingBuffer.h"
#include "MMV2/Core/HashMap.h"
#include "MMV2/Core/String.h"
#include "MMV2/Features/Feature.h"
#include "MMV2/Animation/Trajectory.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Debug Visualization Types
// ============================================================================

enum class DebugDrawType : uint32_t
{
    Point = 0,
    Line,
    Arrow,
    Sphere,
    Box,
    Capsule,
    Circle,
    Text,
    Trajectory,
    PoseSkeleton,
    Heatmap,
    BarChart,
    LineGraph
};

enum class DebugCategory : uint32_t
{
    General = 0,
    Search,
    Trajectory,
    Pose,
    Cost,
    Blend,
    IK,
    Event,
    Performance,
    Network,
    AI,
    Count
};

// ============================================================================
// Debug Draw Command
// ============================================================================

struct DebugDrawCommand
{
    DebugDrawType type;
    DebugCategory category;
    Vec3 position;
    Vec3 endPosition;
    Vec3 color;
    float size;
    float thickness;
    float duration;
    String text;
    uint32_t frameNumber;
    bool persistent;

    DebugDrawCommand()
        : type(DebugDrawType::Point), category(DebugCategory::General),
          position(Vec3::Zero()), endPosition(Vec3::Zero()),
          color(Vec3::One()), size(1.0f), thickness(1.0f),
          duration(0.0f), frameNumber(0), persistent(false) {}
};

// ============================================================================
// Search Debug Info
// ============================================================================

struct SearchDebugInfo
{
    uint32_t frameNumber;
    float searchTimeMs;
    uint32_t posesSearched;
    uint32_t posesEvaluated;
    uint32_t treeNodesVisited;
    float bestCost;
    float worstCost;
    float averageCost;
    uint32_t bestPoseIndex;
    uint32_t previousPoseIndex;
    bool transitionTriggered;
    float transitionCost;
    Trajectory queryTrajectory;
    Trajectory bestMatchTrajectory;
    FeatureVector queryFeatures;
    FeatureVector bestMatchFeatures;
    Vector<float> topCosts;
    Vector<uint32_t> topPoseIndices;
    String searchAlgorithm;

    SearchDebugInfo()
        : frameNumber(0), searchTimeMs(0), posesSearched(0),
          posesEvaluated(0), treeNodesVisited(0), bestCost(FLT_MAX),
          worstCost(0), averageCost(0), bestPoseIndex(0),
          previousPoseIndex(0), transitionTriggered(false),
          transitionCost(0) {}
};

// ============================================================================
// Cost Debug Info
// ============================================================================

struct CostBreakdown
{
    float poseCost;
    float trajectoryCost;
    float velocityCost;
    float headingCost;
    float phaseCost;
    float distanceCost;
    float curveCost;
    float interactionCost;
    float historyCost;
    float continuityCost;
    float totalCost;

    CostBreakdown()
        : poseCost(0), trajectoryCost(0), velocityCost(0),
          headingCost(0), phaseCost(0), distanceCost(0),
          curveCost(0), interactionCost(0), historyCost(0),
          continuityCost(0), totalCost(0) {}
};

struct CostDebugInfo
{
    uint32_t frameNumber;
    CostBreakdown queryCost;
    CostBreakdown bestMatchCost;
    Vector<CostBreakdown> topCandidates;
    Vector<float> featureDifferences;
    Vector<float> normalizedCosts;
    float normalizationFactor;

    CostDebugInfo() : frameNumber(0), normalizationFactor(1.0f) {}
};

// ============================================================================
// Trajectory Debug Info
// ============================================================================

struct TrajectoryDebugInfo
{
    uint32_t frameNumber;
    Trajectory desired;
    Trajectory actual;
    Trajectory predicted;
    Vector<float> positionErrors;
    Vector<float> velocityErrors;
    Vector<float> directionErrors;
    float maxPositionError;
    float maxVelocityError;
    float maxDirectionError;
    float averagePositionError;
    float averageVelocityError;
    float averageDirectionError;

    TrajectoryDebugInfo()
        : frameNumber(0), maxPositionError(0), maxVelocityError(0),
          maxDirectionError(0), averagePositionError(0),
          averageVelocityError(0), averageDirectionError(0) {}
};

// ============================================================================
// Performance Debug Info
// ============================================================================

struct PerformanceSnapshot
{
    uint32_t frameNumber;
    float totalFrameTime;
    float searchTime;
    float blendTime;
    float ikTime;
    float eventTime;
    float trajectoryTime;
    float poseUpdateTime;
    float renderTime;
    uint32_t memoryUsed;
    uint32_t memoryAllocated;
    uint32_t poseCount;
    uint32_t boneCount;
    float fps;

    PerformanceSnapshot()
        : frameNumber(0), totalFrameTime(0), searchTime(0),
          blendTime(0), ikTime(0), eventTime(0), trajectoryTime(0),
          poseUpdateTime(0), renderTime(0), memoryUsed(0),
          memoryAllocated(0), poseCount(0), boneCount(0), fps(0) {}
};

// ============================================================================
// Debug Settings
// ============================================================================

struct DebuggerSettings
{
    bool enabled;
    bool recordSearchHistory;
    bool recordCostHistory;
    bool recordTrajectoryHistory;
    bool recordPerformanceHistory;
    bool visualizeSearch;
    bool visualizeCosts;
    bool visualizeTrajectory;
    bool visualizePose;
    bool visualizeIK;
    bool showHeatmap;
    bool showBarCharts;
    bool showLineGraphs;
    bool showSkeleton;
    bool showBoneLabels;
    bool showVelocityVectors;
    bool showContactPoints;
    bool showFeatureVectors;
    bool showSearchRadius;
    bool showTransitionPath;
    uint32_t maxHistoryFrames;
    float slowFrameThreshold;
    bool autoCaptureSlowFrames;
    bool exportOnCapture;
    String exportPath;

    DebuggerSettings()
        : enabled(true), recordSearchHistory(true), recordCostHistory(true),
          recordTrajectoryHistory(true), recordPerformanceHistory(true),
          visualizeSearch(true), visualizeCosts(true), visualizeTrajectory(true),
          visualizePose(true), visualizeIK(true), showHeatmap(false),
          showBarCharts(true), showLineGraphs(true), showSkeleton(true),
          showBoneLabels(false), showVelocityVectors(false),
          showContactPoints(false), showFeatureVectors(false),
          showSearchRadius(true), showTransitionPath(true),
          maxHistoryFrames(300), slowFrameThreshold(16.67f),
          autoCaptureSlowFrames(true), exportOnCapture(false) {}
};

// ============================================================================
// Debugger
// ============================================================================

class MMV2_API Debugger
{
public:
    Debugger();
    ~Debugger();

    // === Initialization ===
    void Initialize();
    void Shutdown();

    // === Settings ===
    void SetSettings(const DebuggerSettings& settings);
    const DebuggerSettings& GetSettings() const { return m_settings; }

    // === Enable/Disable ===
    void SetEnabled(bool enabled);
    bool IsEnabled() const { return m_settings.enabled; }

    // === Search Debugging ===
    void RecordSearch(const SearchDebugInfo& info);
    const SearchDebugInfo* GetLastSearch() const;
    const SearchDebugInfo* GetSearchHistory(uint32_t frameOffset) const;
    Vector<const SearchDebugInfo*> GetSearchHistory() const;
    void ClearSearchHistory();

    // === Cost Debugging ===
    void RecordCost(const CostDebugInfo& info);
    const CostDebugInfo* GetLastCost() const;
    const CostDebugInfo* GetCostHistory(uint32_t frameOffset) const;
    Vector<const CostDebugInfo*> GetCostHistory() const;
    void ClearCostHistory();

    // === Trajectory Debugging ===
    void RecordTrajectory(const TrajectoryDebugInfo& info);
    const TrajectoryDebugInfo* GetLastTrajectory() const;
    const TrajectoryDebugInfo* GetTrajectoryHistory(uint32_t frameOffset) const;
    Vector<const TrajectoryDebugInfo*> GetTrajectoryHistory() const;
    void ClearTrajectoryHistory();

    // === Performance Debugging ===
    void RecordPerformance(const PerformanceSnapshot& snapshot);
    const PerformanceSnapshot* GetLastPerformance() const;
    const PerformanceSnapshot* GetPerformanceHistory(uint32_t frameOffset) const;
    Vector<const PerformanceSnapshot*> GetPerformanceHistory() const;
    void ClearPerformanceHistory();

    // === Debug Drawing ===
    void DrawPoint(DebugCategory category, const Vec3& pos, const Vec3& color,
                   float size, float duration, bool persistent);
    void DrawLine(DebugCategory category, const Vec3& from, const Vec3& to,
                  const Vec3& color, float thickness, float duration, bool persistent);
    void DrawArrow(DebugCategory category, const Vec3& from, const Vec3& to,
                   const Vec3& color, float thickness, float duration, bool persistent);
    void DrawSphere(DebugCategory category, const Vec3& center, float radius,
                    const Vec3& color, float duration, bool persistent);
    void DrawBox(DebugCategory category, const Vec3& center, const Vec3& extents,
                 const Vec3& color, float duration, bool persistent);
    void DrawText(DebugCategory category, const Vec3& pos, const String& text,
                  const Vec3& color, float duration, bool persistent);
    void DrawTrajectory(DebugCategory category, const Trajectory& trajectory,
                        const Vec3& color, float thickness, float duration, bool persistent);
    void DrawSkeleton(DebugCategory category, const Pose& pose, const Vec3& color,
                      float boneThickness, float duration, bool persistent);
    void ClearDrawCommands();
    void ClearDrawCommands(DebugCategory category);
    Vector<DebugDrawCommand> GetDrawCommands(DebugCategory category) const;

    // === Visualization ===
    void VisualizeSearch(const SearchDebugInfo& info);
    void VisualizeCosts(const CostDebugInfo& info);
    void VisualizeTrajectory(const TrajectoryDebugInfo& info);
    void VisualizePose(const Pose& pose, const Vec3& offset);
    void VisualizeIK(const Vector<Vec3>& targets, const Vector<Vec3>& results);
    void VisualizeHeatmap(const Vector<float>& values, const Vector<Vec3>& positions);
    void VisualizeBarChart(const Vector<float>& values, const Vec3& origin,
                           const Vec3& axis, float barWidth, float spacing);
    void VisualizeLineGraph(const Vector<float>& values, const Vec3& origin,
                            const Vec3& axis, float scale);

    // === Analysis ===
    float GetAverageSearchTime() const;
    float GetMaxSearchTime() const;
    float GetMinSearchTime() const;
    float GetSearchTimeStdDev() const;
    uint32_t GetSlowFrameCount() const;
    float GetAverageFPS() const;
    String GenerateReport() const;

    // === Export ===
    bool ExportSearchHistory(const String& filePath) const;
    bool ExportCostHistory(const String& filePath) const;
    bool ExportTrajectoryHistory(const String& filePath) const;
    bool ExportPerformanceHistory(const String& filePath) const;
    bool ExportFullReport(const String& filePath) const;

    // === Comparison ===
    void BeginComparison();
    void AddComparisonFrame(uint32_t frameNumber);
    void EndComparison();
    bool IsComparing() const { return m_isComparing; }

    // === Update ===
    void Update(float deltaTime);

    // === Debug ===
    void SetDebugEnabled(bool enabled);
    String GetDebugInfo() const;

private:
    DebuggerSettings m_settings;

    // History buffers
    RingBuffer<SearchDebugInfo> m_searchHistory;
    RingBuffer<CostDebugInfo> m_costHistory;
    RingBuffer<TrajectoryDebugInfo> m_trajectoryHistory;
    RingBuffer<PerformanceSnapshot> m_performanceHistory;

    // Draw commands
    Vector<DebugDrawCommand> m_drawCommands;

    // Comparison
    bool m_isComparing;
    Vector<uint32_t> m_comparisonFrames;

    // State
    bool m_initialized;
    bool m_debugEnabled;
    uint32_t m_currentFrame;

    // Statistics
    float m_totalSearchTime;
    float m_maxSearchTime;
    float m_minSearchTime;
    uint32_t m_searchCount;
    uint32_t m_slowFrameCount;

    // Internal
    void UpdateDrawCommands(float deltaTime);
    void AutoCaptureSlowFrame(const PerformanceSnapshot& snapshot);
    void ExportSnapshot(const PerformanceSnapshot& snapshot);
};

MMV2_NAMESPACE_END
