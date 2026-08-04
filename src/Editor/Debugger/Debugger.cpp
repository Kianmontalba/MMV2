// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Debugger Implementation
// ============================================================================

#include "MMV2/Editor/Debugger/Debugger.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Debugger
// ============================================================================

Debugger::Debugger()
    : m_searchHistory(300)
    , m_costHistory(300)
    , m_trajectoryHistory(300)
    , m_performanceHistory(300)
    , m_isComparing(false)
    , m_initialized(false)
    , m_debugEnabled(false)
    , m_currentFrame(0)
    , m_totalSearchTime(0.0f)
    , m_maxSearchTime(0.0f)
    , m_minSearchTime(FLT_MAX)
    , m_searchCount(0)
    , m_slowFrameCount(0)
{
}

Debugger::~Debugger()
{
    Shutdown();
}

void Debugger::Initialize()
{
    if (m_initialized)
        return;

    m_currentFrame = 0;
    m_totalSearchTime = 0.0f;
    m_maxSearchTime = 0.0f;
    m_minSearchTime = FLT_MAX;
    m_searchCount = 0;
    m_slowFrameCount = 0;

    m_initialized = true;
}

void Debugger::Shutdown()
{
    if (!m_initialized)
        return;

    m_searchHistory.Clear();
    m_costHistory.Clear();
    m_trajectoryHistory.Clear();
    m_performanceHistory.Clear();
    m_drawCommands.Clear();
    m_comparisonFrames.Clear();

    m_initialized = false;
}

// ============================================================================
// Settings
// ============================================================================

void Debugger::SetSettings(const DebuggerSettings& settings)
{
    m_settings = settings;

    // Resize history buffers if needed
    if (m_settings.maxHistoryFrames != m_searchHistory.Capacity())
    {
        m_searchHistory.Resize(m_settings.maxHistoryFrames);
        m_costHistory.Resize(m_settings.maxHistoryFrames);
        m_trajectoryHistory.Resize(m_settings.maxHistoryFrames);
        m_performanceHistory.Resize(m_settings.maxHistoryFrames);
    }
}

void Debugger::SetEnabled(bool enabled)
{
    m_settings.enabled = enabled;
}

// ============================================================================
// Search Debugging
// ============================================================================

void Debugger::RecordSearch(const SearchDebugInfo& info)
{
    if (!m_settings.enabled || !m_settings.recordSearchHistory)
        return;

    m_searchHistory.PushBack(info);

    // Update statistics
    m_totalSearchTime += info.searchTimeMs;
    m_maxSearchTime = Math::Max(m_maxSearchTime, info.searchTimeMs);
    m_minSearchTime = Math::Min(m_minSearchTime, info.searchTimeMs);
    m_searchCount++;

    // Visualize if enabled
    if (m_settings.visualizeSearch)
    {
        VisualizeSearch(info);
    }
}

const SearchDebugInfo* Debugger::GetLastSearch() const
{
    if (m_searchHistory.Empty())
        return nullptr;
    return &m_searchHistory.Back();
}

const SearchDebugInfo* Debugger::GetSearchHistory(uint32_t frameOffset) const
{
    if (frameOffset >= m_searchHistory.Size())
        return nullptr;
    return &m_searchHistory[m_searchHistory.Size() - 1 - frameOffset];
}

Vector<const SearchDebugInfo*> Debugger::GetSearchHistory() const
{
    Vector<const SearchDebugInfo*> result;
    result.Reserve(m_searchHistory.Size());

    for (size_type i = 0; i < m_searchHistory.Size(); ++i)
    {
        result.PushBack(&m_searchHistory[i]);
    }

    return result;
}

void Debugger::ClearSearchHistory()
{
    m_searchHistory.Clear();
    m_totalSearchTime = 0.0f;
    m_maxSearchTime = 0.0f;
    m_minSearchTime = FLT_MAX;
    m_searchCount = 0;
}

// ============================================================================
// Cost Debugging
// ============================================================================

void Debugger::RecordCost(const CostDebugInfo& info)
{
    if (!m_settings.enabled || !m_settings.recordCostHistory)
        return;

    m_costHistory.PushBack(info);

    if (m_settings.visualizeCosts)
    {
        VisualizeCosts(info);
    }
}

const CostDebugInfo* Debugger::GetLastCost() const
{
    if (m_costHistory.Empty())
        return nullptr;
    return &m_costHistory.Back();
}

const CostDebugInfo* Debugger::GetCostHistory(uint32_t frameOffset) const
{
    if (frameOffset >= m_costHistory.Size())
        return nullptr;
    return &m_costHistory[m_costHistory.Size() - 1 - frameOffset];
}

Vector<const CostDebugInfo*> Debugger::GetCostHistory() const
{
    Vector<const CostDebugInfo*> result;
    result.Reserve(m_costHistory.Size());

    for (size_type i = 0; i < m_costHistory.Size(); ++i)
    {
        result.PushBack(&m_costHistory[i]);
    }

    return result;
}

void Debugger::ClearCostHistory()
{
    m_costHistory.Clear();
}

// ============================================================================
// Trajectory Debugging
// ============================================================================

void Debugger::RecordTrajectory(const TrajectoryDebugInfo& info)
{
    if (!m_settings.enabled || !m_settings.recordTrajectoryHistory)
        return;

    m_trajectoryHistory.PushBack(info);

    if (m_settings.visualizeTrajectory)
    {
        VisualizeTrajectory(info);
    }
}

const TrajectoryDebugInfo* Debugger::GetLastTrajectory() const
{
    if (m_trajectoryHistory.Empty())
        return nullptr;
    return &m_trajectoryHistory.Back();
}

const TrajectoryDebugInfo* Debugger::GetTrajectoryHistory(uint32_t frameOffset) const
{
    if (frameOffset >= m_trajectoryHistory.Size())
        return nullptr;
    return &m_trajectoryHistory[m_trajectoryHistory.Size() - 1 - frameOffset];
}

Vector<const TrajectoryDebugInfo*> Debugger::GetTrajectoryHistory() const
{
    Vector<const TrajectoryDebugInfo*> result;
    result.Reserve(m_trajectoryHistory.Size());

    for (size_type i = 0; i < m_trajectoryHistory.Size(); ++i)
    {
        result.PushBack(&m_trajectoryHistory[i]);
    }

    return result;
}

void Debugger::ClearTrajectoryHistory()
{
    m_trajectoryHistory.Clear();
}

// ============================================================================
// Performance Debugging
// ============================================================================

void Debugger::RecordPerformance(const PerformanceSnapshot& snapshot)
{
    if (!m_settings.enabled || !m_settings.recordPerformanceHistory)
        return;

    m_performanceHistory.PushBack(snapshot);

    // Check for slow frames
    if (snapshot.totalFrameTime > m_settings.slowFrameThreshold)
    {
        m_slowFrameCount++;

        if (m_settings.autoCaptureSlowFrames)
        {
            AutoCaptureSlowFrame(snapshot);
        }
    }
}

const PerformanceSnapshot* Debugger::GetLastPerformance() const
{
    if (m_performanceHistory.Empty())
        return nullptr;
    return &m_performanceHistory.Back();
}

const PerformanceSnapshot* Debugger::GetPerformanceHistory(uint32_t frameOffset) const
{
    if (frameOffset >= m_performanceHistory.Size())
        return nullptr;
    return &m_performanceHistory[m_performanceHistory.Size() - 1 - frameOffset];
}

Vector<const PerformanceSnapshot*> Debugger::GetPerformanceHistory() const
{
    Vector<const PerformanceSnapshot*> result;
    result.Reserve(m_performanceHistory.Size());

    for (size_type i = 0; i < m_performanceHistory.Size(); ++i)
    {
        result.PushBack(&m_performanceHistory[i]);
    }

    return result;
}

void Debugger::ClearPerformanceHistory()
{
    m_performanceHistory.Clear();
    m_slowFrameCount = 0;
}

// ============================================================================
// Debug Drawing
// ============================================================================

void Debugger::DrawPoint(DebugCategory category, const Vec3& pos,
                          const Vec3& color, float size, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::Point;
    cmd.category = category;
    cmd.position = pos;
    cmd.color = color;
    cmd.size = size;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    m_drawCommands.PushBack(cmd);
}

void Debugger::DrawLine(DebugCategory category, const Vec3& from, const Vec3& to,
                         const Vec3& color, float thickness, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::Line;
    cmd.category = category;
    cmd.position = from;
    cmd.endPosition = to;
    cmd.color = color;
    cmd.thickness = thickness;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    m_drawCommands.PushBack(cmd);
}

void Debugger::DrawArrow(DebugCategory category, const Vec3& from, const Vec3& to,
                          const Vec3& color, float thickness, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::Arrow;
    cmd.category = category;
    cmd.position = from;
    cmd.endPosition = to;
    cmd.color = color;
    cmd.thickness = thickness;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    m_drawCommands.PushBack(cmd);
}

void Debugger::DrawSphere(DebugCategory category, const Vec3& center, float radius,
                           const Vec3& color, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::Sphere;
    cmd.category = category;
    cmd.position = center;
    cmd.color = color;
    cmd.size = radius;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    m_drawCommands.PushBack(cmd);
}

void Debugger::DrawBox(DebugCategory category, const Vec3& center, const Vec3& extents,
                        const Vec3& color, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::Box;
    cmd.category = category;
    cmd.position = center;
    cmd.endPosition = extents;
    cmd.color = color;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    m_drawCommands.PushBack(cmd);
}

void Debugger::DrawText(DebugCategory category, const Vec3& pos, const String& text,
                         const Vec3& color, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::Text;
    cmd.category = category;
    cmd.position = pos;
    cmd.color = color;
    cmd.text = text;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    m_drawCommands.PushBack(cmd);
}

void Debugger::DrawTrajectory(DebugCategory category, const Trajectory& trajectory,
                               const Vec3& color, float thickness, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::Trajectory;
    cmd.category = category;
    cmd.color = color;
    cmd.thickness = thickness;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    // Store trajectory data for rendering
    m_drawCommands.PushBack(cmd);
}

void Debugger::DrawSkeleton(DebugCategory category, const Pose& pose, const Vec3& color,
                             float boneThickness, float duration, bool persistent)
{
    if (!m_settings.enabled)
        return;

    DebugDrawCommand cmd;
    cmd.type = DebugDrawType::PoseSkeleton;
    cmd.category = category;
    cmd.color = color;
    cmd.thickness = boneThickness;
    cmd.duration = duration;
    cmd.frameNumber = m_currentFrame;
    cmd.persistent = persistent;

    m_drawCommands.PushBack(cmd);
}

void Debugger::ClearDrawCommands()
{
    m_drawCommands.Clear();
}

void Debugger::ClearDrawCommands(DebugCategory category)
{
    for (int32 i = static_cast<int32>(m_drawCommands.Size()) - 1; i >= 0; --i)
    {
        if (m_drawCommands[i].category == category)
        {
            m_drawCommands.Erase(i);
        }
    }
}

Vector<DebugDrawCommand> Debugger::GetDrawCommands(DebugCategory category) const
{
    Vector<DebugDrawCommand> result;

    for (const auto& cmd : m_drawCommands)
    {
        if (cmd.category == category)
            result.PushBack(cmd);
    }

    return result;
}

// ============================================================================
// Visualization
// ============================================================================

void Debugger::VisualizeSearch(const SearchDebugInfo& info)
{
    // Draw query trajectory
    DrawTrajectory(DebugCategory::Search, info.queryTrajectory,
                   Vec3(1.0f, 0.0f, 0.0f), 3.0f, 0.0f, false);

    // Draw best match trajectory
    DrawTrajectory(DebugCategory::Search, info.bestMatchTrajectory,
                   Vec3(0.0f, 1.0f, 0.0f), 3.0f, 0.0f, false);

    // Draw search radius sphere
    if (m_settings.showSearchRadius)
    {
        DrawSphere(DebugCategory::Search, Vec3::Zero(), 2.0f,
                   Vec3(0.5f, 0.5f, 0.5f), 0.0f, false);
    }

    // Draw transition indicator
    if (info.transitionTriggered)
    {
        DrawText(DebugCategory::Search, Vec3(0, 2.5f, 0),
                 String::Format("Transition! Cost: %.3f", info.transitionCost),
                 Vec3(1.0f, 1.0f, 0.0f), 1.0f, false);
    }
}

void Debugger::VisualizeCosts(const CostDebugInfo& info)
{
    // Draw cost breakdown as bar chart
    Vector<float> costs;
    costs.PushBack(info.queryCost.poseCost);
    costs.PushBack(info.queryCost.trajectoryCost);
    costs.PushBack(info.queryCost.velocityCost);
    costs.PushBack(info.queryCost.headingCost);
    costs.PushBack(info.queryCost.phaseCost);
    costs.PushBack(info.queryCost.distanceCost);

    VisualizeBarChart(costs, Vec3(-3.0f, 0.5f, 0), Vec3::Right(), 0.3f, 0.1f);

    // Draw total cost text
    DrawText(DebugCategory::Cost, Vec3(-3.0f, 3.0f, 0),
             String::Format("Total: %.3f", info.queryCost.totalCost),
             Vec3(1.0f, 1.0f, 1.0f), 0.0f, false);
}

void Debugger::VisualizeTrajectory(const TrajectoryDebugInfo& info)
{
    // Draw desired trajectory
    DrawTrajectory(DebugCategory::Trajectory, info.desired,
                   Vec3(0.0f, 1.0f, 0.0f), 3.0f, 0.0f, false);

    // Draw actual trajectory
    DrawTrajectory(DebugCategory::Trajectory, info.actual,
                   Vec3(1.0f, 0.0f, 0.0f), 3.0f, 0.0f, false);

    // Draw error indicators
    for (int32 i = 0; i < info.desired.sampleCount && i < info.actual.sampleCount; ++i)
    {
        if (info.desired.points[i].isValid && info.actual.points[i].isValid)
        {
            Vec3 error = info.actual.points[i].position - info.desired.points[i].position;
            float errorLen = error.Length();

            Vec3 color = errorLen > 0.5f ? Vec3(1.0f, 0.0f, 0.0f) :
                         errorLen > 0.2f ? Vec3(1.0f, 1.0f, 0.0f) :
                         Vec3(0.0f, 1.0f, 0.0f);

            DrawLine(DebugCategory::Trajectory,
                     info.desired.points[i].position,
                     info.actual.points[i].position,
                     color, 2.0f, 0.0f, false);
        }
    }
}

void Debugger::VisualizePose(const Pose& pose, const Vec3& offset)
{
    if (m_settings.showSkeleton)
    {
        DrawSkeleton(DebugCategory::Pose, pose, Vec3(0.0f, 1.0f, 0.0f), 2.0f, 0.0f, false);
    }

    if (m_settings.showBoneLabels)
    {
        for (int32 i = 0; i < pose.boneCount; ++i)
        {
            DrawText(DebugCategory::Pose,
                     pose.worldPositions[i] + offset + Vec3(0, 0.1f, 0),
                     String::Format("Bone %d", i),
                     Vec3(1.0f, 1.0f, 1.0f), 0.0f, false);
        }
    }

    if (m_settings.showVelocityVectors)
    {
        // TODO: Draw velocity vectors for each bone
    }
}

void Debugger::VisualizeIK(const Vector<Vec3>& targets, const Vector<Vec3>& results)
{
    for (size_type i = 0; i < targets.Size() && i < results.Size(); ++i)
    {
        // Draw target
        DrawSphere(DebugCategory::IK, targets[i], 0.05f,
                   Vec3(1.0f, 0.0f, 0.0f), 0.0f, false);

        // Draw result
        DrawSphere(DebugCategory::IK, results[i], 0.05f,
                   Vec3(0.0f, 1.0f, 0.0f), 0.0f, false);

        // Draw line connecting them
        DrawLine(DebugCategory::IK, targets[i], results[i],
                 Vec3(1.0f, 1.0f, 0.0f), 2.0f, 0.0f, false);
    }
}

void Debugger::VisualizeHeatmap(const Vector<float>& values, const Vector<Vec3>& positions)
{
    if (values.Size() != positions.Size())
        return;

    float maxVal = 0.0f;
    for (const auto& v : values)
        maxVal = Math::Max(maxVal, v);

    if (maxVal < 1e-6f)
        return;

    for (size_type i = 0; i < values.Size(); ++i)
    {
        float t = values[i] / maxVal;
        Vec3 color = Vec3(t, 1.0f - t, 0.0f); // Green to red

        DrawSphere(DebugCategory::General, positions[i], 0.1f,
                   color, 0.0f, false);
    }
}

void Debugger::VisualizeBarChart(const Vector<float>& values, const Vec3& origin,
                                  const Vec3& axis, float barWidth, float spacing)
{
    float maxVal = 0.0f;
    for (const auto& v : values)
        maxVal = Math::Max(maxVal, v);

    if (maxVal < 1e-6f)
        maxVal = 1.0f;

    Vec3 perpAxis = axis.Cross(Vec3::Up()).Normalized();
    if (perpAxis.LengthSq() < 0.001f)
        perpAxis = Vec3::Right();

    for (size_type i = 0; i < values.Size(); ++i)
    {
        float height = values[i] / maxVal * 2.0f;
        Vec3 barOrigin = origin + perpAxis * (i * (barWidth + spacing));
        Vec3 barExtents = Vec3(barWidth * 0.5f, height * 0.5f, barWidth * 0.5f);
        Vec3 barCenter = barOrigin + Vec3::Up() * (height * 0.5f);

        Vec3 color = Vec3(values[i] / maxVal, 1.0f - values[i] / maxVal, 0.0f);

        DrawBox(DebugCategory::General, barCenter, barExtents, color, 0.0f, false);
    }
}

void Debugger::VisualizeLineGraph(const Vector<float>& values, const Vec3& origin,
                                   const Vec3& axis, float scale)
{
    if (values.Size() < 2)
        return;

    float maxVal = 0.0f;
    for (const auto& v : values)
        maxVal = Math::Max(maxVal, v);

    if (maxVal < 1e-6f)
        maxVal = 1.0f;

    float step = scale / static_cast<float>(values.Size() - 1);

    for (size_type i = 0; i < values.Size() - 1; ++i)
    {
        Vec3 p0 = origin + axis * (i * step) +
                  Vec3::Up() * (values[i] / maxVal * scale);
        Vec3 p1 = origin + axis * ((i + 1) * step) +
                  Vec3::Up() * (values[i + 1] / maxVal * scale);

        DrawLine(DebugCategory::General, p0, p1, Vec3(0.0f, 1.0f, 1.0f), 2.0f, 0.0f, false);
    }
}

// ============================================================================
// Analysis
// ============================================================================

float Debugger::GetAverageSearchTime() const
{
    if (m_searchCount == 0)
        return 0.0f;
    return m_totalSearchTime / static_cast<float>(m_searchCount);
}

float Debugger::GetMaxSearchTime() const
{
    return m_maxSearchTime;
}

float Debugger::GetMinSearchTime() const
{
    return m_searchCount > 0 ? m_minSearchTime : 0.0f;
}

float Debugger::GetSearchTimeStdDev() const
{
    if (m_searchCount < 2)
        return 0.0f;

    float avg = GetAverageSearchTime();
    float sumSq = 0.0f;

    for (size_type i = 0; i < m_searchHistory.Size(); ++i)
    {
        float diff = m_searchHistory[i].searchTimeMs - avg;
        sumSq += diff * diff;
    }

    return std::sqrt(sumSq / static_cast<float>(m_searchCount));
}

uint32_t Debugger::GetSlowFrameCount() const
{
    return m_slowFrameCount;
}

float Debugger::GetAverageFPS() const
{
    if (m_performanceHistory.Empty())
        return 0.0f;

    float totalFps = 0.0f;
    for (size_type i = 0; i < m_performanceHistory.Size(); ++i)
    {
        totalFps += m_performanceHistory[i].fps;
    }

    return totalFps / static_cast<float>(m_performanceHistory.Size());
}

String Debugger::GenerateReport() const
{
    String report;
    report += "=== MMV2 Debugger Report ===\n\n";

    report += "--- Search Statistics ---\n";
    report += String::Format("Total Searches: %u\n", m_searchCount);
    report += String::Format("Average Time: %.3f ms\n", GetAverageSearchTime());
    report += String::Format("Min Time: %.3f ms\n", GetMinSearchTime());
    report += String::Format("Max Time: %.3f ms\n", GetMaxSearchTime());
    report += String::Format("Std Dev: %.3f ms\n", GetSearchTimeStdDev());
    report += "\n";

    report += "--- Performance Statistics ---\n";
    report += String::Format("Slow Frames: %u\n", m_slowFrameCount);
    report += String::Format("Average FPS: %.1f\n", GetAverageFPS());
    report += "\n";

    report += "--- History Sizes ---\n";
    report += String::Format("Search History: %zu\n", m_searchHistory.Size());
    report += String::Format("Cost History: %zu\n", m_costHistory.Size());
    report += String::Format("Trajectory History: %zu\n", m_trajectoryHistory.Size());
    report += String::Format("Performance History: %zu\n", m_performanceHistory.Size());
    report += "\n";

    if (!m_searchHistory.Empty())
    {
        const auto& last = m_searchHistory.Back();
        report += "--- Last Search ---\n";
        report += String::Format("Frame: %u\n", last.frameNumber);
        report += String::Format("Time: %.3f ms\n", last.searchTimeMs);
        report += String::Format("Poses Searched: %u\n", last.posesSearched);
        report += String::Format("Best Cost: %.3f\n", last.bestCost);
        report += String::Format("Algorithm: %s\n", last.searchAlgorithm.CStr());
    }

    return report;
}

// ============================================================================
// Export
// ============================================================================

bool Debugger::ExportSearchHistory(const String& filePath) const
{
    // TODO: Implement search history export (JSON/CSV)
    if (m_debugEnabled)
    {
        Log::Debug("Debugger: Exporting search history to %s", filePath.CStr());
    }
    return false;
}

bool Debugger::ExportCostHistory(const String& filePath) const
{
    // TODO: Implement cost history export
    if (m_debugEnabled)
    {
        Log::Debug("Debugger: Exporting cost history to %s", filePath.CStr());
    }
    return false;
}

bool Debugger::ExportTrajectoryHistory(const String& filePath) const
{
    // TODO: Implement trajectory history export
    if (m_debugEnabled)
    {
        Log::Debug("Debugger: Exporting trajectory history to %s", filePath.CStr());
    }
    return false;
}

bool Debugger::ExportPerformanceHistory(const String& filePath) const
{
    // TODO: Implement performance history export
    if (m_debugEnabled)
    {
        Log::Debug("Debugger: Exporting performance history to %s", filePath.CStr());
    }
    return false;
}

bool Debugger::ExportFullReport(const String& filePath) const
{
    String report = GenerateReport();
    // TODO: Write report to file
    if (m_debugEnabled)
    {
        Log::Debug("Debugger: Exporting full report to %s", filePath.CStr());
    }
    return false;
}

// ============================================================================
// Comparison
// ============================================================================

void Debugger::BeginComparison()
{
    m_isComparing = true;
    m_comparisonFrames.Clear();
}

void Debugger::AddComparisonFrame(uint32_t frameNumber)
{
    if (m_isComparing)
    {
        m_comparisonFrames.PushBack(frameNumber);
    }
}

void Debugger::EndComparison()
{
    m_isComparing = false;
}

// ============================================================================
// Update
// ============================================================================

void Debugger::Update(float deltaTime)
{
    if (!m_settings.enabled)
        return;

    m_currentFrame++;
    UpdateDrawCommands(deltaTime);
}

void Debugger::UpdateDrawCommands(float deltaTime)
{
    for (int32 i = static_cast<int32>(m_drawCommands.Size()) - 1; i >= 0; --i)
    {
        if (!m_drawCommands[i].persistent)
        {
            m_drawCommands[i].duration -= deltaTime;
            if (m_drawCommands[i].duration <= 0.0f)
            {
                m_drawCommands.Erase(i);
            }
        }
    }
}

void Debugger::AutoCaptureSlowFrame(const PerformanceSnapshot& snapshot)
{
    if (m_settings.exportOnCapture && !m_settings.exportPath.IsEmpty())
    {
        String fileName = String::Format("slowframe_%u.txt", snapshot.frameNumber);
        String fullPath = m_settings.exportPath + "/" + fileName;
        ExportSnapshot(snapshot);
    }
}

void Debugger::ExportSnapshot(const PerformanceSnapshot& snapshot)
{
    // TODO: Implement snapshot export
    if (m_debugEnabled)
    {
        Log::Debug("Debugger: Auto-captured slow frame %u (%.2f ms)",
                   snapshot.frameNumber, snapshot.totalFrameTime);
    }
}

// ============================================================================
// Debug
// ============================================================================

void Debugger::SetDebugEnabled(bool enabled)
{
    m_debugEnabled = enabled;
}

String Debugger::GetDebugInfo() const
{
    String info;
    info += String::Format("Enabled: %s\n", m_settings.enabled ? "Yes" : "No");
    info += String::Format("Frame: %u\n", m_currentFrame);
    info += String::Format("Search History: %zu/%zu\n",
                           m_searchHistory.Size(), m_searchHistory.Capacity());
    info += String::Format("Cost History: %zu/%zu\n",
                           m_costHistory.Size(), m_costHistory.Capacity());
    info += String::Format("Trajectory History: %zu/%zu\n",
                           m_trajectoryHistory.Size(), m_trajectoryHistory.Capacity());
    info += String::Format("Performance History: %zu/%zu\n",
                           m_performanceHistory.Size(), m_performanceHistory.Capacity());
    info += String::Format("Draw Commands: %zu\n", m_drawCommands.Size());
    info += String::Format("Avg Search: %.3f ms\n", GetAverageSearchTime());
    info += String::Format("Slow Frames: %u\n", m_slowFrameCount);
    info += String::Format("Comparing: %s\n", m_isComparing ? "Yes" : "No");
    return info;
}

MMV2_NAMESPACE_END
