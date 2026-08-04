// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Asset Indexer Implementation
// ============================================================================

#include "MMV2/Asset/AssetIndexer.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include <algorithm>
#include <filesystem>

MMV2_NAMESPACE_BEGIN

AssetIndexer::AssetIndexer()
    : m_database(nullptr), m_isBuilding(false), m_cancelRequested(false) {}

AssetIndexer::~AssetIndexer() {
    delete m_database;
}

void AssetIndexer::AddSource(const AssetSource& source) {
    m_sources.PushBack(source);
}

void AssetIndexer::RemoveSource(int32 index) {
    if (index < 0 || index >= static_cast<int32>(m_sources.Size())) return;
    m_sources.Erase(m_sources.begin() + index);
}

void AssetIndexer::ClearSources() {
    m_sources.Clear();
}

void AssetIndexer::SetBuildSettings(const DatabaseBuildSettings& settings) {
    m_buildSettings = settings;
}

void AssetIndexer::SetOutputPath(const char* path) {
    m_outputPath = path;
}

bool AssetIndexer::Build() {
    if (m_isBuilding) return false;
    if (m_sources.Empty()) return false;

    m_isBuilding = true;
    m_cancelRequested = false;
    m_progress = IndexerProgress();
    m_progress.totalAssets = static_cast<int32>(m_sources.Size());

    delete m_database;
    m_database = new MotionDatabase();

    for (size_type i = 0; i < m_sources.Size(); ++i) {
        if (m_cancelRequested) break;

        m_progress.currentAsset = m_sources[i].name;
        m_progress.processedAssets = static_cast<int32>(i);
        m_progress.percentComplete = static_cast<float32>(i) / m_sources.Size() * 100.0f;

        if (!ProcessSource(m_sources[i])) {
            m_progress.hasError = true;
            m_progress.errorMessage = String::Format("Failed to process: %s", m_sources[i].path.CStr());
            m_isBuilding = false;
            return false;
        }
    }

    // Build database
    if (!m_database->Build(m_buildSettings)) {
        m_progress.hasError = true;
        m_progress.errorMessage = "Database build failed";
        m_isBuilding = false;
        return false;
    }

    m_progress.isComplete = true;
    m_progress.percentComplete = 100.0f;
    m_progress.processedAssets = m_progress.totalAssets;
    m_isBuilding = false;

    // Save if path is set
    if (!m_outputPath.Empty()) {
        SaveDatabase();
    }

    return true;
}

bool AssetIndexer::BuildAsync() {
    // Simplified async - in production, use std::async or thread pool
    return Build();
}

void AssetIndexer::CancelBuild() {
    m_cancelRequested = true;
}

bool AssetIndexer::SaveDatabase() const {
    if (!m_database || m_outputPath.Empty()) return false;
    return m_database->Save(m_outputPath.CStr());
}

bool AssetIndexer::ProcessSource(const AssetSource& source) {
    AnimationClip clip;
    if (!LoadAnimationFromPath(source.path.CStr(), clip)) return false;

    clip.name = source.name.IsEmpty() ? source.path : source.name;
    clip.frameRate = source.frameRate;
    clip.isLooping = source.isLooping;
    clip.isMirrored = source.isMirrored;
    clip.tags = source.tags;

    // Trim if needed
    if (source.startTime > 0.0f || source.endTime > 0.0f) {
        float32 start = source.startTime;
        float32 end = source.endTime > 0.0f ? source.endTime : clip.duration;
        int32 startFrame = clip.GetFrameAtTime(start);
        int32 endFrame = clip.GetFrameAtTime(end);
        startFrame = std::max(0, startFrame);
        endFrame = std::min(endFrame, clip.frameCount - 1);

        if (endFrame > startFrame) {
            Vector<Pose> trimmedPoses;
            trimmedPoses.Reserve(endFrame - startFrame);
            for (int32 f = startFrame; f < endFrame; ++f) {
                trimmedPoses.PushBack(clip.poses[f]);
            }
            clip.poses = std::move(trimmedPoses);
            clip.frameCount = static_cast<int32>(clip.poses.Size());
            clip.duration = clip.frameCount / clip.frameRate;
        }
    }

    m_database->AddClip(clip);
    m_progress.totalFrames += clip.frameCount;
    m_progress.processedFrames += clip.frameCount;

    return true;
}

bool AssetIndexer::LoadAnimationFromPath(const char* path, AnimationClip& outClip) {
    // Placeholder - in production, this would load FBX, BVH, etc.
    // For now, try to deserialize from our binary format
    BinarySerializer serializer;
    if (!serializer.OpenRead(path)) return false;

    // Read clip data
    serializer.Read(outClip.name);
    serializer.Read(outClip.sourcePath);
    serializer.Read(outClip.hash);
    serializer.Read(outClip.duration);
    serializer.Read(outClip.frameRate);
    serializer.Read(outClip.frameCount);
    serializer.Read(outClip.boneCount);
    serializer.Read(outClip.isLooping);
    serializer.Read(outClip.isMirrored);

    uint32_t poseCount;
    serializer.Read(poseCount);
    outClip.poses.Resize(poseCount);
    for (uint32_t i = 0; i < poseCount; ++i) {
        outClip.poses[i].Deserialize(serializer);
    }

    serializer.Close();
    return true;
}

void AssetIndexer::AddSourceDirectory(const char* dirPath, const char* extension) {
    // C++20 filesystem
    namespace fs = std::filesystem;
    if (!fs::exists(dirPath)) return;

    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.is_regular_file()) {
            String ext = entry.path().extension().string().c_str();
            ext.ToLower();
            if (ext == extension) {
                AssetSource source;
                source.path = entry.path().string().c_str();
                source.name = entry.path().stem().string().c_str();
                m_sources.PushBack(source);
            }
        }
    }
}

void AssetIndexer::AddSourceList(const Vector<String>& paths) {
    for (const auto& path : paths) {
        AssetSource source;
        source.path = path;
        source.name = path;
        m_sources.PushBack(source);
    }
}

MMV2_NAMESPACE_END
