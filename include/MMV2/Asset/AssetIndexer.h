// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Asset Indexer
// ============================================================================

#pragma once
#ifndef MMV2_ASSET_INDEXER_H
#define MMV2_ASSET_INDEXER_H

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/String.h"
#include "MMV2/Database/Database.h"

MMV2_NAMESPACE_BEGIN

struct AssetSource {
    String path;
    String name;
    float32 frameRate;
    bool isLooping;
    bool isMirrored;
    BoneFlags tags;
    float32 startTime;
    float32 endTime;

    AssetSource()
        : frameRate(30.0f), isLooping(false), isMirrored(false), tags(BoneFlags::None),
          startTime(0.0f), endTime(-1.0f) {}
};

struct IndexerProgress {
    int32 totalAssets;
    int32 processedAssets;
    int32 totalFrames;
    int32 processedFrames;
    float32 percentComplete;
    String currentAsset;
    bool isComplete;
    bool hasError;
    String errorMessage;

    IndexerProgress()
        : totalAssets(0), processedAssets(0), totalFrames(0), processedFrames(0),
          percentComplete(0.0f), isComplete(false), hasError(false) {}
};

class MMV2_API AssetIndexer {
public:
    AssetIndexer();
    ~AssetIndexer();

    void AddSource(const AssetSource& source);
    void RemoveSource(int32 index);
    void ClearSources();

    void SetBuildSettings(const DatabaseBuildSettings& settings);
    void SetOutputPath(const char* path);

    bool Build();
    bool BuildAsync();
    void CancelBuild();

    bool IsBuilding() const { return m_isBuilding; }
    const IndexerProgress& GetProgress() const { return m_progress; }

    MotionDatabase* GetDatabase() const { return m_database; }
    bool SaveDatabase() const;

    // Batch processing
    void AddSourceDirectory(const char* dirPath, const char* extension);
    void AddSourceList(const Vector<String>& paths);

private:
    Vector<AssetSource> m_sources;
    DatabaseBuildSettings m_buildSettings;
    String m_outputPath;
    MotionDatabase* m_database;
    bool m_isBuilding;
    bool m_cancelRequested;
    IndexerProgress m_progress;

    bool ProcessSource(const AssetSource& source);
    bool LoadAnimationFromPath(const char* path, AnimationClip& outClip);
};

MMV2_NAMESPACE_END

#endif
