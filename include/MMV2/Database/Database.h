#pragma once
#ifndef MMV2_DATABASE_H
#define MMV2_DATABASE_H

#include "Config.h"
#include "Pose.h"
#include "Feature.h"
#include "Trajectory.h"
#include "Bone.h"
#include "Vector.h"
#include "String.h"
#include "HashMap.h"
#include <atomic>

MMV2_NAMESPACE_BEGIN

struct AnimationClip {
    String name;
    String sourcePath;
    uint32 hash;
    float32 duration;
    float32 frameRate;
    int32 frameCount;
    int32 boneCount;
    bool isLooping;
    bool isMirrored;
    BoneFlags tags;
    Vector<Pose> poses;
    Vector<PoseVelocity> velocities;
    Vector<Trajectory> trajectories;
    Vector<FeatureVector> features;
    Vector<float32> contactStates;
    Vector<int32> eventMarkers;

    AnimationClip() : hash(0), duration(0.0f), frameRate(30.0f), frameCount(0),
                      boneCount(0), isLooping(false), isMirrored(false), tags(BoneFlags::None) {}

    float32 GetTimeAtFrame(int32 frame) const { return frame / frameRate; }
    int32 GetFrameAtTime(float32 time) const { return static_cast<int32>(time * frameRate); }
    const Pose* GetPoseAtTime(float32 time) const;
    const PoseVelocity* GetVelocityAtTime(float32 time) const;
};

struct DatabaseEntry {
    int32 clipIndex;
    int32 frameIndex;
    float32 time;
    FeatureVector feature;
    Pose pose;
    PoseVelocity velocity;
    Trajectory trajectory;
    float32 phase;
    float32 contactLeft;
    float32 contactRight;
    bool isValid;

    DatabaseEntry() : clipIndex(-1), frameIndex(-1), time(0.0f), phase(0.0f),
                      contactLeft(0.0f), contactRight(0.0f), isValid(false) {}
};

enum class DatabaseCompressionType : uint8 {
    None = 0,
    Quantization = 1,
    PCA = 2,
    AutoEncoder = 3,
    Custom = 4
};

struct DatabaseCompressionSettings {
    DatabaseCompressionType type;
    int32 targetDimensions;
    float32 quality;
    bool enableQuantization;
    int32 quantizationBits;
    bool enableDeltaEncoding;

    DatabaseCompressionSettings()
        : type(DatabaseCompressionType::None), targetDimensions(0),
          quality(0.95f), enableQuantization(false), quantizationBits(16),
          enableDeltaEncoding(false) {}
};

struct DatabaseBuildSettings {
    FeatureSchema featureSchema;
    TrajectoryGenerator trajectoryGenerator;
    DatabaseCompressionSettings compression;
    bool computeVelocities;
    bool computeTrajectories;
    bool computeFeatures;
    bool normalizeFeatures;
    bool buildSearchStructures;
    bool enableMirroring;
    float32 poseSampleRate;
    float32 minClipDuration;
    int32 minClipFrames;

    DatabaseBuildSettings()
        : computeVelocities(true), computeTrajectories(true), computeFeatures(true),
          normalizeFeatures(true), buildSearchStructures(true), enableMirroring(false),
          poseSampleRate(30.0f), minClipDuration(0.5f), minClipFrames(15) {}
};

struct DatabaseStats {
    int32 totalEntries;
    int32 totalClips;
    int32 totalFrames;
    int32 totalFeatures;
    size_type memoryUsage;
    size_type compressedSize;
    float32 compressionRatio;
    float32 avgFeatureDistance;
    float32 maxFeatureDistance;
    float32 buildTime;
    float32 avgSearchTime;
    float32 minSearchTime;
    float32 maxSearchTime;

    DatabaseStats()
        : totalEntries(0), totalClips(0), totalFrames(0), totalFeatures(0),
          memoryUsage(0), compressedSize(0), compressionRatio(1.0f),
          avgFeatureDistance(0.0f), maxFeatureDistance(0.0f),
          buildTime(0.0f), avgSearchTime(0.0f), minSearchTime(0.0f), maxSearchTime(0.0f) {}
};

class MMV2_API MotionDatabase {
public:
    MotionDatabase();
    ~MotionDatabase();

    // Building
    bool AddClip(const AnimationClip& clip);
    bool AddClips(Span<const AnimationClip> clips);
    bool Build(const DatabaseBuildSettings& settings);
    bool Rebuild();
    void Clear();

    // Query
    const DatabaseEntry* GetEntry(int32 index) const;
    int32 GetEntryCount() const { return static_cast<int32>(m_entries.Size()); }
    const AnimationClip* GetClip(int32 index) const;
    int32 GetClipCount() const { return static_cast<int32>(m_clips.Size()); }
    const BoneHierarchy& GetSkeleton() const { return m_skeleton; }
    const FeatureSchema& GetFeatureSchema() const { return m_featureSchema; }
    const DatabaseStats& GetStats() const { return m_stats; }

    // Feature queries
    FeatureVector ExtractFeatures(const Pose& pose, const PoseVelocity& velocity, const Trajectory& trajectory) const;
    void ExtractFeaturesBatch(Span<const Pose> poses, Span<const PoseVelocity> velocities,
                              Span<const Trajectory> trajectories, Vector<FeatureVector>& outFeatures) const;

    // Search
    int32 FindNearest(const FeatureVector& query, float32* outDistance = nullptr) const;
    void FindKNearest(const FeatureVector& query, int32 k, Vector<int32>& outIndices,
                      Vector<float32>* outDistances = nullptr) const;
    void FindInRadius(const FeatureVector& query, float32 radius, Vector<int32>& outIndices) const;

    // Traversal
    int32 GetNextFrame(int32 currentIndex, int32 step = 1) const;
    int32 GetPreviousFrame(int32 currentIndex, int32 step = 1) const;
    bool IsValidTransition(int32 fromIndex, int32 toIndex) const;

    // Serialization
    bool Save(const char* path) const;
    bool Load(const char* path);
    bool ExportToFormat(const char* path, const char* format) const;

    // Compression
    bool Compress(const DatabaseCompressionSettings& settings);
    bool Decompress();
    bool IsCompressed() const { return m_isCompressed; }

    // Validation
    bool Validate() const;
    void ComputeStats();

    // Metadata
    void SetName(const char* name) { m_name = name; }
    const String& GetName() const { return m_name; }
    void SetTag(const char* key, const char* value);
    const char* GetTag(const char* key) const;

private:
    String m_name;
    BoneHierarchy m_skeleton;
    Vector<AnimationClip> m_clips;
    Vector<DatabaseEntry> m_entries;
    FeatureSchema m_featureSchema;
    DatabaseBuildSettings m_buildSettings;
    DatabaseStats m_stats;
    HashMap<String, String> m_tags;
    bool m_isBuilt;
    bool m_isCompressed;
    std::atomic<bool> m_isBuilding;

    // Search structures (forward declarations)
    class KDTree* m_kdTree;
    class VPTree* m_vpTree;
    class ANNIndex* m_annIndex;

    bool BuildFeatures();
    bool BuildSearchStructures();
    bool ValidateEntry(const DatabaseEntry& entry) const;
    void ComputeEntryFeatures(DatabaseEntry& entry);
};

MMV2_NAMESPACE_END

#endif
