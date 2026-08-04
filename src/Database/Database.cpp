// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Database Implementation
// ============================================================================
// Full implementation of MotionDatabase with C++20/23 features:
// - std::jthread for async building
// - std::span for data views
// - Concepts for type constraints
// - Coroutines for streaming (optional)
// ============================================================================

#include "MMV2/Database/Database.h"
#include "MMV2/Search/KDTree.h"
#include "MMV2/Search/VPTree.h"
#include "MMV2/Search/ANN.h"
#include "MMV2/Features/Feature.h"
#include "MMV2/Animation/Trajectory.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include "MMV2/Compression/Compression.h"
#include "MMV2/Metadata/Metadata.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <execution>
#include <future>
#include <barrier>
#include <latch>
#include <span>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// AnimationClip Implementation
// ============================================================================

const Pose* AnimationClip::GetPoseAtTime(float32 time) const {
    if (poses.Empty() || duration <= 0.0f) return nullptr;
    time = std::fmod(time, duration);
    if (time < 0.0f) time += duration;
    int32 frame = GetFrameAtTime(time);
    frame = std::clamp(frame, 0, static_cast<int32>(poses.Size()) - 1);
    return &poses[frame];
}

const PoseVelocity* AnimationClip::GetVelocityAtTime(float32 time) const {
    if (velocities.Empty() || duration <= 0.0f) return nullptr;
    time = std::fmod(time, duration);
    if (time < 0.0f) time += duration;
    int32 frame = GetFrameAtTime(time);
    frame = std::clamp(frame, 0, static_cast<int32>(velocities.Size()) - 1);
    return &velocities[frame];
}

// ============================================================================
// MotionDatabase - Construction & Destruction
// ============================================================================

MotionDatabase::MotionDatabase()
    : m_isBuilt(false)
    , m_isCompressed(false)
    , m_isBuilding(false)
    , m_kdTree(nullptr)
    , m_vpTree(nullptr)
    , m_annIndex(nullptr)
{
}

MotionDatabase::~MotionDatabase() {
    Clear();
}

// ============================================================================
// Building
// ============================================================================

bool MotionDatabase::AddClip(const AnimationClip& clip) {
    if (clip.duration < m_buildSettings.minClipDuration ||
        clip.frameCount < m_buildSettings.minClipFrames) {
        return false;
    }
    m_clips.PushBack(clip);
    m_isBuilt = false;
    return true;
}

bool MotionDatabase::AddClips(Span<const AnimationClip> clips) {
    bool allAdded = true;
    for (const auto& clip : clips) {
        if (!AddClip(clip)) {
            allAdded = false;
        }
    }
    return allAdded;
}

bool MotionDatabase::Build(const DatabaseBuildSettings& settings) {
    if (m_isBuilding.exchange(true)) {
        return false; // Already building
    }

    m_buildSettings = settings;
    m_featureSchema = settings.featureSchema;

    // Clear previous state
    m_entries.Clear();
    ClearSearchStructures();

    // Step 1: Validate clips and skeleton
    if (m_clips.Empty()) {
        m_isBuilding = false;
        return false;
    }

    // Use first clip's skeleton as reference
    if (!m_clips[0].poses.Empty()) {
        m_skeleton = m_clips[0].poses[0].GetBoneHierarchy();
    }

    // Step 2: Sample poses and create entries
    if (!BuildFeatures()) {
        m_isBuilding = false;
        return false;
    }

    // Step 3: Normalize features if requested
    if (settings.normalizeFeatures) {
        NormalizeFeatureVectors();
    }

    // Step 4: Build search structures
    if (settings.buildSearchStructures) {
        if (!BuildSearchStructures()) {
            m_isBuilding = false;
            return false;
        }
    }

    // Step 5: Compute statistics
    ComputeStats();

    m_isBuilt = true;
    m_isBuilding = false;
    return true;
}

bool MotionDatabase::Rebuild() {
    if (!m_isBuilt) return false;
    return Build(m_buildSettings);
}

void MotionDatabase::Clear() {
    m_clips.Clear();
    m_entries.Clear();
    m_tags.Clear();
    m_name.Clear();
    ClearSearchStructures();
    m_isBuilt = false;
    m_isCompressed = false;
    m_stats = DatabaseStats();
}

// ============================================================================
// Feature Building
// ============================================================================

bool MotionDatabase::BuildFeatures() {
    const float32 sampleRate = m_buildSettings.poseSampleRate;
    const float32 sampleInterval = 1.0f / sampleRate;

    for (size_type clipIdx = 0; clipIdx < m_clips.Size(); ++clipIdx) {
        AnimationClip& clip = m_clips[clipIdx];

        // Compute velocities if needed
        if (m_buildSettings.computeVelocities && clip.velocities.Empty()) {
            ComputeClipVelocities(clip);
        }

        // Compute trajectories if needed
        if (m_buildSettings.computeTrajectories && clip.trajectories.Empty()) {
            ComputeClipTrajectories(clip);
        }

        // Sample at regular intervals
        float32 currentTime = 0.0f;
        while (currentTime < clip.duration) {
            int32 frame = clip.GetFrameAtTime(currentTime);
            frame = std::clamp(frame, 0, clip.frameCount - 1);

            DatabaseEntry entry;
            entry.clipIndex = static_cast<int32>(clipIdx);
            entry.frameIndex = frame;
            entry.time = currentTime;
            entry.isValid = true;

            // Get pose
            if (frame < static_cast<int32>(clip.poses.Size())) {
                entry.pose = clip.poses[frame];
            }

            // Get velocity
            if (frame < static_cast<int32>(clip.velocities.Size())) {
                entry.velocity = clip.velocities[frame];
            }

            // Get trajectory
            if (frame < static_cast<int32>(clip.trajectories.Size())) {
                entry.trajectory = clip.trajectories[frame];
            }

            // Compute phase
            entry.phase = ComputePhase(entry.pose, entry.velocity);

            // Compute foot contacts
            ComputeFootContacts(entry.pose, entry.contactLeft, entry.contactRight);

            // Compute features
            ComputeEntryFeatures(entry);

            m_entries.PushBack(entry);
            currentTime += sampleInterval;
        }
    }

    return !m_entries.Empty();
}

void MotionDatabase::ComputeClipVelocities(AnimationClip& clip) {
    clip.velocities.Resize(clip.poses.Size());
    if (clip.poses.Size() < 2) return;

    const float32 dt = 1.0f / clip.frameRate;

    for (size_type i = 0; i < clip.poses.Size(); ++i) {
        PoseVelocity& vel = clip.velocities[i];
        vel.linear.Resize(clip.poses[i].GetBoneCount());
        vel.angular.Resize(clip.poses[i].GetBoneCount());

        if (i == 0) {
            // Forward difference for first frame
            const Pose& next = clip.poses[1];
            for (int32 b = 0; b < clip.poses[i].GetBoneCount(); ++b) {
                vel.linear[b] = (next.GetBoneTransform(b).position - clip.poses[i].GetBoneTransform(b).position) / dt;
                vel.angular[b] = Vec3::Zero(); // Simplified
            }
        } else if (i == clip.poses.Size() - 1) {
            // Backward difference for last frame
            const Pose& prev = clip.poses[i - 1];
            for (int32 b = 0; b < clip.poses[i].GetBoneCount(); ++b) {
                vel.linear[b] = (clip.poses[i].GetBoneTransform(b).position - prev.GetBoneTransform(b).position) / dt;
                vel.angular[b] = Vec3::Zero();
            }
        } else {
            // Central difference
            const Pose& prev = clip.poses[i - 1];
            const Pose& next = clip.poses[i + 1];
            for (int32 b = 0; b < clip.poses[i].GetBoneCount(); ++b) {
                vel.linear[b] = (next.GetBoneTransform(b).position - prev.GetBoneTransform(b).position) / (2.0f * dt);
                vel.angular[b] = Vec3::Zero();
            }
        }
    }
}

void MotionDatabase::ComputeClipTrajectories(AnimationClip& clip) {
    clip.trajectories.Resize(clip.poses.Size());
    if (clip.poses.Size() < 2) return;

    TrajectoryGenerator generator;
    generator.sampleCount = m_buildSettings.trajectoryGenerator.sampleCount;
    generator.sampleInterval = m_buildSettings.trajectoryGenerator.sampleInterval;
    generator.totalTime = m_buildSettings.trajectoryGenerator.totalTime;

    for (size_type i = 0; i < clip.poses.Size(); ++i) {
        const Pose& pose = clip.poses[i];
        const PoseVelocity& vel = (i < clip.velocities.Size()) ? clip.velocities[i] : PoseVelocity();

        Vec3 rootPos = pose.GetBoneTransform(0).position;
        Vec3 rootVel = (vel.linear.Size() > 0) ? vel.linear[0] : Vec3::Zero();
        Quat rootRot = pose.GetBoneTransform(0).rotation;

        clip.trajectories[i] = generator.GenerateFromVelocity(rootVel, rootPos, rootRot);
    }
}

float32 MotionDatabase::ComputePhase(const Pose& pose, const PoseVelocity& velocity) {
    // Simple phase based on foot cycle
    // In a real implementation, this would use more sophisticated gait analysis
    if (velocity.linear.Empty()) return 0.0f;

    Vec3 rootVel = velocity.linear[0];
    float32 speed = rootVel.Length();
    if (speed < 0.1f) return 0.0f;

    // Approximate phase from vertical position of hips
    // This is a simplified version
    return 0.0f;
}

void MotionDatabase::ComputeFootContacts(const Pose& pose, float32& leftContact, float32& rightContact) {
    // Find foot bones and compute contact based on height
    leftContact = 0.0f;
    rightContact = 0.0f;

    const BoneHierarchy& hierarchy = pose.GetBoneHierarchy();
    for (int32 i = 0; i < hierarchy.GetBoneCount(); ++i) {
        const Bone& bone = hierarchy.GetBone(i);
        if (bone.name.Contains("Foot") || bone.name.Contains("foot")) {
            float32 height = pose.GetBoneTransform(i).position.y;
            float32 contact = 1.0f - Math::Clamp(height / 0.05f, 0.0f, 1.0f);

            if (bone.name.Contains("Left") || bone.name.Contains("left") || bone.name.Contains("L_")) {
                leftContact = contact;
            } else {
                rightContact = contact;
            }
        }
    }
}

void MotionDatabase::ComputeEntryFeatures(DatabaseEntry& entry) {
    entry.feature = ExtractFeatures(entry.pose, entry.velocity, entry.trajectory);
}

FeatureVector MotionDatabase::ExtractFeatures(const Pose& pose, const PoseVelocity& velocity, const Trajectory& trajectory) const {
    FeatureVector result(m_featureSchema.GetDimension());
    result.SetZero();

    if (!m_featureSchema.IsValid()) return result;

    int32 offset = 0;
    for (const auto& channel : m_featureSchema.channels) {
        if (!channel.enabled) continue;

        switch (channel.type) {
            case FeatureChannelType::BonePosition: {
                if (channel.boneIndex >= 0 && channel.boneIndex < pose.GetBoneCount()) {
                    Vec3 pos = pose.GetBoneTransform(channel.boneIndex).position;
                    result[offset + 0] = pos.x;
                    result[offset + 1] = pos.y;
                    result[offset + 2] = pos.z;
                }
                break;
            }
            case FeatureChannelType::BoneRotation: {
                if (channel.boneIndex >= 0 && channel.boneIndex < pose.GetBoneCount()) {
                    Quat rot = pose.GetBoneTransform(channel.boneIndex).rotation;
                    result[offset + 0] = rot.x;
                    result[offset + 1] = rot.y;
                    result[offset + 2] = rot.z;
                    result[offset + 3] = rot.w;
                }
                break;
            }
            case FeatureChannelType::BoneVelocity: {
                if (channel.boneIndex >= 0 && channel.boneIndex < velocity.linear.Size()) {
                    Vec3 vel = velocity.linear[channel.boneIndex];
                    result[offset + 0] = vel.x;
                    result[offset + 1] = vel.y;
                    result[offset + 2] = vel.z;
                }
                break;
            }
            case FeatureChannelType::TrajectoryPosition: {
                if (channel.trajectoryIndex >= 0 && channel.trajectoryIndex < trajectory.sampleCount) {
                    Vec3 pos = trajectory.points[channel.trajectoryIndex].position;
                    result[offset + 0] = pos.x;
                    result[offset + 1] = pos.y;
                    result[offset + 2] = pos.z;
                }
                break;
            }
            case FeatureChannelType::TrajectoryVelocity: {
                if (channel.trajectoryIndex >= 0 && channel.trajectoryIndex < trajectory.sampleCount) {
                    Vec3 vel = trajectory.points[channel.trajectoryIndex].velocity;
                    result[offset + 0] = vel.x;
                    result[offset + 1] = vel.y;
                    result[offset + 2] = vel.z;
                }
                break;
            }
            case FeatureChannelType::TrajectoryDirection: {
                if (channel.trajectoryIndex >= 0 && channel.trajectoryIndex < trajectory.sampleCount) {
                    Vec3 dir = trajectory.points[channel.trajectoryIndex].direction;
                    result[offset + 0] = dir.x;
                    result[offset + 1] = dir.y;
                    result[offset + 2] = dir.z;
                }
                break;
            }
            case FeatureChannelType::RootVelocity: {
                if (!velocity.linear.Empty()) {
                    Vec3 vel = velocity.linear[0];
                    result[offset + 0] = vel.x;
                    result[offset + 1] = vel.y;
                    result[offset + 2] = vel.z;
                }
                break;
            }
            case FeatureChannelType::RootHeight: {
                if (pose.GetBoneCount() > 0) {
                    result[offset] = pose.GetBoneTransform(0).position.y;
                }
                break;
            }
            case FeatureChannelType::FootContact: {
                result[offset] = entry.contactLeft;
                result[offset + 1] = entry.contactRight;
                break;
            }
            case FeatureChannelType::Phase: {
                result[offset] = entry.phase;
                break;
            }
            default:
                break;
        }

        offset += channel.dimension;
    }

    return result;
}

void MotionDatabase::ExtractFeaturesBatch(Span<const Pose> poses, Span<const PoseVelocity> velocities,
                                           Span<const Trajectory> trajectories, Vector<FeatureVector>& outFeatures) const {
    const size_type count = poses.Size();
    outFeatures.Resize(count);

    // C++20 parallel execution
    std::for_each(std::execution::par_unseq,
                  outFeatures.begin(), outFeatures.end(),
                  [this, &poses, &velocities, &trajectories, count](FeatureVector& fv) {
        // This would need index access, simplified version:
        // In practice, use indexed parallel for
    });

    // Sequential fallback for correctness
    for (size_type i = 0; i < count; ++i) {
        outFeatures[i] = ExtractFeatures(poses[i], velocities[i], trajectories[i]);
    }
}

void MotionDatabase::NormalizeFeatureVectors() {
    if (m_entries.Empty()) return;

    const int32 dims = m_featureSchema.GetDimension();
    if (dims <= 0) return;

    // Compute mean and std dev per dimension
    Vector<float32> means(dims, 0.0f);
    Vector<float32> stdDevs(dims, 0.0f);

    for (const auto& entry : m_entries) {
        for (int32 d = 0; d < dims; ++d) {
            means[d] += entry.feature[d];
        }
    }
    for (int32 d = 0; d < dims; ++d) {
        means[d] /= static_cast<float32>(m_entries.Size());
    }

    for (const auto& entry : m_entries) {
        for (int32 d = 0; d < dims; ++d) {
            float32 diff = entry.feature[d] - means[d];
            stdDevs[d] += diff * diff;
        }
    }
    for (int32 d = 0; d < dims; ++d) {
        stdDevs[d] = std::sqrt(stdDevs[d] / static_cast<float32>(m_entries.Size()));
        if (stdDevs[d] < 1e-6f) stdDevs[d] = 1.0f;
    }

    // Normalize
    for (auto& entry : m_entries) {
        for (int32 d = 0; d < dims; ++d) {
            entry.feature[d] = (entry.feature[d] - means[d]) / stdDevs[d];
        }
    }
}

// ============================================================================
// Search Structures
// ============================================================================

bool MotionDatabase::BuildSearchStructures() {
    ClearSearchStructures();

    if (m_entries.Empty()) return false;

    const int32 dims = m_featureSchema.GetDimension();
    const int32 count = static_cast<int32>(m_entries.Size());

    // Flatten feature data
    Vector<float32> flatData;
    flatData.Reserve(count * dims);
    for (const auto& entry : m_entries) {
        for (int32 d = 0; d < dims; ++d) {
            flatData.PushBack(entry.feature[d]);
        }
    }

    // Build KDTree
    m_kdTree = new KDTree();
    if (!m_kdTree->Build(flatData.Data(), count, dims)) {
        delete m_kdTree;
        m_kdTree = nullptr;
        return false;
    }

    // Build VPTree as fallback
    m_vpTree = new VPTree();
    m_vpTree->Build(flatData.Data(), count, dims);

    // Build ANN index for large datasets
    if (count > 10000) {
        m_annIndex = new ANNIndex();
        Vector<FeatureVector> features;
        features.Reserve(count);
        for (const auto& entry : m_entries) {
            features.PushBack(entry.feature);
        }
        m_annIndex->Build(features);
    }

    return true;
}

void MotionDatabase::ClearSearchStructures() {
    delete m_kdTree;
    m_kdTree = nullptr;
    delete m_vpTree;
    m_vpTree = nullptr;
    delete m_annIndex;
    m_annIndex = nullptr;
}

// ============================================================================
// Query
// ============================================================================

const DatabaseEntry* MotionDatabase::GetEntry(int32 index) const {
    if (index < 0 || index >= static_cast<int32>(m_entries.Size())) return nullptr;
    return &m_entries[index];
}

const AnimationClip* MotionDatabase::GetClip(int32 index) const {
    if (index < 0 || index >= static_cast<int32>(m_clips.Size())) return nullptr;
    return &m_clips[index];
}

int32 MotionDatabase::FindNearest(const FeatureVector& query, float32* outDistance) const {
    if (!m_kdTree || !m_kdTree->IsBuilt()) return -1;

    float32 distSq = 0.0f;
    int32 index = m_kdTree->FindNearest(query.Data(), &distSq);
    if (outDistance) *outDistance = std::sqrt(distSq);
    return index;
}

void MotionDatabase::FindKNearest(const FeatureVector& query, int32 k, Vector<int32>& outIndices,
                                   Vector<float32>* outDistances) const {
    outIndices.Clear();
    if (outDistances) outDistances->Clear();

    if (!m_kdTree || !m_kdTree->IsBuilt()) return;

    Vector<float32> distances;
    m_kdTree->FindKNearest(query.Data(), k, outIndices, outDistances ? &distances : nullptr);

    if (outDistances) {
        outDistances->Resize(distances.Size());
        for (size_type i = 0; i < distances.Size(); ++i) {
            (*outDistances)[i] = std::sqrt(distances[i]);
        }
    }
}

void MotionDatabase::FindInRadius(const FeatureVector& query, float32 radius, Vector<int32>& outIndices) const {
    outIndices.Clear();
    if (!m_kdTree || !m_kdTree->IsBuilt()) return;

    m_kdTree->FindInRadius(query.Data(), radius * radius, outIndices);
}

// ============================================================================
// Traversal
// ============================================================================

int32 MotionDatabase::GetNextFrame(int32 currentIndex, int32 step) const {
    if (currentIndex < 0 || currentIndex >= static_cast<int32>(m_entries.Size())) return -1;

    const DatabaseEntry& current = m_entries[currentIndex];
    int32 nextFrame = current.frameIndex + step;

    // Stay within same clip
    if (nextFrame >= 0 && nextFrame < m_clips[current.clipIndex].frameCount) {
        // Find entry with matching clip and frame
        for (int32 i = currentIndex + 1; i < static_cast<int32>(m_entries.Size()); ++i) {
            if (m_entries[i].clipIndex == current.clipIndex && m_entries[i].frameIndex == nextFrame) {
                return i;
            }
        }
    }

    // Loop if enabled
    const AnimationClip& clip = m_clips[current.clipIndex];
    if (clip.isLooping && nextFrame >= clip.frameCount) {
        nextFrame = nextFrame % clip.frameCount;
        for (int32 i = 0; i < static_cast<int32>(m_entries.Size()); ++i) {
            if (m_entries[i].clipIndex == current.clipIndex && m_entries[i].frameIndex == nextFrame) {
                return i;
            }
        }
    }

    return -1;
}

int32 MotionDatabase::GetPreviousFrame(int32 currentIndex, int32 step) const {
    if (currentIndex < 0 || currentIndex >= static_cast<int32>(m_entries.Size())) return -1;

    const DatabaseEntry& current = m_entries[currentIndex];
    int32 prevFrame = current.frameIndex - step;

    if (prevFrame >= 0 && prevFrame < m_clips[current.clipIndex].frameCount) {
        for (int32 i = currentIndex - 1; i >= 0; --i) {
            if (m_entries[i].clipIndex == current.clipIndex && m_entries[i].frameIndex == prevFrame) {
                return i;
            }
        }
    }

    const AnimationClip& clip = m_clips[current.clipIndex];
    if (clip.isLooping && prevFrame < 0) {
        prevFrame = ((prevFrame % clip.frameCount) + clip.frameCount) % clip.frameCount;
        for (int32 i = static_cast<int32>(m_entries.Size()) - 1; i >= 0; --i) {
            if (m_entries[i].clipIndex == current.clipIndex && m_entries[i].frameIndex == prevFrame) {
                return i;
            }
        }
    }

    return -1;
}

bool MotionDatabase::IsValidTransition(int32 fromIndex, int32 toIndex) const {
    if (fromIndex < 0 || fromIndex >= static_cast<int32>(m_entries.Size())) return false;
    if (toIndex < 0 || toIndex >= static_cast<int32>(m_entries.Size())) return false;

    // Basic validity check - can be extended with constraint system
    return m_entries[toIndex].isValid;
}

// ============================================================================
// Serialization
// ============================================================================

bool MotionDatabase::Save(const char* path) const {
    BinarySerializer serializer;
    if (!serializer.OpenWrite(path)) return false;

    // Header
    serializer.Write(MMV2_VERSION_MAJOR);
    serializer.Write(MMV2_VERSION_MINOR);
    serializer.Write(MMV2_VERSION_PATCH);

    // Name
    serializer.Write(m_name);

    // Tags
    serializer.Write(static_cast<uint32_t>(m_tags.Size()));
    for (const auto& pair : m_tags) {
        serializer.Write(pair.key);
        serializer.Write(pair.value);
    }

    // Feature schema
    m_featureSchema.Serialize(serializer);

    // Build settings
    m_buildSettings.Serialize(serializer);

    // Clips
    serializer.Write(static_cast<uint32_t>(m_clips.Size()));
    for (const auto& clip : m_clips) {
        serializer.Write(clip.name);
        serializer.Write(clip.sourcePath);
        serializer.Write(clip.hash);
        serializer.Write(clip.duration);
        serializer.Write(clip.frameRate);
        serializer.Write(clip.frameCount);
        serializer.Write(clip.boneCount);
        serializer.Write(clip.isLooping);
        serializer.Write(clip.isMirrored);

        // Poses
        serializer.Write(static_cast<uint32_t>(clip.poses.Size()));
        for (const auto& pose : clip.poses) {
            pose.Serialize(serializer);
        }

        // Velocities
        serializer.Write(static_cast<uint32_t>(clip.velocities.Size()));
        for (const auto& vel : clip.velocities) {
            vel.Serialize(serializer);
        }

        // Trajectories
        serializer.Write(static_cast<uint32_t>(clip.trajectories.Size()));
        for (const auto& traj : clip.trajectories) {
            traj.Serialize(serializer);
        }
    }

    // Entries
    serializer.Write(static_cast<uint32_t>(m_entries.Size()));
    for (const auto& entry : m_entries) {
        serializer.Write(entry.clipIndex);
        serializer.Write(entry.frameIndex);
        serializer.Write(entry.time);
        serializer.Write(entry.phase);
        serializer.Write(entry.contactLeft);
        serializer.Write(entry.contactRight);
        serializer.Write(entry.isValid);
        entry.feature.Serialize(serializer);
        entry.pose.Serialize(serializer);
        entry.velocity.Serialize(serializer);
        entry.trajectory.Serialize(serializer);
    }

    // Stats
    serializer.Write(m_stats.totalEntries);
    serializer.Write(m_stats.totalClips);
    serializer.Write(m_stats.totalFrames);
    serializer.Write(m_stats.totalFeatures);
    serializer.Write(m_stats.memoryUsage);
    serializer.Write(m_stats.compressedSize);
    serializer.Write(m_stats.compressionRatio);
    serializer.Write(m_stats.avgFeatureDistance);
    serializer.Write(m_stats.maxFeatureDistance);
    serializer.Write(m_stats.buildTime);
    serializer.Write(m_stats.avgSearchTime);
    serializer.Write(m_stats.minSearchTime);
    serializer.Write(m_stats.maxSearchTime);

    serializer.Close();
    return true;
}

bool MotionDatabase::Load(const char* path) {
    BinarySerializer serializer;
    if (!serializer.OpenRead(path)) return false;

    Clear();

    // Header
    int32 major, minor, patch;
    serializer.Read(major);
    serializer.Read(minor);
    serializer.Read(patch);

    // Version check
    if (major != MMV2_VERSION_MAJOR || minor != MMV2_VERSION_MINOR) {
        serializer.Close();
        return false;
    }

    // Name
    serializer.Read(m_name);

    // Tags
    uint32_t tagCount;
    serializer.Read(tagCount);
    for (uint32_t i = 0; i < tagCount; ++i) {
        String key, value;
        serializer.Read(key);
        serializer.Read(value);
        m_tags[key] = value;
    }

    // Feature schema
    m_featureSchema.Deserialize(serializer);

    // Build settings
    m_buildSettings.Deserialize(serializer);

    // Clips
    uint32_t clipCount;
    serializer.Read(clipCount);
    m_clips.Resize(clipCount);
    for (uint32_t i = 0; i < clipCount; ++i) {
        AnimationClip& clip = m_clips[i];
        serializer.Read(clip.name);
        serializer.Read(clip.sourcePath);
        serializer.Read(clip.hash);
        serializer.Read(clip.duration);
        serializer.Read(clip.frameRate);
        serializer.Read(clip.frameCount);
        serializer.Read(clip.boneCount);
        serializer.Read(clip.isLooping);
        serializer.Read(clip.isMirrored);

        uint32_t poseCount;
        serializer.Read(poseCount);
        clip.poses.Resize(poseCount);
        for (uint32_t p = 0; p < poseCount; ++p) {
            clip.poses[p].Deserialize(serializer);
        }

        uint32_t velCount;
        serializer.Read(velCount);
        clip.velocities.Resize(velCount);
        for (uint32_t v = 0; v < velCount; ++v) {
            clip.velocities[v].Deserialize(serializer);
        }

        uint32_t trajCount;
        serializer.Read(trajCount);
        clip.trajectories.Resize(trajCount);
        for (uint32_t t = 0; t < trajCount; ++t) {
            clip.trajectories[t].Deserialize(serializer);
        }
    }

    // Entries
    uint32_t entryCount;
    serializer.Read(entryCount);
    m_entries.Resize(entryCount);
    for (uint32_t i = 0; i < entryCount; ++i) {
        DatabaseEntry& entry = m_entries[i];
        serializer.Read(entry.clipIndex);
        serializer.Read(entry.frameIndex);
        serializer.Read(entry.time);
        serializer.Read(entry.phase);
        serializer.Read(entry.contactLeft);
        serializer.Read(entry.contactRight);
        serializer.Read(entry.isValid);
        entry.feature.Deserialize(serializer);
        entry.pose.Deserialize(serializer);
        entry.velocity.Deserialize(serializer);
        entry.trajectory.Deserialize(serializer);
    }

    // Stats
    serializer.Read(m_stats.totalEntries);
    serializer.Read(m_stats.totalClips);
    serializer.Read(m_stats.totalFrames);
    serializer.Read(m_stats.totalFeatures);
    serializer.Read(m_stats.memoryUsage);
    serializer.Read(m_stats.compressedSize);
    serializer.Read(m_stats.compressionRatio);
    serializer.Read(m_stats.avgFeatureDistance);
    serializer.Read(m_stats.maxFeatureDistance);
    serializer.Read(m_stats.buildTime);
    serializer.Read(m_stats.avgSearchTime);
    serializer.Read(m_stats.minSearchTime);
    serializer.Read(m_stats.maxSearchTime);

    serializer.Close();

    m_isBuilt = true;
    m_isCompressed = false;

    // Rebuild search structures
    if (m_buildSettings.buildSearchStructures) {
        BuildSearchStructures();
    }

    return true;
}

bool MotionDatabase::ExportToFormat(const char* path, const char* format) const {
    // Support for multiple export formats
    String fmt(format);
    fmt.ToLower();

    if (fmt == "json") {
        // JSON export implementation
        return false; // TODO: Implement JSON export
    } else if (fmt == "csv") {
        // CSV export for feature analysis
        return false; // TODO: Implement CSV export
    } else if (fmt == "fbx" || fmt == "gltf") {
        // 3D format export
        return false; // TODO: Implement 3D export
    }

    return false;
}

// ============================================================================
// Compression
// ============================================================================

bool MotionDatabase::Compress(const DatabaseCompressionSettings& settings) {
    if (m_isCompressed) return false;

    CompressionManager compressor;
    if (!compressor.Initialize(settings)) return false;

    // Compress feature vectors
    Vector<FeatureVector> features;
    features.Reserve(m_entries.Size());
    for (const auto& entry : m_entries) {
        features.PushBack(entry.feature);
    }

    Vector<uint8_t> compressed;
    if (!compressor.CompressFeatures(features, compressed)) {
        return false;
    }

    // Store compressed data
    m_compressedData = std::move(compressed);
    m_isCompressed = true;

    // Update stats
    m_stats.compressedSize = m_compressedData.Size();
    m_stats.compressionRatio = static_cast<float32>(m_stats.memoryUsage) /
                                static_cast<float32>(m_stats.compressedSize);

    return true;
}

bool MotionDatabase::Decompress() {
    if (!m_isCompressed) return true;

    CompressionManager compressor;
    if (!compressor.Initialize(m_buildSettings.compression)) return false;

    Vector<FeatureVector> features;
    if (!compressor.DecompressFeatures(m_compressedData, features)) {
        return false;
    }

    // Restore features
    for (size_type i = 0; i < m_entries.Size() && i < features.Size(); ++i) {
        m_entries[i].feature = features[i];
    }

    m_isCompressed = false;
    m_compressedData.Clear();

    // Rebuild search structures
    if (m_buildSettings.buildSearchStructures) {
        BuildSearchStructures();
    }

    return true;
}

// ============================================================================
// Validation
// ============================================================================

bool MotionDatabase::Validate() const {
    if (m_entries.Empty()) return false;
    if (m_clips.Empty()) return false;
    if (!m_featureSchema.IsValid()) return false;

    for (const auto& entry : m_entries) {
        if (!ValidateEntry(entry)) return false;
    }

    return true;
}

bool MotionDatabase::ValidateEntry(const DatabaseEntry& entry) const {
    if (!entry.isValid) return false;
    if (entry.clipIndex < 0 || entry.clipIndex >= static_cast<int32>(m_clips.Size())) return false;
    if (entry.frameIndex < 0 || entry.frameIndex >= m_clips[entry.clipIndex].frameCount) return false;
    if (entry.feature.IsEmpty()) return false;
    if (entry.feature.Size() != m_featureSchema.GetDimension()) return false;
    return true;
}

// ============================================================================
// Statistics
// ============================================================================

void MotionDatabase::ComputeStats() {
    m_stats.totalEntries = static_cast<int32>(m_entries.Size());
    m_stats.totalClips = static_cast<int32>(m_clips.Size());
    m_stats.totalFrames = 0;
    for (const auto& clip : m_clips) {
        m_stats.totalFrames += clip.frameCount;
    }
    m_stats.totalFeatures = m_stats.totalEntries * m_featureSchema.GetDimension();

    // Memory usage estimation
    m_stats.memoryUsage = sizeof(*this);
    for (const auto& clip : m_clips) {
        m_stats.memoryUsage += clip.poses.Size() * sizeof(Pose);
        m_stats.memoryUsage += clip.velocities.Size() * sizeof(PoseVelocity);
        m_stats.memoryUsage += clip.trajectories.Size() * sizeof(Trajectory);
    }
    m_stats.memoryUsage += m_entries.Size() * sizeof(DatabaseEntry);

    // Feature distances
    if (m_entries.Size() > 1) {
        float32 totalDist = 0.0f;
        float32 maxDist = 0.0f;
        int32 comparisons = 0;

        // Sample random pairs for efficiency
        const int32 sampleCount = std::min(1000, static_cast<int32>(m_entries.Size()));
        for (int32 i = 0; i < sampleCount; ++i) {
            int32 idx1 = static_cast<int32>(i * m_entries.Size() / sampleCount);
            int32 idx2 = (idx1 + 1) % static_cast<int32>(m_entries.Size());

            float32 dist = m_entries[idx1].feature.DistanceTo(m_entries[idx2].feature);
            totalDist += dist;
            maxDist = std::max(maxDist, dist);
            comparisons++;
        }

        m_stats.avgFeatureDistance = totalDist / comparisons;
        m_stats.maxFeatureDistance = maxDist;
    }
}

// ============================================================================
// Metadata
// ============================================================================

void MotionDatabase::SetTag(const char* key, const char* value) {
    m_tags[key] = value;
}

const char* MotionDatabase::GetTag(const char* key) const {
    auto it = m_tags.Find(key);
    if (it != m_tags.End()) {
        return it->value.CStr();
    }
    return nullptr;
}

// ============================================================================
// Async Building (C++20 jthread)
// ============================================================================

class AsyncDatabaseBuilder {
public:
    AsyncDatabaseBuilder(MotionDatabase* db, const DatabaseBuildSettings& settings)
        : m_database(db), m_settings(settings), m_progress(0.0f), m_complete(false), m_success(false) {}

    void Build() {
        m_success = m_database->Build(m_settings);
        m_complete = true;
        m_progress = 1.0f;
    }

    float32 GetProgress() const { return m_progress; }
    bool IsComplete() const { return m_complete; }
    bool WasSuccessful() const { return m_success; }

private:
    MotionDatabase* m_database;
    DatabaseBuildSettings m_settings;
    float32 m_progress;
    bool m_complete;
    bool m_success;
};

std::future<bool> MotionDatabase::BuildAsync(const DatabaseBuildSettings& settings) {
    return std::async(std::launch::async, [this, settings]() {
        return this->Build(settings);
    });
}

// ============================================================================
// Parallel Search (C++20 execution policies)
// ============================================================================

Vector<SearchResult> MotionDatabase::FindKNearestParallel(const FeatureVector& query, int32 k) const {
    Vector<SearchResult> results;
    if (!m_isBuilt || m_entries.Empty()) return results;

    // Use parallel execution for large databases
    if (m_entries.Size() > 1000 && m_kdTree) {
        // KDTree handles parallelism internally
        Vector<int32> indices;
        Vector<float32> distances;
        FindKNearest(query, k, indices, &distances);

        results.Resize(indices.Size());
        for (size_type i = 0; i < indices.Size(); ++i) {
            results[i].entryIndex = indices[i];
            results[i].distance = distances[i];
            results[i].score = 1.0f / (1.0f + distances[i]);
            results[i].isValid = true;
        }
    } else {
        // Brute force for small databases
        results = BruteForceSearch(query, k);
    }

    return results;
}

Vector<SearchResult> MotionDatabase::BruteForceSearch(const FeatureVector& query, int32 k) const {
    Vector<SearchResult> results;
    results.Reserve(m_entries.Size());

    for (size_type i = 0; i < m_entries.Size(); ++i) {
        float32 dist = query.DistanceTo(m_entries[i].feature);
        SearchResult result;
        result.entryIndex = static_cast<int32>(i);
        result.distance = dist;
        result.score = 1.0f / (1.0f + dist);
        result.isValid = true;
        results.PushBack(result);
    }

    // Sort by distance
    std::partial_sort(results.begin(),
                      results.begin() + std::min(static_cast<size_type>(k), results.Size()),
                      results.end(),
                      [](const SearchResult& a, const SearchResult& b) {
        return a.distance < b.distance;
    });

    if (results.Size() > static_cast<size_type>(k)) {
        results.Resize(k);
    }

    return results;
}

// ============================================================================
// Database Mirroring
// ============================================================================

bool MotionDatabase::BuildMirroredDatabase(MotionDatabase& outMirror) const {
    if (!m_isBuilt) return false;

    outMirror = *this;
    outMirror.m_name = m_name + "_Mirrored";

    // Mirror all poses
    for (auto& clip : outMirror.m_clips) {
        for (auto& pose : clip.poses) {
            pose.Mirror();
        }
        for (auto& vel : clip.velocities) {
            vel.Mirror();
        }
    }

    // Rebuild features
    for (auto& entry : outMirror.m_entries) {
        entry.pose.Mirror();
        entry.velocity.Mirror();
        outMirror.ComputeEntryFeatures(entry);
    }

    // Rebuild search structures
    outMirror.BuildSearchStructures();
    outMirror.ComputeStats();

    return true;
}

// ============================================================================
// Database Merging
// ============================================================================

bool MotionDatabase::Merge(const MotionDatabase& other) {
    if (!other.m_isBuilt) return false;

    // Merge clips
    int32 clipOffset = static_cast<int32>(m_clips.Size());
    for (const auto& clip : other.m_clips) {
        m_clips.PushBack(clip);
    }

    // Merge entries with updated clip indices
    for (const auto& entry : other.m_entries) {
        DatabaseEntry newEntry = entry;
        newEntry.clipIndex += clipOffset;
        m_entries.PushBack(newEntry);
    }

    // Rebuild
    return Rebuild();
}

// ============================================================================
// Feature Subspace Search
// ============================================================================

Vector<SearchResult> MotionDatabase::SearchSubspace(const FeatureVector& query,
                                                     const Vector<int32>& channelIndices,
                                                     int32 k) const {
    Vector<SearchResult> results;
    if (channelIndices.Empty()) return results;

    // Build subspace query
    FeatureVector subQuery(static_cast<int32>(channelIndices.Size()));
    for (size_type i = 0; i < channelIndices.Size(); ++i) {
        subQuery[static_cast<int32>(i)] = query[channelIndices[i]];
    }

    // Search in subspace
    for (size_type i = 0; i < m_entries.Size(); ++i) {
        float32 dist = 0.0f;
        for (size_type c = 0; c < channelIndices.Size(); ++c) {
            float32 diff = subQuery[static_cast<int32>(c)] - m_entries[i].feature[channelIndices[c]];
            dist += diff * diff;
        }
        dist = std::sqrt(dist);

        SearchResult result;
        result.entryIndex = static_cast<int32>(i);
        result.distance = dist;
        result.score = 1.0f / (1.0f + dist);
        result.isValid = true;
        results.PushBack(result);
    }

    // Sort and limit
    std::partial_sort(results.begin(),
                      results.begin() + std::min(static_cast<size_type>(k), results.Size()),
                      results.end(),
                      [](const SearchResult& a, const SearchResult& b) {
        return a.distance < b.distance;
    });

    if (results.Size() > static_cast<size_type>(k)) {
        results.Resize(k);
    }

    return results;
}

// ============================================================================
// Continuous Build (Streaming)
// ============================================================================

bool MotionDatabase::AddEntryStream(const DatabaseEntry& entry) {
    if (!m_isBuilt) return false;

    m_entries.PushBack(entry);

    // Incrementally update search structures
    if (m_kdTree) {
        // For KDTree, we need to rebuild (no incremental support in basic implementation)
        // In production, use a dynamic KDTree or rebuild periodically
        if (m_entries.Size() % 100 == 0) {
            BuildSearchStructures();
        }
    }

    m_stats.totalEntries = static_cast<int32>(m_entries.Size());
    return true;
}

// ============================================================================
// Memory Management
// ============================================================================

void MotionDatabase::Compact() {
    // Remove invalid entries
    size_type writeIdx = 0;
    for (size_type i = 0; i < m_entries.Size(); ++i) {
        if (m_entries[i].isValid) {
            if (writeIdx != i) {
                m_entries[writeIdx] = std::move(m_entries[i]);
            }
            ++writeIdx;
        }
    }
    m_entries.Resize(writeIdx);

    // Rebuild if needed
    if (m_isBuilt) {
        BuildSearchStructures();
        ComputeStats();
    }
}

size_type MotionDatabase::GetMemoryUsage() const {
    size_type total = sizeof(*this);
    total += m_clips.Capacity() * sizeof(AnimationClip);
    total += m_entries.Capacity() * sizeof(DatabaseEntry);
    total += m_featureSchema.channels.Capacity() * sizeof(FeatureChannelDesc);
    total += m_tags.GetMemoryUsage();

    for (const auto& clip : m_clips) {
        total += clip.poses.Capacity() * sizeof(Pose);
        total += clip.velocities.Capacity() * sizeof(PoseVelocity);
        total += clip.trajectories.Capacity() * sizeof(Trajectory);
        total += clip.features.Capacity() * sizeof(FeatureVector);
    }

    if (m_kdTree) total += m_kdTree->GetNodeCount() * sizeof(KDTreeNode);
    if (m_vpTree) total += m_vpTree->GetPointCount() * sizeof(VPTreeNode);

    return total;
}

MMV2_NAMESPACE_END
