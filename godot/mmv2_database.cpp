// ============================================================================
// MMV2 GDExtension - Database Resource Implementation
// ============================================================================

#include "godot/mmv2_database.h"
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void MMV2Database::_bind_methods() {
    ClassDB::bind_method(D_METHOD("build_from_clips", "clips", "settings"), &MMV2Database::build_from_clips);
    ClassDB::bind_method(D_METHOD("load_from_file", "path"), &MMV2Database::load_from_file);
    ClassDB::bind_method(D_METHOD("save_to_file", "path"), &MMV2Database::save_to_file);
    ClassDB::bind_method(D_METHOD("get_entry_count"), &MMV2Database::get_entry_count);
    ClassDB::bind_method(D_METHOD("get_clip_count"), &MMV2Database::get_clip_count);
    ClassDB::bind_method(D_METHOD("get_entry", "index"), &MMV2Database::get_entry);
    ClassDB::bind_method(D_METHOD("get_clip", "index"), &MMV2Database::get_clip);
    ClassDB::bind_method(D_METHOD("find_nearest", "query_features"), &MMV2Database::find_nearest);
    ClassDB::bind_method(D_METHOD("find_k_nearest", "query_features", "k"), &MMV2Database::find_k_nearest);
    ClassDB::bind_method(D_METHOD("is_built"), &MMV2Database::is_built);
    ClassDB::bind_method(D_METHOD("clear"), &MMV2Database::clear);
    ClassDB::bind_method(D_METHOD("get_stats"), &MMV2Database::get_stats);

    ADD_SIGNAL(MethodInfo("database_built"));
    ADD_SIGNAL(MethodInfo("database_loaded"));
    ADD_SIGNAL(MethodInfo("database_saved"));
}

MMV2Database::MMV2Database() : m_database(nullptr), m_isBuilt(false) {
    m_database = memnew(MMV2::MotionDatabase);
}

MMV2Database::~MMV2Database() {
    if (m_database) {
        memdelete(m_database);
        m_database = nullptr;
    }
}

bool MMV2Database::build_from_clips(const TypedArray<Dictionary>& clips, const Dictionary& settings) {
    if (!m_database) return false;

    m_database->Clear();

    // Parse settings
    MMV2::DatabaseBuildSettings buildSettings;
    if (settings.has("pose_sample_rate")) {
        buildSettings.poseSampleRate = settings["pose_sample_rate"];
    }
    if (settings.has("min_clip_duration")) {
        buildSettings.minClipDuration = settings["min_clip_duration"];
    }
    if (settings.has("normalize_features")) {
        buildSettings.normalizeFeatures = settings["normalize_features"];
    }
    if (settings.has("build_search_structures")) {
        buildSettings.buildSearchStructures = settings["build_search_structures"];
    }

    // Add clips
    for (int32 i = 0; i < clips.size(); ++i) {
        Dictionary clipDict = clips[i];
        MMV2::AnimationClip clip;

        if (clipDict.has("name")) {
            clip.name = String(clipDict["name"]).utf8().get_data();
        }
        if (clipDict.has("duration")) {
            clip.duration = clipDict["duration"];
        }
        if (clipDict.has("frame_rate")) {
            clip.frameRate = clipDict["frame_rate"];
        }
        if (clipDict.has("is_looping")) {
            clip.isLooping = clipDict["is_looping"];
        }
        if (clipDict.has("is_mirrored")) {
            clip.isMirrored = clipDict["is_mirrored"];
        }

        // Parse poses if provided
        if (clipDict.has("poses")) {
            TypedArray<Dictionary> poses = clipDict["poses"];
            clip.poses.Resize(poses.size());
            for (int32 p = 0; p < poses.size(); ++p) {
                // Parse pose data - simplified
                // In production, this would parse bone transforms
            }
        }

        m_database->AddClip(clip);
    }

    bool success = m_database->Build(buildSettings);
    m_isBuilt = success;

    if (success) {
        emit_signal("database_built");
    }

    return success;
}

bool MMV2Database::load_from_file(const String& path) {
    if (!m_database) return false;
    bool success = m_database->Load(path.utf8().get_data());
    m_isBuilt = success;
    if (success) {
        emit_signal("database_loaded");
    }
    return success;
}

bool MMV2Database::save_to_file(const String& path) const {
    if (!m_database || !m_isBuilt) return false;
    bool success = m_database->Save(path.utf8().get_data());
    if (success) {
        emit_signal("database_saved");
    }
    return success;
}

int32 MMV2Database::get_entry_count() const {
    if (!m_database) return 0;
    return m_database->GetEntryCount();
}

int32 MMV2Database::get_clip_count() const {
    if (!m_database) return 0;
    return m_database->GetClipCount();
}

Dictionary MMV2Database::get_entry(int32 index) const {
    Dictionary result;
    if (!m_database || index < 0 || index >= m_database->GetEntryCount()) return result;

    const MMV2::DatabaseEntry* entry = m_database->GetEntry(index);
    if (!entry) return result;

    result["clip_index"] = entry->clipIndex;
    result["frame_index"] = entry->frameIndex;
    result["time"] = entry->time;
    result["phase"] = entry->phase;
    result["contact_left"] = entry->contactLeft;
    result["contact_right"] = entry->contactRight;
    result["is_valid"] = entry->isValid;

    // Feature vector
    PackedFloat32Array features;
    features.resize(entry->feature.Size());
    for (int32 i = 0; i < entry->feature.Size(); ++i) {
        features[i] = entry->feature[i];
    }
    result["features"] = features;

    return result;
}

Dictionary MMV2Database::get_clip(int32 index) const {
    Dictionary result;
    if (!m_database || index < 0 || index >= m_database->GetClipCount()) return result;

    const MMV2::AnimationClip* clip = m_database->GetClip(index);
    if (!clip) return result;

    result["name"] = String(clip->name.CStr());
    result["source_path"] = String(clip->sourcePath.CStr());
    result["duration"] = clip->duration;
    result["frame_rate"] = clip->frameRate;
    result["frame_count"] = clip->frameCount;
    result["bone_count"] = clip->boneCount;
    result["is_looping"] = clip->isLooping;
    result["is_mirrored"] = clip->isMirrored;

    return result;
}

int32 MMV2Database::find_nearest(const PackedFloat32Array& query_features) const {
    if (!m_database || !m_isBuilt || query_features.is_empty()) return -1;

    MMV2::FeatureVector query(query_features.size());
    for (int32 i = 0; i < query_features.size(); ++i) {
        query[i] = query_features[i];
    }

    return m_database->FindNearest(query);
}

PackedInt32Array MMV2Database::find_k_nearest(const PackedFloat32Array& query_features, int32 k) const {
    PackedInt32Array result;
    if (!m_database || !m_isBuilt || query_features.is_empty() || k <= 0) return result;

    MMV2::FeatureVector query(query_features.size());
    for (int32 i = 0; i < query_features.size(); ++i) {
        query[i] = query_features[i];
    }

    MMV2::Vector<MMV2::int32> indices;
    m_database->FindKNearest(query, k, indices, nullptr);

    result.resize(indices.Size());
    for (MMV2::size_type i = 0; i < indices.Size(); ++i) {
        result[i] = indices[i];
    }

    return result;
}

void MMV2Database::clear() {
    if (m_database) {
        m_database->Clear();
    }
    m_isBuilt = false;
}

Dictionary MMV2Database::get_stats() const {
    Dictionary result;
    if (!m_database) return result;

    const MMV2::DatabaseStats& stats = m_database->GetStats();
    result["total_entries"] = stats.totalEntries;
    result["total_clips"] = stats.totalClips;
    result["total_frames"] = stats.totalFrames;
    result["total_features"] = stats.totalFeatures;
    result["memory_usage"] = static_cast<int64_t>(stats.memoryUsage);
    result["avg_feature_distance"] = stats.avgFeatureDistance;
    result["max_feature_distance"] = stats.maxFeatureDistance;
    result["build_time"] = stats.buildTime;
    result["avg_search_time"] = stats.avgSearchTime;

    return result;
}
