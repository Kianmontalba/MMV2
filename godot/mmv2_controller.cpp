// ============================================================================
// MMV2 GDExtension - Controller Implementation
// ============================================================================

#include "godot/mmv2_controller.h"
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/classes/scene_tree.hpp>

using namespace godot;

void MMV2Controller::_bind_methods() {
    // Properties
    ClassDB::bind_method(D_METHOD("set_database", "database"), &MMV2Controller::set_database);
    ClassDB::bind_method(D_METHOD("get_database"), &MMV2Controller::get_database);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "database", PROPERTY_HINT_RESOURCE_TYPE, "MMV2Database"), "set_database", "get_database");

    ClassDB::bind_method(D_METHOD("set_skeleton_path", "path"), &MMV2Controller::set_skeleton_path);
    ClassDB::bind_method(D_METHOD("get_skeleton_path"), &MMV2Controller::get_skeleton_path);
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "skeleton_path"), "set_skeleton_path", "get_skeleton_path");

    ClassDB::bind_method(D_METHOD("set_playback_speed", "speed"), &MMV2Controller::set_playback_speed);
    ClassDB::bind_method(D_METHOD("get_playback_speed"), &MMV2Controller::get_playback_speed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "playback_speed"), "set_playback_speed", "get_playback_speed");

    ClassDB::bind_method(D_METHOD("set_auto_update", "enabled"), &MMV2Controller::set_auto_update);
    ClassDB::bind_method(D_METHOD("get_auto_update"), &MMV2Controller::get_auto_update);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_update"), "set_auto_update", "get_auto_update");

    ClassDB::bind_method(D_METHOD("set_search_interval", "interval"), &MMV2Controller::set_search_interval);
    ClassDB::bind_method(D_METHOD("get_search_interval"), &MMV2Controller::get_search_interval);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "search_interval"), "set_search_interval", "get_search_interval");

    ClassDB::bind_method(D_METHOD("set_blend_time", "time"), &MMV2Controller::set_blend_time);
    ClassDB::bind_method(D_METHOD("get_blend_time"), &MMV2Controller::get_blend_time);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend_time"), "set_blend_time", "get_blend_time");

    ClassDB::bind_method(D_METHOD("set_use_async_search", "enabled"), &MMV2Controller::set_use_async_search);
    ClassDB::bind_method(D_METHOD("get_use_async_search"), &MMV2Controller::get_use_async_search);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_async_search"), "set_use_async_search", "get_use_async_search");

    ClassDB::bind_method(D_METHOD("set_enable_foot_locking", "enabled"), &MMV2Controller::set_enable_foot_locking);
    ClassDB::bind_method(D_METHOD("get_enable_foot_locking"), &MMV2Controller::get_enable_foot_locking);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enable_foot_locking"), "set_enable_foot_locking", "get_enable_foot_locking");

    ClassDB::bind_method(D_METHOD("set_enable_motion_warping", "enabled"), &MMV2Controller::set_enable_motion_warping);
    ClassDB::bind_method(D_METHOD("get_enable_motion_warping"), &MMV2Controller::get_enable_motion_warping);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enable_motion_warping"), "set_enable_motion_warping", "get_enable_motion_warping");

    ClassDB::bind_method(D_METHOD("set_enable_inertialization", "enabled"), &MMV2Controller::set_enable_inertialization);
    ClassDB::bind_method(D_METHOD("get_enable_inertialization"), &MMV2Controller::get_enable_inertialization);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enable_inertialization"), "set_enable_inertialization", "get_enable_inertialization");

    ClassDB::bind_method(D_METHOD("set_desired_velocity", "velocity"), &MMV2Controller::set_desired_velocity);
    ClassDB::bind_method(D_METHOD("get_desired_velocity"), &MMV2Controller::get_desired_velocity);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "desired_velocity"), "set_desired_velocity", "get_desired_velocity");

    ClassDB::bind_method(D_METHOD("set_desired_direction", "direction"), &MMV2Controller::set_desired_direction);
    ClassDB::bind_method(D_METHOD("get_desired_direction"), &MMV2Controller::get_desired_direction);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "desired_direction"), "set_desired_direction", "get_desired_direction");

    // Methods
    ClassDB::bind_method(D_METHOD("initialize", "database"), &MMV2Controller::initialize);
    ClassDB::bind_method(D_METHOD("shutdown"), &MMV2Controller::shutdown);
    ClassDB::bind_method(D_METHOD("is_initialized"), &MMV2Controller::is_initialized);
    ClassDB::bind_method(D_METHOD("force_transition", "entry_index"), &MMV2Controller::force_transition);
    ClassDB::bind_method(D_METHOD("force_transition_by_clip", "clip_name", "normalized_time"), &MMV2Controller::force_transition_by_clip);
    ClassDB::bind_method(D_METHOD("pause"), &MMV2Controller::pause);
    ClassDB::bind_method(D_METHOD("resume"), &MMV2Controller::resume);
    ClassDB::bind_method(D_METHOD("reset"), &MMV2Controller::reset);
    ClassDB::bind_method(D_METHOD("get_current_entry"), &MMV2Controller::get_current_entry);
    ClassDB::bind_method(D_METHOD("get_next_entry"), &MMV2Controller::get_next_entry);
    ClassDB::bind_method(D_METHOD("get_current_time"), &MMV2Controller::get_current_time);
    ClassDB::bind_method(D_METHOD("get_normalized_time"), &MMV2Controller::get_normalized_time);
    ClassDB::bind_method(D_METHOD("get_phase"), &MMV2Controller::get_phase);
    ClassDB::bind_method(D_METHOD("is_transitioning"), &MMV2Controller::is_transitioning);
    ClassDB::bind_method(D_METHOD("get_transition_progress"), &MMV2Controller::get_transition_progress);
    ClassDB::bind_method(D_METHOD("get_debug_info"), &MMV2Controller::get_debug_info);
    ClassDB::bind_method(D_METHOD("get_current_pose_data"), &MMV2Controller::get_current_pose_data);
    ClassDB::bind_method(D_METHOD("get_current_features"), &MMV2Controller::get_current_features);
    ClassDB::bind_method(D_METHOD("get_last_search_results"), &MMV2Controller::get_last_search_results);

    // Signals
    ADD_SIGNAL(MethodInfo("state_changed", PropertyInfo(Variant::INT, "old_state"), PropertyInfo(Variant::INT, "new_state")));
    ADD_SIGNAL(MethodInfo("transition_started", PropertyInfo(Variant::INT, "from_entry"), PropertyInfo(Variant::INT, "to_entry")));
    ADD_SIGNAL(MethodInfo("search_completed", PropertyInfo(Variant::FLOAT, "search_time_ms")));
    ADD_SIGNAL(MethodInfo("pose_applied"));
}

MMV2Controller::MMV2Controller()
    : m_controller(nullptr), m_skeleton(nullptr), m_isInitialized(false),
      m_autoUpdate(true), m_searchInterval(0.1f), m_blendTime(0.2f),
      m_playbackSpeed(1.0f), m_enableFootLocking(true), m_enableMotionWarping(true),
      m_enableInertialization(true), m_useAsyncSearch(false), m_hasTrajectoryInput(false) {
    m_controller = memnew(MMV2::MotionMatchingController);
}

MMV2Controller::~MMV2Controller() {
    shutdown();
    if (m_controller) {
        memdelete(m_controller);
        m_controller = nullptr;
    }
}

void MMV2Controller::_notification(int p_what) {
    switch (p_what) {
        case NOTIFICATION_READY:
            _ready();
            break;
        case NOTIFICATION_PROCESS:
            if (m_autoUpdate && m_isInitialized) {
                _process(get_process_delta_time());
            }
            break;
        case NOTIFICATION_EXIT_TREE:
            shutdown();
            break;
    }
}

void MMV2Controller::_ready() {
    if (!m_skeleton_path.is_empty()) {
        Node* node = get_node_or_null(m_skeleton_path);
        if (node) {
            m_skeleton = Object::cast_to<Skeleton3D>(node);
        }
    }

    if (!m_skeleton && m_database.is_valid() && m_database->is_built()) {
        initialize(m_database);
    }
}

void MMV2Controller::_process(double delta) {
    if (!m_isInitialized || !m_controller) return;

    float32 dt = static_cast<float32>(delta);

    // Update trajectory input if provided
    if (m_hasTrajectoryInput) {
        MMV2::TrajectoryGenerator generator;
        generator.sampleCount = 8;
        generator.sampleInterval = 0.1f;
        generator.totalTime = 1.0f;

        MMV2::Vec3 vel = to_mmvec3(m_desiredVelocity);
        MMV2::Vec3 pos = to_mmvec3(get_global_position());
        MMV2::Quat rot = to_mmquat(get_global_basis().get_rotation_quaternion());

        MMV2::Trajectory trajectory = generator.GenerateFromVelocity(vel, pos, rot);
        m_controller->SetTrajectory(trajectory);
    }

    // Update controller
    if (m_useAsyncSearch) {
        m_controller->UpdateAsync(dt);
    } else {
        m_controller->Update(dt);
    }

    // Apply pose to skeleton
    if (m_skeleton) {
        update_skeleton();
    }

    // Update this node's transform to match root motion
    const MMV2::Pose& currentPose = m_controller->GetCurrentPose();
    if (currentPose.GetBoneCount() > 0) {
        MMV2::Vec3 rootPos = currentPose.GetBoneTransform(0).position;
        MMV2::Quat rootRot = currentPose.GetBoneTransform(0).rotation;
        set_global_position(to_gdvec3(rootPos));
        set_global_basis(Basis(to_gdquat(rootRot)));
    }
}

bool MMV2Controller::initialize(Ref<MMV2Database> database) {
    if (!database.is_valid() || !database->is_built()) return false;
    if (!m_controller) return false;

    m_database = database;

    MMV2::MotionMatchingSettings settings;
    settings.searchInterval = m_searchInterval;
    settings.transition.blendTime = m_blendTime;
    settings.playback.playbackSpeed = m_playbackSpeed;
    settings.playback.enableFootLocking = m_enableFootLocking;
    settings.playback.enableMotionWarping = m_enableMotionWarping;
    settings.playback.enableInertialization = m_enableInertialization;
    settings.search.useKDTree = true;
    settings.search.kNearest = 10;

    bool success = m_controller->Initialize(database->get_native(), settings);
    m_isInitialized = success;

    return success;
}

void MMV2Controller::shutdown() {
    if (m_controller) {
        m_controller->Shutdown();
    }
    m_isInitialized = false;
}

void MMV2Controller::set_database(Ref<MMV2Database> database) {
    m_database = database;
    if (m_database.is_valid() && m_database->is_built() && !m_isInitialized) {
        initialize(m_database);
    }
}

void MMV2Controller::set_skeleton_path(const NodePath& path) {
    m_skeleton_path = path;
    if (is_inside_tree()) {
        Node* node = get_node_or_null(path);
        if (node) {
            m_skeleton = Object::cast_to<Skeleton3D>(node);
        }
    }
}

void MMV2Controller::set_playback_speed(float32 speed) {
    m_playbackSpeed = speed;
    if (m_controller) {
        m_controller->SetPlaybackSpeed(speed);
    }
}

float32 MMV2Controller::get_playback_speed() const {
    return m_playbackSpeed;
}

void MMV2Controller::set_auto_update(bool enabled) {
    m_autoUpdate = enabled;
    set_process(enabled);
}

bool MMV2Controller::get_auto_update() const {
    return m_autoUpdate;
}

void MMV2Controller::set_search_interval(float32 interval) {
    m_searchInterval = Math::Max(interval, 0.01f);
}

float32 MMV2Controller::get_search_interval() const {
    return m_searchInterval;
}

void MMV2Controller::set_blend_time(float32 time) {
    m_blendTime = Math::Max(time, 0.0f);
}

float32 MMV2Controller::get_blend_time() const {
    return m_blendTime;
}

void MMV2Controller::set_use_async_search(bool enabled) {
    m_useAsyncSearch = enabled;
}

bool MMV2Controller::get_use_async_search() const {
    return m_useAsyncSearch;
}

void MMV2Controller::set_enable_foot_locking(bool enabled) {
    m_enableFootLocking = enabled;
}

bool MMV2Controller::get_enable_foot_locking() const {
    return m_enableFootLocking;
}

void MMV2Controller::set_enable_motion_warping(bool enabled) {
    m_enableMotionWarping = enabled;
}

bool MMV2Controller::get_enable_motion_warping() const {
    return m_enableMotionWarping;
}

void MMV2Controller::set_enable_inertialization(bool enabled) {
    m_enableInertialization = enabled;
}

bool MMV2Controller::get_enable_inertialization() const {
    return m_enableInertialization;
}

void MMV2Controller::set_desired_velocity(const Vector3& velocity) {
    m_desiredVelocity = velocity;
    m_hasTrajectoryInput = true;
}

Vector3 MMV2Controller::get_desired_velocity() const {
    return m_desiredVelocity;
}

void MMV2Controller::set_desired_direction(const Vector3& direction) {
    m_desiredDirection = direction.normalized();
    m_hasTrajectoryInput = true;
}

Vector3 MMV2Controller::get_desired_direction() const {
    return m_desiredDirection;
}

void MMV2Controller::force_transition(int32 entry_index) {
    if (!m_controller) return;
    MMV2::TransitionSettings settings;
    settings.blendTime = m_blendTime;
    m_controller->ForceTransition(entry_index, settings);
}

void MMV2Controller::force_transition_by_clip(const String& clip_name, float32 normalized_time) {
    if (!m_controller) return;
    MMV2::TransitionSettings settings;
    settings.blendTime = m_blendTime;
    m_controller->ForceTransition(clip_name.utf8().get_data(), normalized_time, settings);
}

void MMV2Controller::pause() {
    if (m_controller) m_controller->Pause();
}

void MMV2Controller::resume() {
    if (m_controller) m_controller->Resume();
}

void MMV2Controller::reset() {
    if (m_controller) m_controller->Reset();
}

int32 MMV2Controller::get_current_entry() const {
    if (!m_controller) return -1;
    return m_controller->GetCurrentState().currentEntry;
}

int32 MMV2Controller::get_next_entry() const {
    if (!m_controller) return -1;
    return m_controller->GetCurrentState().nextEntry;
}

float32 MMV2Controller::get_current_time() const {
    if (!m_controller) return 0.0f;
    return m_controller->GetCurrentState().currentTime;
}

float32 MMV2Controller::get_normalized_time() const {
    if (!m_controller) return 0.0f;
    return m_controller->GetCurrentState().normalizedTime;
}

float32 MMV2Controller::get_phase() const {
    if (!m_controller) return 0.0f;
    return m_controller->GetCurrentState().phase;
}

bool MMV2Controller::is_transitioning() const {
    if (!m_controller) return false;
    return m_controller->GetCurrentState().transition.isActive;
}

float32 MMV2Controller::get_transition_progress() const {
    if (!m_controller) return 0.0f;
    return m_controller->GetCurrentState().transition.progress;
}

String MMV2Controller::get_debug_info() const {
    if (!m_controller) return "Not initialized";
    return String(m_controller->GetDebugInfo().CStr());
}

Dictionary MMV2Controller::get_current_pose_data() const {
    Dictionary result;
    if (!m_controller) return result;

    const MMV2::Pose& pose = m_controller->GetCurrentPose();
    result["bone_count"] = pose.GetBoneCount();

    Array bone_data;
    for (int32 i = 0; i < pose.GetBoneCount(); ++i) {
        Dictionary bone;
        MMV2::Vec3 pos = pose.GetBoneTransform(i).position;
        MMV2::Quat rot = pose.GetBoneTransform(i).rotation;
        bone["position"] = to_gdvec3(pos);
        bone["rotation"] = to_gdquat(rot);
        bone["scale"] = Vector3(1, 1, 1);
        bone_data.append(bone);
    }
    result["bones"] = bone_data;

    return result;
}

PackedFloat32Array MMV2Controller::get_current_features() const {
    PackedFloat32Array result;
    if (!m_controller) return result;

    const MMV2::FeatureVector& features = m_controller->GetCurrentState().currentFeatures;
    result.resize(features.Size());
    for (int32 i = 0; i < features.Size(); ++i) {
        result[i] = features[i];
    }

    return result;
}

Dictionary MMV2Controller::get_last_search_results() const {
    Dictionary result;
    if (!m_controller) return result;

    const MMV2::Vector<MMV2::SearchResult>& results = m_controller->GetCurrentState().lastSearchResults;
    Array entries;
    Array scores;
    Array distances;

    for (MMV2::size_type i = 0; i < results.Size(); ++i) {
        entries.append(results[i].entryIndex);
        scores.append(results[i].score);
        distances.append(results[i].distance);
    }

    result["entries"] = entries;
    result["scores"] = scores;
    result["distances"] = distances;

    return result;
}

void MMV2Controller::update_skeleton() {
    if (!m_skeleton || !m_controller) return;

    const MMV2::Pose& pose = m_controller->GetCurrentPose();
    apply_pose_to_skeleton(pose);
}

void MMV2Controller::apply_pose_to_skeleton(const MMV2::Pose& pose) {
    if (!m_skeleton) return;

    for (int32 i = 0; i < pose.GetBoneCount() && i < m_skeleton->get_bone_count(); ++i) {
        MMV2::Vec3 pos = pose.GetBoneTransform(i).position;
        MMV2::Quat rot = pose.GetBoneTransform(i).rotation;

        Vector3 gdPos = to_gdvec3(pos);
        Quaternion gdRot = to_gdquat(rot);

        // Convert to local space if needed
        int32 parent = m_skeleton->get_bone_parent(i);
        if (parent >= 0) {
            Transform3D parentGlobal = m_skeleton->get_bone_global_pose(parent);
            Transform3D boneGlobal = Transform3D(Basis(gdRot), gdPos);
            Transform3D boneLocal = parentGlobal.affine_inverse() * boneGlobal;
            gdPos = boneLocal.origin;
            gdRot = boneLocal.basis.get_rotation_quaternion();
        }

        m_skeleton->set_bone_pose_position(i, gdPos);
        m_skeleton->set_bone_pose_rotation(i, gdRot);
    }
}

MMV2::Vec3 MMV2Controller::to_mmvec3(const Vector3& v) const {
    return MMV2::Vec3(v.x, v.y, v.z);
}

Vector3 MMV2Controller::to_gdvec3(const MMV2::Vec3& v) const {
    return Vector3(v.x, v.y, v.z);
}

MMV2::Quat MMV2Controller::to_mmquat(const Quaternion& q) const {
    return MMV2::Quat(q.x, q.y, q.z, q.w);
}

Quaternion MMV2Controller::to_gdquat(const MMV2::Quat& q) const {
    return Quaternion(q.x, q.y, q.z, q.w);
}

// Weight getters/setters
void MMV2Controller::set_pose_weight(float32 weight) {
    if (!m_controller) return;
    MMV2::MotionMatchingSettings settings = m_controller->GetSettings();
    settings.search.poseWeight = weight;
    m_controller->SetSettings(settings);
}

float32 MMV2Controller::get_pose_weight() const {
    if (!m_controller) return 0.0f;
    return m_controller->GetSettings().search.poseWeight;
}

void MMV2Controller::set_trajectory_weight(float32 weight) {
    if (!m_controller) return;
    MMV2::MotionMatchingSettings settings = m_controller->GetSettings();
    settings.search.trajectoryWeight = weight;
    m_controller->SetSettings(settings);
}

float32 MMV2Controller::get_trajectory_weight() const {
    if (!m_controller) return 0.0f;
    return m_controller->GetSettings().search.trajectoryWeight;
}

void MMV2Controller::set_velocity_weight(float32 weight) {
    if (!m_controller) return;
    MMV2::MotionMatchingSettings settings = m_controller->GetSettings();
    settings.search.velocityWeight = weight;
    m_controller->SetSettings(settings);
}

float32 MMV2Controller::get_velocity_weight() const {
    if (!m_controller) return 0.0f;
    return m_controller->GetSettings().search.velocityWeight;
}
