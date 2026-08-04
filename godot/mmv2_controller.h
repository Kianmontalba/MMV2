// ============================================================================
// MMV2 GDExtension - Motion Matching Controller Node
// ============================================================================

#pragma once
#ifndef MMV2_GODOT_CONTROLLER_H
#define MMV2_GODOT_CONTROLLER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/animation_player.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include "godot/mmv2_database.h"
#include "MMV2/Animation/Controller.h"

using namespace godot;

class MMV2Controller : public Node3D {
    GDCLASS(MMV2Controller, Node3D)

private:
    MMV2::MotionMatchingController* m_controller;
    Ref<MMV2Database> m_database;
    NodePath m_skeleton_path;
    Skeleton3D* m_skeleton;
    bool m_isInitialized;
    bool m_autoUpdate;
    float32 m_searchInterval;
    float32 m_blendTime;
    float32 m_playbackSpeed;
    bool m_enableFootLocking;
    bool m_enableMotionWarping;
    bool m_enableInertialization;
    bool m_useAsyncSearch;

    // Trajectory input
    Vector3 m_desiredVelocity;
    Vector3 m_desiredDirection;
    bool m_hasTrajectoryInput;

protected:
    static void _bind_methods();
    void _notification(int p_what);

public:
    MMV2Controller();
    ~MMV2Controller();

    void _ready() override;
    void _process(double delta) override;

    // Initialization
    bool initialize(Ref<MMV2Database> database);
    void shutdown();
    bool is_initialized() const { return m_isInitialized; }

    // Database
    void set_database(Ref<MMV2Database> database);
    Ref<MMV2Database> get_database() const { return m_database; }

    // Skeleton
    void set_skeleton_path(const NodePath& path);
    NodePath get_skeleton_path() const { return m_skeleton_path; }

    // Playback
    void set_playback_speed(float32 speed);
    float32 get_playback_speed() const { return m_playbackSpeed; }

    void set_auto_update(bool enabled);
    bool get_auto_update() const { return m_autoUpdate; }

    // Search settings
    void set_search_interval(float32 interval);
    float32 get_search_interval() const { return m_searchInterval; }

    void set_blend_time(float32 time);
    float32 get_blend_time() const { return m_blendTime; }

    void set_use_async_search(bool enabled);
    bool get_use_async_search() const { return m_useAsyncSearch; }

    // Feature weights
    void set_pose_weight(float32 weight);
    float32 get_pose_weight() const;
    void set_trajectory_weight(float32 weight);
    float32 get_trajectory_weight() const;
    void set_velocity_weight(float32 weight);
    float32 get_velocity_weight() const;

    // Post-processing
    void set_enable_foot_locking(bool enabled);
    bool get_enable_foot_locking() const { return m_enableFootLocking; }

    void set_enable_motion_warping(bool enabled);
    bool get_enable_motion_warping() const { return m_enableMotionWarping; }

    void set_enable_inertialization(bool enabled);
    bool get_enable_inertialization() const { return m_enableInertialization; }

    // Trajectory input
    void set_desired_velocity(const Vector3& velocity);
    Vector3 get_desired_velocity() const { return m_desiredVelocity; }

    void set_desired_direction(const Vector3& direction);
    Vector3 get_desired_direction() const { return m_desiredDirection; }

    // Control
    void force_transition(int32 entry_index);
    void force_transition_by_clip(const String& clip_name, float32 normalized_time);
    void pause();
    void resume();
    void reset();

    // State queries
    int32 get_current_entry() const;
    int32 get_next_entry() const;
    float32 get_current_time() const;
    float32 get_normalized_time() const;
    float32 get_phase() const;
    bool is_transitioning() const;
    float32 get_transition_progress() const;

    // Debug
    String get_debug_info() const;
    Dictionary get_current_pose_data() const;
    PackedFloat32Array get_current_features() const;
    Dictionary get_last_search_results() const;

    // Internal
    MMV2::MotionMatchingController* get_native() const { return m_controller; }

private:
    void update_skeleton();
    void apply_pose_to_skeleton(const MMV2::Pose& pose);
    MMV2::Vec3 to_mmvec3(const Vector3& v) const;
    Vector3 to_gdvec3(const MMV2::Vec3& v) const;
    MMV2::Quat to_mmquat(const Quaternion& q) const;
    Quaternion to_gdquat(const MMV2::Quat& q) const;
};

#endif
