// ============================================================================
// MMV2 GDExtension - Pose Resource
// ============================================================================

#pragma once
#ifndef MMV2_GODOT_POSE_RESOURCE_H
#define MMV2_GODOT_POSE_RESOURCE_H

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include "MMV2/Core/Pose.h"

using namespace godot;

class MMV2PoseResource : public Resource {
    GDCLASS(MMV2PoseResource, Resource)

private:
    MMV2::Pose m_pose;

protected:
    static void _bind_methods();

public:
    MMV2PoseResource();
    ~MMV2PoseResource();

    void set_bone_count(int32 count);
    int32 get_bone_count() const;

    void set_bone_transform(int32 index, const Vector3& position, const Quaternion& rotation);
    Dictionary get_bone_transform(int32 index) const;

    void set_bone_position(int32 index, const Vector3& position);
    Vector3 get_bone_position(int32 index) const;

    void set_bone_rotation(int32 index, const Quaternion& rotation);
    Quaternion get_bone_rotation(int32 index) const;

    bool is_valid() const;
    void clear();

    Ref<MMV2PoseResource> duplicate_pose() const;

    // Internal
    const MMV2::Pose& get_native() const { return m_pose; }
    void set_native(const MMV2::Pose& pose) { m_pose = pose; }
};

#endif
