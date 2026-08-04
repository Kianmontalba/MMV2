// ============================================================================
// MMV2 GDExtension - Pose Resource Implementation
// ============================================================================

#include "godot/mmv2_pose_resource.h"
#include <godot_cpp/core/memory.hpp>

using namespace godot;

void MMV2PoseResource::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_bone_count", "count"), &MMV2PoseResource::set_bone_count);
    ClassDB::bind_method(D_METHOD("get_bone_count"), &MMV2PoseResource::get_bone_count);
    ClassDB::bind_method(D_METHOD("set_bone_transform", "index", "position", "rotation"), &MMV2PoseResource::set_bone_transform);
    ClassDB::bind_method(D_METHOD("get_bone_transform", "index"), &MMV2PoseResource::get_bone_transform);
    ClassDB::bind_method(D_METHOD("set_bone_position", "index", "position"), &MMV2PoseResource::set_bone_position);
    ClassDB::bind_method(D_METHOD("get_bone_position", "index"), &MMV2PoseResource::get_bone_position);
    ClassDB::bind_method(D_METHOD("set_bone_rotation", "index", "rotation"), &MMV2PoseResource::set_bone_rotation);
    ClassDB::bind_method(D_METHOD("get_bone_rotation", "index"), &MMV2PoseResource::get_bone_rotation);
    ClassDB::bind_method(D_METHOD("is_valid"), &MMV2PoseResource::is_valid);
    ClassDB::bind_method(D_METHOD("clear"), &MMV2PoseResource::clear);
    ClassDB::bind_method(D_METHOD("duplicate_pose"), &MMV2PoseResource::duplicate_pose);
}

MMV2PoseResource::MMV2PoseResource() {}

MMV2PoseResource::~MMV2PoseResource() {}

void MMV2PoseResource::set_bone_count(int32 count) {
    m_pose.Resize(count);
}

int32 MMV2PoseResource::get_bone_count() const {
    return m_pose.GetBoneCount();
}

void MMV2PoseResource::set_bone_transform(int32 index, const Vector3& position, const Quaternion& rotation) {
    if (index < 0 || index >= m_pose.GetBoneCount()) return;
    MMV2::Transform t;
    t.position = MMV2::Vec3(position.x, position.y, position.z);
    t.rotation = MMV2::Quat(rotation.x, rotation.y, rotation.z, rotation.w);
    m_pose.SetBoneTransform(index, t);
}

Dictionary MMV2PoseResource::get_bone_transform(int32 index) const {
    Dictionary result;
    if (index < 0 || index >= m_pose.GetBoneCount()) return result;

    const MMV2::Transform& t = m_pose.GetBoneTransform(index);
    result["position"] = Vector3(t.position.x, t.position.y, t.position.z);
    result["rotation"] = Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);

    return result;
}

void MMV2PoseResource::set_bone_position(int32 index, const Vector3& position) {
    if (index < 0 || index >= m_pose.GetBoneCount()) return;
    MMV2::Transform t = m_pose.GetBoneTransform(index);
    t.position = MMV2::Vec3(position.x, position.y, position.z);
    m_pose.SetBoneTransform(index, t);
}

Vector3 MMV2PoseResource::get_bone_position(int32 index) const {
    if (index < 0 || index >= m_pose.GetBoneCount()) return Vector3();
    const MMV2::Transform& t = m_pose.GetBoneTransform(index);
    return Vector3(t.position.x, t.position.y, t.position.z);
}

void MMV2PoseResource::set_bone_rotation(int32 index, const Quaternion& rotation) {
    if (index < 0 || index >= m_pose.GetBoneCount()) return;
    MMV2::Transform t = m_pose.GetBoneTransform(index);
    t.rotation = MMV2::Quat(rotation.x, rotation.y, rotation.z, rotation.w);
    m_pose.SetBoneTransform(index, t);
}

Quaternion MMV2PoseResource::get_bone_rotation(int32 index) const {
    if (index < 0 || index >= m_pose.GetBoneCount()) return Quaternion();
    const MMV2::Transform& t = m_pose.GetBoneTransform(index);
    return Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
}

bool MMV2PoseResource::is_valid() const {
    return m_pose.GetBoneCount() > 0;
}

void MMV2PoseResource::clear() {
    m_pose = MMV2::Pose();
}

Ref<MMV2PoseResource> MMV2PoseResource::duplicate_pose() const {
    Ref<MMV2PoseResource> dup;
    dup.instantiate();
    dup->set_native(m_pose);
    return dup;
}
