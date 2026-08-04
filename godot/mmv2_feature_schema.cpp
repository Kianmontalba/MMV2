// ============================================================================
// MMV2 GDExtension - Feature Schema Implementation
// ============================================================================

#include "godot/mmv2_feature_schema.h"
#include <godot_cpp/core/memory.hpp>

using namespace godot;

void MMV2FeatureSchema::_bind_methods() {
    ClassDB::bind_method(D_METHOD("add_position_channel", "bone_index", "weight"), &MMV2FeatureSchema::add_position_channel);
    ClassDB::bind_method(D_METHOD("add_rotation_channel", "bone_index", "weight"), &MMV2FeatureSchema::add_rotation_channel);
    ClassDB::bind_method(D_METHOD("add_velocity_channel", "bone_index", "weight"), &MMV2FeatureSchema::add_velocity_channel);
    ClassDB::bind_method(D_METHOD("add_trajectory_position_channel", "sample_index", "weight"), &MMV2FeatureSchema::add_trajectory_position_channel);
    ClassDB::bind_method(D_METHOD("add_trajectory_velocity_channel", "sample_index", "weight"), &MMV2FeatureSchema::add_trajectory_velocity_channel);
    ClassDB::bind_method(D_METHOD("add_trajectory_direction_channel", "sample_index", "weight"), &MMV2FeatureSchema::add_trajectory_direction_channel);
    ClassDB::bind_method(D_METHOD("add_root_velocity_channel", "weight"), &MMV2FeatureSchema::add_root_velocity_channel);
    ClassDB::bind_method(D_METHOD("add_root_height_channel", "weight"), &MMV2FeatureSchema::add_root_height_channel);
    ClassDB::bind_method(D_METHOD("add_foot_contact_channel", "weight"), &MMV2FeatureSchema::add_foot_contact_channel);
    ClassDB::bind_method(D_METHOD("add_phase_channel", "weight"), &MMV2FeatureSchema::add_phase_channel);
    ClassDB::bind_method(D_METHOD("clear_channels"), &MMV2FeatureSchema::clear_channels);
    ClassDB::bind_method(D_METHOD("get_channel_count"), &MMV2FeatureSchema::get_channel_count);
    ClassDB::bind_method(D_METHOD("get_dimension"), &MMV2FeatureSchema::get_dimension);
    ClassDB::bind_method(D_METHOD("is_valid"), &MMV2FeatureSchema::is_valid);
    ClassDB::bind_method(D_METHOD("get_channel_info", "index"), &MMV2FeatureSchema::get_channel_info);
    ClassDB::bind_method(D_METHOD("get_channel_names"), &MMV2FeatureSchema::get_channel_names);
}

MMV2FeatureSchema::MMV2FeatureSchema() {}

MMV2FeatureSchema::~MMV2FeatureSchema() {}

void MMV2FeatureSchema::add_position_channel(int32 bone_index, float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::BonePosition;
    channel.boneIndex = bone_index;
    channel.weight = weight;
    channel.dimension = 3;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_rotation_channel(int32 bone_index, float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::BoneRotation;
    channel.boneIndex = bone_index;
    channel.weight = weight;
    channel.dimension = 4;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_velocity_channel(int32 bone_index, float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::BoneVelocity;
    channel.boneIndex = bone_index;
    channel.weight = weight;
    channel.dimension = 3;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_trajectory_position_channel(int32 sample_index, float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::TrajectoryPosition;
    channel.trajectoryIndex = sample_index;
    channel.weight = weight;
    channel.dimension = 3;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_trajectory_velocity_channel(int32 sample_index, float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::TrajectoryVelocity;
    channel.trajectoryIndex = sample_index;
    channel.weight = weight;
    channel.dimension = 3;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_trajectory_direction_channel(int32 sample_index, float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::TrajectoryDirection;
    channel.trajectoryIndex = sample_index;
    channel.weight = weight;
    channel.dimension = 3;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_root_velocity_channel(float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::RootVelocity;
    channel.weight = weight;
    channel.dimension = 3;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_root_height_channel(float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::RootHeight;
    channel.weight = weight;
    channel.dimension = 1;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_foot_contact_channel(float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::FootContact;
    channel.weight = weight;
    channel.dimension = 2;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::add_phase_channel(float32 weight) {
    MMV2::FeatureChannelDesc channel;
    channel.type = MMV2::FeatureChannelType::Phase;
    channel.weight = weight;
    channel.dimension = 1;
    channel.enabled = true;
    m_schema.channels.PushBack(channel);
    m_schema.RecomputeDimension();
}

void MMV2FeatureSchema::clear_channels() {
    m_schema.channels.Clear();
    m_schema.RecomputeDimension();
}

int32 MMV2FeatureSchema::get_channel_count() const {
    return m_schema.channels.Size();
}

int32 MMV2FeatureSchema::get_dimension() const {
    return m_schema.GetDimension();
}

bool MMV2FeatureSchema::is_valid() const {
    return m_schema.IsValid();
}

Dictionary MMV2FeatureSchema::get_channel_info(int32 index) const {
    Dictionary result;
    if (index < 0 || index >= m_schema.channels.Size()) return result;

    const MMV2::FeatureChannelDesc& ch = m_schema.channels[index];
    result["type"] = static_cast<int32>(ch.type);
    result["bone_index"] = ch.boneIndex;
    result["trajectory_index"] = ch.trajectoryIndex;
    result["dimension"] = ch.dimension;
    result["weight"] = ch.weight;
    result["enabled"] = ch.enabled;

    return result;
}

PackedStringArray MMV2FeatureSchema::get_channel_names() const {
    PackedStringArray names;
    for (const auto& ch : m_schema.channels) {
        String name;
        switch (ch.type) {
            case MMV2::FeatureChannelType::BonePosition: name = "BonePosition"; break;
            case MMV2::FeatureChannelType::BoneRotation: name = "BoneRotation"; break;
            case MMV2::FeatureChannelType::BoneVelocity: name = "BoneVelocity"; break;
            case MMV2::FeatureChannelType::TrajectoryPosition: name = "TrajectoryPosition"; break;
            case MMV2::FeatureChannelType::TrajectoryVelocity: name = "TrajectoryVelocity"; break;
            case MMV2::FeatureChannelType::TrajectoryDirection: name = "TrajectoryDirection"; break;
            case MMV2::FeatureChannelType::RootVelocity: name = "RootVelocity"; break;
            case MMV2::FeatureChannelType::RootHeight: name = "RootHeight"; break;
            case MMV2::FeatureChannelType::FootContact: name = "FootContact"; break;
            case MMV2::FeatureChannelType::Phase: name = "Phase"; break;
            default: name = "Unknown"; break;
        }
        names.append(name);
    }
    return names;
}
