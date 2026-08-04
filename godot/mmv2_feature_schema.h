// ============================================================================
// MMV2 GDExtension - Feature Schema Resource
// ============================================================================

#pragma once
#ifndef MMV2_GODOT_FEATURE_SCHEMA_H
#define MMV2_GODOT_FEATURE_SCHEMA_H

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/ref.hpp>
#include "MMV2/Features/Feature.h"

using namespace godot;

class MMV2FeatureSchema : public Resource {
    GDCLASS(MMV2FeatureSchema, Resource)

private:
    MMV2::FeatureSchema m_schema;

protected:
    static void _bind_methods();

public:
    MMV2FeatureSchema();
    ~MMV2FeatureSchema();

    void add_position_channel(int32 bone_index, float32 weight);
    void add_rotation_channel(int32 bone_index, float32 weight);
    void add_velocity_channel(int32 bone_index, float32 weight);
    void add_trajectory_position_channel(int32 sample_index, float32 weight);
    void add_trajectory_velocity_channel(int32 sample_index, float32 weight);
    void add_trajectory_direction_channel(int32 sample_index, float32 weight);
    void add_root_velocity_channel(float32 weight);
    void add_root_height_channel(float32 weight);
    void add_foot_contact_channel(float32 weight);
    void add_phase_channel(float32 weight);

    void clear_channels();
    int32 get_channel_count() const;
    int32 get_dimension() const;

    bool is_valid() const;

    Dictionary get_channel_info(int32 index) const;
    PackedStringArray get_channel_names() const;

    // Internal
    const MMV2::FeatureSchema& get_native() const { return m_schema; }
    void set_native(const MMV2::FeatureSchema& schema) { m_schema = schema; }
};

#endif
