// ============================================================================
// MMV2 GDExtension - Database Resource
// ============================================================================

#pragma once
#ifndef MMV2_GODOT_DATABASE_H
#define MMV2_GODOT_DATABASE_H

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include "MMV2/Database/Database.h"

using namespace godot;

class MMV2Database : public Resource {
    GDCLASS(MMV2Database, Resource)

private:
    MMV2::MotionDatabase* m_database;
    bool m_isBuilt;

protected:
    static void _bind_methods();

public:
    MMV2Database();
    ~MMV2Database();

    bool build_from_clips(const TypedArray<Dictionary>& clips, const Dictionary& settings);
    bool load_from_file(const String& path);
    bool save_to_file(const String& path) const;

    int32 get_entry_count() const;
    int32 get_clip_count() const;
    Dictionary get_entry(int32 index) const;
    Dictionary get_clip(int32 index) const;

    int32 find_nearest(const PackedFloat32Array& query_features) const;
    PackedInt32Array find_k_nearest(const PackedFloat32Array& query_features, int32 k) const;

    bool is_built() const { return m_isBuilt; }
    void clear();

    Dictionary get_stats() const;

    // Internal access
    MMV2::MotionDatabase* get_native() const { return m_database; }
};

#endif
