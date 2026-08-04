// ============================================================================
// MMV2 GDExtension - Register Types
// ============================================================================

#pragma once
#ifndef MMV2_REGISTER_TYPES_H
#define MMV2_REGISTER_TYPES_H

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_mm2_module(ModuleInitializationLevel p_level);
void uninitialize_mm2_module(ModuleInitializationLevel p_level);

#endif
