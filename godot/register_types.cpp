// ============================================================================
// MMV2 GDExtension - Register Types Implementation
// ============================================================================

#include "godot/register_types.h"

#include "godot/mmv2_database.h"
#include "godot/mmv2_controller.h"
#include "godot/mmv2_feature_schema.h"
#include "godot/mmv2_pose_resource.h"

using namespace godot;

void initialize_mm2_module(ModuleInitializationLevel p_level) {
    if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
        ClassDB::register_class<MMV2Database>();
        ClassDB::register_class<MMV2Controller>();
        ClassDB::register_class<MMV2FeatureSchema>();
        ClassDB::register_class<MMV2PoseResource>();
    }
}

void uninitialize_mm2_module(ModuleInitializationLevel p_level) {
    if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
    }
}

extern "C" {
    GDExtensionBool GDE_EXPORT mmv2_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
        godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
        init_obj.register_initializer(initialize_mm2_module);
        init_obj.register_terminator(uninitialize_mm2_module);
        init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
        return init_obj.init();
    }
}
