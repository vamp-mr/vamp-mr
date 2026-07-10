// Runtime loader for VAMP plugin shared libraries (see vamp_plugin_api.h).
// Opens the library, validates the plugin ABI version, and wraps the plugin's
// PlanInstance in a shared_ptr that keeps the library loaded until destruction.
#pragma once

#include <memory>
#include <string>

class PlanInstance;

namespace mr_planner::vamp_plugin
{
// Loads a `mr_planner_vamp_plugin_get_api()` plugin shared library and returns
// an owning PlanInstance pointer. The plugin remains loaded until the instance
// is destroyed.
auto load_instance_from_library(const std::string &library_path) -> std::shared_ptr<PlanInstance>;
}  // namespace mr_planner::vamp_plugin

