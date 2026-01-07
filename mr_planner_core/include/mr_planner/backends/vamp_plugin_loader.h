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

