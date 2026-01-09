#pragma once

#include <cstdint>

class PlanInstance;

namespace mr_planner::vamp_plugin
{
// Bump when the plugin interface changes in a breaking way.
inline constexpr std::uint32_t kAbiVersion = 2;

struct Api
{
    std::uint32_t abi_version;
    const char *plugin_name;
    PlanInstance *(*create_instance)();
    void (*destroy_instance)(PlanInstance *);
};
}  // namespace mr_planner::vamp_plugin

extern "C"
{
// Required entrypoint for mr_planner VAMP plugins.
// The returned pointer must remain valid for the lifetime of the plugin.
const mr_planner::vamp_plugin::Api *mr_planner_vamp_plugin_get_api();
}
