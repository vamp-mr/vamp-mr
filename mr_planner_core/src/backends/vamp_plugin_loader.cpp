#include <mr_planner/backends/vamp_plugin_api.h>
#include <mr_planner/backends/vamp_plugin_loader.h>

#include <mr_planner/core/logger.h>

#include <stdexcept>
#include <utility>

#if defined(__unix__) || defined(__APPLE__)
#include <dlfcn.h>
#endif

namespace mr_planner::vamp_plugin
{
namespace
{
struct LoadedPlugin
{
#if defined(__unix__) || defined(__APPLE__)
    void *handle{nullptr};
#endif
    const Api *api{nullptr};
    std::string path;

    ~LoadedPlugin()
    {
#if defined(__unix__) || defined(__APPLE__)
        if (handle)
        {
            dlclose(handle);
        }
#endif
    }
};

auto require_symbol(void *handle, const char *name) -> void *
{
#if defined(__unix__) || defined(__APPLE__)
    dlerror();
    void *sym = dlsym(handle, name);
    const char *err = dlerror();
    if (!sym || err)
    {
        throw std::runtime_error(std::string("Missing symbol '") + name + "': " + (err ? err : "unknown error"));
    }
    return sym;
#else
    (void)handle;
    (void)name;
    throw std::runtime_error("Plugin loading is only supported on POSIX platforms");
#endif
}

auto open_plugin(const std::string &library_path) -> std::shared_ptr<LoadedPlugin>
{
#if defined(__unix__) || defined(__APPLE__)
    void *handle = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle)
    {
        const char *err = dlerror();
        throw std::runtime_error("Failed to dlopen plugin: " + library_path + " (" + (err ? err : "unknown") + ")");
    }

    auto get_api = reinterpret_cast<const Api *(*)()>(require_symbol(handle, "mr_planner_vamp_plugin_get_api"));
    const Api *api = get_api();
    if (!api)
    {
        dlclose(handle);
        throw std::runtime_error("mr_planner_vamp_plugin_get_api() returned null: " + library_path);
    }
    if (api->abi_version != kAbiVersion)
    {
        dlclose(handle);
        throw std::runtime_error("Plugin ABI mismatch for " + library_path + " (expected " + std::to_string(kAbiVersion) +
                                 ", got " + std::to_string(api->abi_version) + ")");
    }
    if (!api->create_instance)
    {
        dlclose(handle);
        throw std::runtime_error("Plugin create_instance is null: " + library_path);
    }
    if (!api->destroy_instance)
    {
        dlclose(handle);
        throw std::runtime_error("Plugin destroy_instance is null: " + library_path);
    }

    auto plugin = std::make_shared<LoadedPlugin>();
    plugin->handle = handle;
    plugin->api = api;
    plugin->path = library_path;
    return plugin;
#else
    (void)library_path;
    throw std::runtime_error("Plugin loading is only supported on POSIX platforms");
#endif
}
}  // namespace

auto load_instance_from_library(const std::string &library_path) -> std::shared_ptr<PlanInstance>
{
    auto plugin = open_plugin(library_path);
    PlanInstance *instance = plugin->api->create_instance();
    if (!instance)
    {
        throw std::runtime_error("Plugin returned null PlanInstance: " + library_path);
    }
    return std::shared_ptr<PlanInstance>(instance, [plugin = std::move(plugin)](PlanInstance *ptr) {
        try
        {
            plugin->api->destroy_instance(ptr);
        }
        catch (const std::exception &e)
        {
            log(std::string("Failed to destroy plugin instance: ") + e.what(), LogLevel::ERROR);
        }
    });
}
}  // namespace mr_planner::vamp_plugin

