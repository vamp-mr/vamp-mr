#pragma once

#if !MR_PLANNER_WITH_VAMP
#error "vamp_env_factory.h included without VAMP support"
#endif

#include <Eigen/Geometry>

#include <optional>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

class PlanInstance;

namespace vamp_env
{
    enum class VampVariant
    {
        DualGp4,
        QuadGp4,
        PandaPair,
        PandaTriple,
        PandaQuad,
        PandaHex,
        PandaOct,
        PandaDec
    };

    struct EnvironmentConfig
    {
        std::string environment_name;
        std::string move_group;
        std::vector<std::string> robot_groups;
        std::vector<std::string> hand_groups;
        std::optional<VampVariant> vamp_variant;
        std::string vamp_plugin;
        std::optional<std::vector<Eigen::Isometry3d>> base_transforms;
    };

    // `name` can be a built-in environment name (e.g. "dual_gp4") or a path/name
    // that resolves to a JSON file describing a plugin-based environment.
    auto make_environment_config(const std::string &name) -> std::optional<EnvironmentConfig>;

    auto make_vamp_instance(VampVariant variant) -> std::shared_ptr<PlanInstance>;
    auto make_vamp_instance(const EnvironmentConfig &config) -> std::shared_ptr<PlanInstance>;
    void add_environment_obstacles(const EnvironmentConfig &config, PlanInstance &instance);

    void add_environment_attachments(const EnvironmentConfig &config, PlanInstance &instance);

    auto default_base_transforms(const EnvironmentConfig &config)
        -> std::optional<std::vector<Eigen::Isometry3d>>;
}  // namespace vamp_env
