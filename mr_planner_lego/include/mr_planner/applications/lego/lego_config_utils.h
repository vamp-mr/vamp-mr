#pragma once

#include <mr_planner/applications/lego/lego/Lego.hpp>

#include <jsoncpp/json/json.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace lego_manipulation
{
namespace lego
{
inline std::string resolveRepoPath(const std::string &root_pwd, const std::string &path)
{
    if (path.empty() || root_pwd.empty())
    {
        return path;
    }

    auto strip_trailing_slash = [](const std::string &s) -> std::string {
        if (!s.empty() && s.back() == '/')
        {
            return s.substr(0, s.size() - 1);
        }
        return s;
    };

    // Backward-compatible behavior: many configs store repo-relative paths as "/config/...".
    // Treat those as relative to root_pwd.
    if (!path.empty() && path.front() == '/')
    {
        if (path.rfind("/config/", 0) == 0 || path.rfind("/outputs/", 0) == 0 || path.rfind("/proto/", 0) == 0)
        {
            return strip_trailing_slash(root_pwd) + path;
        }
        return path;  // absolute filesystem path
    }

    return strip_trailing_slash(root_pwd) + "/" + path;
}

inline std::vector<RobotCalibration> loadRobotCalibrations(const Json::Value &config,
                                                           const std::string &root_pwd)
{
    auto resolve = [&](const Json::Value &node, const std::string &key) -> std::string
    {
        if (!node.isMember(key))
        {
            throw std::runtime_error("Robot calibration missing key '" + key + "'");
        }
        return resolveRepoPath(root_pwd, node[key].asString());
    };

    std::vector<RobotCalibration> robots;
    if (config.isMember("robots") && config["robots"].isArray() && !config["robots"].empty())
    {
        for (const auto &robot_cfg : config["robots"])
        {
            RobotCalibration calib;
            calib.dh_fname = resolve(robot_cfg, "DH_fname");
            calib.dh_tool_fname = resolve(robot_cfg, "DH_tool_fname");
            calib.dh_tool_disassemble_fname = resolve(robot_cfg, "DH_tool_disassemble_fname");
            calib.dh_tool_assemble_fname = resolve(robot_cfg, "DH_tool_assemble_fname");
            calib.dh_tool_alt_fname = resolve(robot_cfg, "DH_tool_alt_fname");
            calib.dh_tool_alt_assemble_fname = resolve(robot_cfg, "DH_tool_alt_assemble_fname");
            calib.dh_tool_handover_assemble_fname = resolve(robot_cfg, "DH_tool_handover_assemble_fname");
            calib.base_fname = resolve(robot_cfg, "base_fname");
            robots.push_back(calib);
        }
        return robots;
    }

    RobotCalibration r1;
    r1.dh_fname = resolve(config, "r1_DH_fname");
    r1.dh_tool_fname = resolve(config, "r1_DH_tool_fname");
    r1.dh_tool_disassemble_fname = resolve(config, "r1_DH_tool_disassemble_fname");
    r1.dh_tool_assemble_fname = resolve(config, "r1_DH_tool_assemble_fname");
    r1.dh_tool_alt_fname = resolve(config, "r1_DH_tool_alt_fname");
    r1.dh_tool_alt_assemble_fname = resolve(config, "r1_DH_tool_alt_assemble_fname");
    r1.dh_tool_handover_assemble_fname = resolve(config, "r1_DH_tool_handover_assemble_fname");
    r1.base_fname = resolve(config, "Robot1_Base_fname");
    robots.push_back(r1);

    RobotCalibration r2;
    r2.dh_fname = resolve(config, "r2_DH_fname");
    r2.dh_tool_fname = resolve(config, "r2_DH_tool_fname");
    r2.dh_tool_disassemble_fname = resolve(config, "r2_DH_tool_disassemble_fname");
    r2.dh_tool_assemble_fname = resolve(config, "r2_DH_tool_assemble_fname");
    r2.dh_tool_alt_fname = resolve(config, "r2_DH_tool_alt_fname");
    r2.dh_tool_alt_assemble_fname = resolve(config, "r2_DH_tool_alt_assemble_fname");
    r2.dh_tool_handover_assemble_fname = resolve(config, "r2_DH_tool_handover_assemble_fname");
    r2.base_fname = resolve(config, "Robot2_Base_fname");
    robots.push_back(r2);

    return robots;
}
}  // namespace lego
}  // namespace lego_manipulation
