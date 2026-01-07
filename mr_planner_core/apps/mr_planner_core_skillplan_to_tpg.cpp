#include <mr_planner/core/logger.h>
#include <mr_planner/execution/adg.h>
#include <mr_planner/execution/tpg.h>
#include <mr_planner/io/graph_proto.h>
#include <mr_planner/io/skillplan.h>
#include <mr_planner/visualization/meshcat_playback.h>

#if MR_PLANNER_WITH_VAMP
#include <mr_planner/backends/vamp_env_factory.h>
#endif

#include <filesystem>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

struct Args
{
    std::string skillplan_file;
    std::string graph_type{"adg"};  // tpg|adg
    std::string output_dir;
    std::string vamp_environment;
    double dt{0.0};
    bool dt_set{false};
    double vmax{1.0};
    double shortcut_time{1.0};
    std::string dot_file;
    std::string pb_file;
    bool meshcat{false};
    std::string meshcat_host{"127.0.0.1"};
    int meshcat_port{7600};
    double meshcat_rate{1.0};
};

void usage(const char *prog)
{
    std::cerr << "Usage: " << prog << " --skillplan <file.json> [options]\n"
              << "Options:\n"
              << "  --graph-type <tpg|adg>       Graph type to build (default: adg)\n"
              << "  --output-dir <path>          Output directory (default: skillplan's folder)\n"
              << "  --vamp-environment <name>    VAMP environment (default: from skillplan or dual_gp4)\n"
              << "  --dt <sec>                   Discretization timestep (default: 0.05/vmax)\n"
              << "  --vmax <val>                 Joint-space L1 vmax (default: 1.0)\n"
              << "  --shortcut-time <sec>        Shortcut runtime limit (default: 1.0; 0 disables)\n"
              << "  --dot <path>                 Write DOT file (optional)\n"
              << "  --pb <path>                  Write portable protobuf (default: <output-dir>/<graph>_from_skillplan.pb)\n"
              << "  --meshcat                    Visualize the resulting schedule via Meshcat\n"
              << "  --meshcat-host <host>        Meshcat bridge host (default: 127.0.0.1)\n"
              << "  --meshcat-port <port>        Meshcat bridge port (default: 7600)\n"
              << "  --meshcat-rate <x>           Playback rate (default: 1.0; 0 disables sleeping)\n";
}

bool parse_double(const std::string &s, double *out)
{
    if (!out)
    {
        return false;
    }
    try
    {
        *out = std::stod(s);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

Args parse_args(int argc, char **argv)
{
    Args args;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a(argv[i]);
        auto require_value = [&](const std::string &flag) -> std::string {
            if (i + 1 >= argc)
            {
                throw std::runtime_error("Missing value for " + flag);
            }
            return std::string(argv[++i]);
        };

        if (a == "--help" || a == "-h")
        {
            usage(argv[0]);
            std::exit(0);
        }
        else if (a == "--skillplan")
        {
            args.skillplan_file = require_value(a);
        }
        else if (a == "--graph-type")
        {
            args.graph_type = require_value(a);
        }
        else if (a == "--output-dir")
        {
            args.output_dir = require_value(a);
        }
        else if (a == "--vamp-environment")
        {
            args.vamp_environment = require_value(a);
        }
        else if (a == "--dt")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.dt) || args.dt <= 0.0)
            {
                throw std::runtime_error("Invalid --dt: " + v);
            }
            args.dt_set = true;
        }
        else if (a == "--vmax")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.vmax) || args.vmax <= 0.0)
            {
                throw std::runtime_error("Invalid --vmax: " + v);
            }
        }
        else if (a == "--shortcut-time")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.shortcut_time) || args.shortcut_time < 0.0)
            {
                throw std::runtime_error("Invalid --shortcut-time: " + v);
            }
        }
        else if (a == "--dot")
        {
            args.dot_file = require_value(a);
        }
        else if (a == "--pb")
        {
            args.pb_file = require_value(a);
        }
        else if (a == "--meshcat")
        {
            args.meshcat = true;
        }
        else if (a == "--meshcat-host")
        {
            args.meshcat_host = require_value(a);
        }
        else if (a == "--meshcat-port")
        {
            const std::string v = require_value(a);
            if (v.empty())
            {
                throw std::runtime_error("Invalid --meshcat-port: empty");
            }
            args.meshcat_port = std::stoi(v);
            if (args.meshcat_port <= 0 || args.meshcat_port > 65535)
            {
                throw std::runtime_error("Invalid --meshcat-port: " + v);
            }
        }
        else if (a == "--meshcat-rate")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.meshcat_rate) || args.meshcat_rate < 0.0)
            {
                throw std::runtime_error("Invalid --meshcat-rate: " + v);
            }
        }
        else
        {
            throw std::runtime_error("Unknown argument: " + a);
        }
    }

    if (args.skillplan_file.empty())
    {
        throw std::runtime_error("Missing required --skillplan");
    }

    if (args.graph_type != "tpg" && args.graph_type != "adg")
    {
        throw std::runtime_error("Invalid --graph-type: " + args.graph_type);
    }

    if (!args.dt_set)
    {
        args.dt = 0.05 / std::max(args.vmax, 1e-6);
    }

    return args;
}

std::string default_output_dir(const std::string &skillplan_file)
{
    std::filesystem::path path(skillplan_file);
    if (path.has_parent_path())
    {
        return path.parent_path().string();
    }
    return ".";
}

std::vector<std::string> robot_names_from_skillplan(const Json::Value &plan)
{
    std::vector<std::string> names;
    const auto robots = plan["robots"];
    if (!robots.isArray())
    {
        return names;
    }
    names.reserve(robots.size());
    for (Json::ArrayIndex i = 0; i < robots.size(); ++i)
    {
        const auto &r = robots[i];
        if (r.isObject() && r["name"].isString())
        {
            names.push_back(r["name"].asString());
        }
        else
        {
            names.push_back("robot_" + std::to_string(i));
        }
    }
    return names;
}

std::vector<int> robot_dofs_from_skillplan(const Json::Value &plan)
{
    std::vector<int> dofs;
    const auto robots = plan["robots"];
    if (!robots.isArray())
    {
        return dofs;
    }
    dofs.reserve(robots.size());
    for (Json::ArrayIndex i = 0; i < robots.size(); ++i)
    {
        const auto &r = robots[i];
        if (r.isObject() && r["dof"].isInt())
        {
            dofs.push_back(r["dof"].asInt());
        }
        else
        {
            dofs.push_back(0);
        }
    }
    return dofs;
}

std::string normalize_activity_key(const std::string &name)
{
    std::string out;
    out.reserve(name.size());

    bool last_was_us = false;
    for (unsigned char c : name)
    {
        if (std::isalnum(c))
        {
            out.push_back(static_cast<char>(std::tolower(c)));
            last_was_us = false;
            continue;
        }
        if (c == '_' || c == '-' || c == ' ' || c == '/' || c == '.')
        {
            if (!out.empty() && !last_was_us)
            {
                out.push_back('_');
                last_was_us = true;
            }
        }
    }

    while (!out.empty() && out.back() == '_')
    {
        out.pop_back();
    }
    return out;
}

Activity::Type activity_type_from_action(const Json::Value &action)
{
    static const std::unordered_map<std::string, Activity::Type> kTypeByName = [] {
        std::unordered_map<std::string, Activity::Type> map;
        for (const auto &kv : Activity::enumStringMap)
        {
            map.emplace(kv.second, kv.first);
        }
        return map;
    }();

    std::string raw;
    if (action.isMember("activity") && action["activity"].isObject() && action["activity"]["type"].isString())
    {
        raw = action["activity"]["type"].asString();
    }
    else if (action.isMember("activity_type") && action["activity_type"].isString())
    {
        raw = action["activity_type"].asString();
    }
    else if (action.isMember("skill") && action["skill"].isObject() && action["skill"]["name"].isString())
    {
        raw = action["skill"]["name"].asString();
    }
    else if (action.isMember("meta_skill") && action["meta_skill"].isObject() && action["meta_skill"]["name"].isString())
    {
        raw = action["meta_skill"]["name"].asString();
    }
    else if (action.isMember("key") && action["key"].isString())
    {
        raw = action["key"].asString();
    }

    if (raw.empty())
    {
        return Activity::Type::pick;
    }

    const std::string norm = normalize_activity_key(raw);
    const auto it = kTypeByName.find(norm);
    if (it != kTypeByName.end())
    {
        return it->second;
    }
    return Activity::Type::pick;
}

bool parse_object_json(const Json::Value &v, Object *out, bool *vanish, bool *fixed, bool *handover);

void seed_initial_scene(const Json::Value &plan,
                        std::unordered_map<std::string, ObjPtr> *last_obj_by_name,
                        ActivityGraph *act_graph,
                        PlanInstance &instance)
{
    if (!plan.isObject() || !plan.isMember("initial_scene") || !plan["initial_scene"].isObject())
    {
        return;
    }
    const auto init_objects = plan["initial_scene"]["objects"];
    if (!init_objects.isArray())
    {
        return;
    }

    for (Json::ArrayIndex i = 0; i < init_objects.size(); ++i)
    {
        Object obj;
        bool vanish = false;
        bool fixed = false;
        bool handover = false;
        if (!parse_object_json(init_objects[i], &obj, &vanish, &fixed, &handover))
        {
            continue;
        }

        if (act_graph && last_obj_by_name)
        {
            ObjPtr node = act_graph->add_obj(obj);
            node->vanish = vanish;
            node->fixed = fixed;
            node->handover = handover;
            (*last_obj_by_name)[obj.name] = node;
        }

        if (vanish)
        {
            continue;
        }

        try
        {
            if (instance.hasObject(obj.name))
            {
                instance.moveObject(obj);
            }
            else
            {
                instance.addMoveableObject(obj);
            }
        }
        catch (const std::exception &)
        {
            // Ignore objects unsupported by the backend (e.g., non-Box in VAMP).
        }
    }
    instance.updateScene();
}

bool parse_number(const Json::Value &v, double *out)
{
    if (!out || !v.isNumeric())
    {
        return false;
    }
    *out = v.asDouble();
    return true;
}

bool parse_pose_xyzquat(const Json::Value &pose,
                        double *x,
                        double *y,
                        double *z,
                        double *qx,
                        double *qy,
                        double *qz,
                        double *qw)
{
    if (!pose.isObject() || !pose["position"].isObject() || !pose["orientation"].isObject())
    {
        return false;
    }
    const auto &p = pose["position"];
    const auto &q = pose["orientation"];
    return parse_number(p["x"], x) && parse_number(p["y"], y) && parse_number(p["z"], z) && parse_number(q["x"], qx) &&
           parse_number(q["y"], qy) && parse_number(q["z"], qz) && parse_number(q["w"], qw);
}

Object::Shape shape_from_string(const std::string &s)
{
    const std::string k = normalize_activity_key(s);
    if (k == "sphere")
    {
        return Object::Shape::Sphere;
    }
    if (k == "cylinder")
    {
        return Object::Shape::Cylinder;
    }
    if (k == "mesh")
    {
        return Object::Shape::Mesh;
    }
    return Object::Shape::Box;
}

Object::State state_from_string(const std::string &s)
{
    const std::string k = normalize_activity_key(s);
    if (k == "attached")
    {
        return Object::State::Attached;
    }
    if (k == "supported")
    {
        return Object::State::Supported;
    }
    if (k == "handover")
    {
        return Object::State::Handover;
    }
    return Object::State::Static;
}

bool parse_object_json(const Json::Value &v,
                       Object *out,
                       bool *vanish,
                       bool *fixed,
                       bool *handover)
{
    if (!out || !v.isObject() || !v["name"].isString())
    {
        return false;
    }

    Object obj;
    obj.name = v["name"].asString();
    obj.parent_link = v.isMember("parent_link") && v["parent_link"].isString() ? v["parent_link"].asString() : "world";
    obj.robot_id = v.isMember("robot_id") && v["robot_id"].isInt() ? v["robot_id"].asInt() : -1;

    if (v.isMember("shape") && v["shape"].isString())
    {
        obj.shape = shape_from_string(v["shape"].asString());
    }
    if (v.isMember("state") && v["state"].isString())
    {
        obj.state = state_from_string(v["state"].asString());
    }

    if (v.isMember("pose"))
    {
        parse_pose_xyzquat(v["pose"], &obj.x, &obj.y, &obj.z, &obj.qx, &obj.qy, &obj.qz, &obj.qw);
    }
    if (v.isMember("attach_pose"))
    {
        parse_pose_xyzquat(v["attach_pose"],
                           &obj.x_attach,
                           &obj.y_attach,
                           &obj.z_attach,
                           &obj.qx_attach,
                           &obj.qy_attach,
                           &obj.qz_attach,
                           &obj.qw_attach);
    }

    if (v.isMember("dimensions") && v["dimensions"].isObject())
    {
        const auto &d = v["dimensions"];
        double dx = 0.0;
        double dy = 0.0;
        double dz = 0.0;
        parse_number(d["x"], &dx);
        parse_number(d["y"], &dy);
        parse_number(d["z"], &dz);
        obj.length = dx;
        obj.width = dy;
        obj.height = dz;
        if (d.isMember("radius") && d["radius"].isNumeric())
        {
            obj.radius = d["radius"].asDouble();
        }
    }
    if (v.isMember("mesh_path") && v["mesh_path"].isString())
    {
        obj.mesh_path = v["mesh_path"].asString();
    }

    bool obj_vanish = false;
    bool obj_fixed = false;
    bool obj_handover = false;
    if (v.isMember("mr_planner") && v["mr_planner"].isObject())
    {
        const auto &m = v["mr_planner"];
        if (m.isMember("vanish") && m["vanish"].isBool())
        {
            obj_vanish = m["vanish"].asBool();
        }
        if (m.isMember("fixed") && m["fixed"].isBool())
        {
            obj_fixed = m["fixed"].asBool();
        }
        if (m.isMember("handover") && m["handover"].isBool())
        {
            obj_handover = m["handover"].asBool();
        }
    }
    if (v.isMember("lego") && v["lego"].isObject())
    {
        const auto &lego = v["lego"];
        if (lego.isMember("fixed") && lego["fixed"].isBool())
        {
            obj_fixed = lego["fixed"].asBool();
        }
        if (lego.isMember("in_storage") && lego["in_storage"].isBool())
        {
            obj_vanish = lego["in_storage"].asBool();
        }
    }

    *out = std::move(obj);
    if (vanish)
    {
        *vanish = obj_vanish;
    }
    if (fixed)
    {
        *fixed = obj_fixed;
    }
    if (handover)
    {
        *handover = obj_handover;
    }
    return true;
}

std::string vamp_environment_from_skillplan(const Json::Value &plan)
{
    if (!plan.isObject() || !plan.isMember("environment"))
    {
        return {};
    }
    const auto env = plan["environment"];
    if (env.isObject() && env["name"].isString())
    {
        return env["name"].asString();
    }
    return {};
}

}  // namespace

int main(int argc, char **argv)
{
    Args args;
    try
    {
        args = parse_args(argc, argv);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[error] " << e.what() << "\n";
        usage(argv[0]);
        return 2;
    }

#if !MR_PLANNER_WITH_VAMP
    std::cerr << "[error] mr_planner_core was built without VAMP support; this tool requires VAMP.\n";
    return 2;
#else
    Json::Value plan;
    std::string err;
    if (!mr_planner::skillplan::read_json_from_file(args.skillplan_file, &plan, &err))
    {
        std::cerr << "[error] failed to read skillplan '" << args.skillplan_file << "': " << err << "\n";
        return 2;
    }

    const auto robot_names = robot_names_from_skillplan(plan);
    const auto robot_dofs = robot_dofs_from_skillplan(plan);
    const int num_robots = static_cast<int>(robot_names.size());
    if (num_robots <= 0)
    {
        std::cerr << "[error] skillplan robots[] is empty\n";
        return 2;
    }

    MRTrajectory trajectories;
    if (!mr_planner::skillplan::extract_robot_trajectories(plan, &trajectories, &err))
    {
        std::cerr << "[error] failed to extract trajectories from skillplan: " << err << "\n";
        return 2;
    }
    if (static_cast<int>(trajectories.size()) != num_robots)
    {
        std::cerr << "[error] trajectory robot count mismatch: robots[]=" << num_robots
                  << ", trajectories=" << trajectories.size() << " (robot ids must be contiguous 0..N-1)\n";
        return 2;
    }

    // For TPG construction from sequential skillplans (many actions), build a synchronized trajectory by
    // replaying actions[] in file order and holding the non-active robots stationary.
    MRTrajectory tpg_trajectories = trajectories;
    if (args.graph_type == "tpg")
    {
        int max_actions_per_robot = 0;
        std::vector<int> counts(static_cast<std::size_t>(num_robots), 0);
        const auto actions_json = plan["actions"];
        if (actions_json.isArray())
        {
            for (Json::ArrayIndex i = 0; i < actions_json.size(); ++i)
            {
                const auto &a = actions_json[i];
                if (!a.isObject() || !a["robot"].isObject() || !a["robot"]["id"].isInt())
                {
                    continue;
                }
                const int rid = a["robot"]["id"].asInt();
                if (rid >= 0 && rid < num_robots)
                {
                    counts[static_cast<std::size_t>(rid)]++;
                    max_actions_per_robot = std::max(max_actions_per_robot, counts[static_cast<std::size_t>(rid)]);
                }
            }
        }
        if (max_actions_per_robot > 1)
        {
            if (!mr_planner::skillplan::extract_synchronized_trajectory(plan, &tpg_trajectories, &err))
            {
                std::cerr << "[warn] failed to build synchronized trajectory from actions[]: " << err << "\n";
                tpg_trajectories = trajectories;
            }
        }
    }

    if (args.output_dir.empty())
    {
        args.output_dir = default_output_dir(args.skillplan_file);
    }

    if (args.pb_file.empty())
    {
        args.pb_file = (std::filesystem::path(args.output_dir) /
                        (args.graph_type == "adg" ? "adg_from_skillplan.pb" : "tpg_from_skillplan.pb"))
                           .string();
    }

    if (!args.dot_file.empty())
    {
        // Use as-is.
    }
    else if (!args.output_dir.empty())
    {
        // Default to skipping dot output unless explicitly requested.
    }

    std::filesystem::create_directories(args.output_dir);

    std::string resolved_environment = args.vamp_environment;
    if (resolved_environment.empty())
    {
        resolved_environment = vamp_environment_from_skillplan(plan);
    }
    if (resolved_environment.empty())
    {
        resolved_environment = "dual_gp4";
    }

    std::optional<vamp_env::EnvironmentConfig> config_opt = vamp_env::make_environment_config(resolved_environment);
    if (!config_opt && args.vamp_environment.empty() && resolved_environment != "dual_gp4")
    {
        const std::string fallback = "dual_gp4";
        config_opt = vamp_env::make_environment_config(fallback);
        if (config_opt)
        {
            std::cerr << "[warn] unsupported VAMP environment in skillplan ('" << resolved_environment
                      << "'); falling back to '" << fallback << "'\n";
            resolved_environment = fallback;
        }
    }
    if (!config_opt)
    {
        std::cerr << "[error] unsupported VAMP environment: " << resolved_environment << "\n";
        return 2;
    }

    auto instance = vamp_env::make_vamp_instance(*config_opt);
    if (!instance)
    {
        std::cerr << "[error] failed to construct VAMP instance for environment: " << resolved_environment << "\n";
        return 2;
    }

    instance->setNumberOfRobots(num_robots);
    instance->setRobotNames(robot_names);
    for (int i = 0; i < num_robots; ++i)
    {
        const int dof = (i < static_cast<int>(robot_dofs.size())) ? robot_dofs[i] : 0;
        if (dof > 0)
        {
            instance->setRobotDOF(i, static_cast<std::size_t>(dof));
        }
    }
    instance->setVmax(args.vmax);

    vamp_env::add_environment_obstacles(*config_opt, *instance);
    vamp_env::add_environment_attachments(*config_opt, *instance);

    if (auto defaults = vamp_env::default_base_transforms(*config_opt))
    {
        if (defaults->size() == static_cast<std::size_t>(num_robots))
        {
            for (int i = 0; i < num_robots; ++i)
            {
                instance->setRobotBaseTransform(i, (*defaults)[i]);
            }
        }
    }

    for (int i = 0; i < num_robots; ++i)
    {
        const auto &traj = (args.graph_type == "tpg") ? tpg_trajectories[static_cast<std::size_t>(i)]
                                                      : trajectories[static_cast<std::size_t>(i)];
        if (!traj.trajectory.empty())
        {
            instance->setStartPose(i, traj.trajectory.front().joint_values);
            instance->setGoalPose(i, traj.trajectory.back().joint_values);
        }
    }

    tpg::TPGConfig tpg_config;
    tpg_config.dt = args.dt;
    tpg_config.output_dir = args.output_dir;
    tpg_config.parallel = false;
    tpg_config.use_sweep_type2 = true;
    tpg_config.shortcut_time = args.shortcut_time;
    // Match LEGO planning defaults: do not force cross-robot synchronous task execution.
    tpg_config.sync_task = false;
    // Keep the default pruning of skippable activities during ADG reconstruction (matches LEGO planning).
    // This avoids zero-span "home" / "home_handover" segments causing cycles when rebuilding from sequential skillplans.
    tpg_config.preserve_skippable_acts = false;

    if (args.graph_type == "adg")
    {
        const auto actions_json = plan["actions"];
        if (!actions_json.isArray())
        {
            std::cerr << "[error] skillplan actions[] must be an array\n";
            return 2;
        }

        struct ActKey
        {
            int robot_id{-1};
            int act_id{-1};
        };
        std::unordered_map<std::string, ActKey> action_to_act;
        std::unordered_map<std::string, const Json::Value *> action_json_by_id;
        std::vector<std::vector<std::string>> actions_by_robot(static_cast<std::size_t>(num_robots));
        std::vector<int> next_act_id(static_cast<std::size_t>(num_robots), 0);

        for (Json::ArrayIndex i = 0; i < actions_json.size(); ++i)
        {
            const auto &action = actions_json[i];
            if (!action.isObject() || !action["id"].isString() || !action["robot"].isObject() ||
                !action["robot"]["id"].isInt())
            {
                std::cerr << "[error] actions[" << i << "] missing required fields (id, robot.id)\n";
                return 2;
            }
            const std::string aid = action["id"].asString();
            const int rid = action["robot"]["id"].asInt();
            if (rid < 0 || rid >= num_robots)
            {
                std::cerr << "[error] actions[" << i << "].robot.id=" << rid << " out of range\n";
                return 2;
            }
            const int act_id = next_act_id[static_cast<std::size_t>(rid)]++;
            action_to_act[aid] = ActKey{rid, act_id};
            action_json_by_id.emplace(aid, &action);
            actions_by_robot[static_cast<std::size_t>(rid)].push_back(aid);
        }

        auto act_graph = std::make_shared<ActivityGraph>(num_robots);
        std::vector<std::vector<ActPtr>> acts(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            const auto &act_ids = actions_by_robot[static_cast<std::size_t>(rid)];
            if (act_ids.empty())
            {
                std::cerr << "[error] no actions found for robot_id=" << rid << "\n";
                return 2;
            }
            acts[static_cast<std::size_t>(rid)].reserve(act_ids.size());
            for (int act_id = 0; act_id < static_cast<int>(act_ids.size()); ++act_id)
            {
                const std::string &aid = act_ids[static_cast<std::size_t>(act_id)];
                Activity::Type type = Activity::Type::pick;
                const auto ait = action_json_by_id.find(aid);
                if (ait != action_json_by_id.end() && ait->second && ait->second->isObject())
                {
                    type = activity_type_from_action(*ait->second);
                }
                auto act = act_graph->add_act(rid, type);
                acts[static_cast<std::size_t>(rid)].push_back(act);
            }
        }

        // Seed object nodes + instance scene from initial_scene.objects (optional).
        std::unordered_map<std::string, ObjPtr> last_obj_by_name;
        seed_initial_scene(plan, &last_obj_by_name, act_graph.get(), *instance);

        for (int rid = 0; rid < num_robots; ++rid)
        {
            const auto &traj = trajectories[static_cast<std::size_t>(rid)];
            if (traj.trajectory.size() != traj.act_ids.size())
            {
                std::cerr << "[error] robot " << rid << " trajectory.act_ids size mismatch\n";
                return 2;
            }
            const int num_acts = static_cast<int>(acts[static_cast<std::size_t>(rid)].size());
            for (int act_id = 0; act_id < num_acts; ++act_id)
            {
                int first = -1;
                int last = -1;
                for (std::size_t k = 0; k < traj.act_ids.size(); ++k)
                {
                    if (traj.act_ids[k] != act_id)
                    {
                        continue;
                    }
                    if (first < 0)
                    {
                        first = static_cast<int>(k);
                    }
                    last = static_cast<int>(k);
                }
                if (first < 0 || last < 0)
                {
                    std::cerr << "[error] robot " << rid << " missing trajectory samples for act_id=" << act_id << "\n";
                    return 2;
                }
                auto &act = acts[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)];
                act->start_pose = traj.trajectory[static_cast<std::size_t>(first)];
                act->end_pose = traj.trajectory[static_cast<std::size_t>(last)];
            }
        }

        for (Json::ArrayIndex i = 0; i < actions_json.size(); ++i)
        {
            const auto &action = actions_json[i];
            if (!action.isObject() || !action["id"].isString())
            {
                continue;
            }
            const std::string aid = action["id"].asString();
            const auto it = action_to_act.find(aid);
            if (it == action_to_act.end())
            {
                continue;
            }
            const int rid = it->second.robot_id;
            const int act_id = it->second.act_id;
            if (rid < 0 || rid >= num_robots)
            {
                continue;
            }
            if (act_id < 0 || act_id >= static_cast<int>(acts[static_cast<std::size_t>(rid)].size()))
            {
                continue;
            }

            // Apply scene updates (objects + attachments + collision overrides) for this activity.
            if (action.isMember("scene_updates") && action["scene_updates"].isObject())
            {
                const auto &scene = action["scene_updates"];
                if (scene.isMember("objects") && scene["objects"].isObject())
                {
                    const auto &obj_updates = scene["objects"]["update"];
                    if (obj_updates.isArray())
                    {
                        for (Json::ArrayIndex u = 0; u < obj_updates.size(); ++u)
                        {
                            Object obj;
                            bool vanish = false;
                            bool fixed = false;
                            bool handover = false;
                            if (!parse_object_json(obj_updates[u], &obj, &vanish, &fixed, &handover))
                            {
                                continue;
                            }
                            ObjPtr node = act_graph->add_obj(obj);
                            node->vanish = vanish;
                            node->fixed = fixed;
                            node->handover = handover;
                            last_obj_by_name[obj.name] = node;

                            if (!vanish)
                            {
                                try
                                {
                                    if (instance->hasObject(obj.name))
                                    {
                                        instance->moveObject(obj);
                                    }
                                    else
                                    {
                                        instance->addMoveableObject(obj);
                                    }
                                }
                                catch (const std::exception &)
                                {
                                }
                            }
                        }
                        instance->updateScene();
                    }
                }

                const auto &attachments = scene["attachments"];
                if (attachments.isArray())
                {
                    for (Json::ArrayIndex a = 0; a < attachments.size(); ++a)
                    {
                        const auto &att = attachments[a];
                        if (!att.isObject() || !att["action"].isString() || !att["object"].isString())
                        {
                            continue;
                        }
                        const std::string kind = normalize_activity_key(att["action"].asString());
                        const std::string obj_name = att["object"].asString();
                        const std::string link = att.isMember("link") && att["link"].isString() ? att["link"].asString() : "";
                        const auto oit = last_obj_by_name.find(obj_name);
                        if (oit == last_obj_by_name.end() || !oit->second)
                        {
                            continue;
                        }
                        if (kind == "attach")
                        {
                            act_graph->attach_obj(oit->second,
                                                  link,
                                                  acts[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)]);
                        }
                        else if (kind == "detach")
                        {
                            act_graph->detach_obj(oit->second,
                                                  acts[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)]);
                        }
                    }
                }

                const auto add_collision = [&](const Json::Value &entries, bool allow) {
                    if (!entries.isArray())
                    {
                        return;
                    }
                    for (Json::ArrayIndex c = 0; c < entries.size(); ++c)
                    {
                        const auto &e = entries[c];
                        if (!e.isObject() || !e["object"].isString() || !e["link"].isString())
                        {
                            continue;
                        }
                        act_graph->set_collision(
                            e["object"].asString(),
                            e["link"].asString(),
                            acts[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)],
                            allow);
                    }
                };
                add_collision(scene["collisions_allow"], true);
                add_collision(scene["collisions_disallow"], false);
            }

            const auto deps = action["depends_on"];
            if (!deps.isArray())
            {
                continue;
            }
            for (const auto &dep : deps)
            {
                if (!dep.isString())
                {
                    continue;
                }
                const auto dep_it = action_to_act.find(dep.asString());
                if (dep_it == action_to_act.end())
                {
                    continue;
                }
                const int dep_rid = dep_it->second.robot_id;
                const int dep_act_id = dep_it->second.act_id;
                if (dep_rid == rid)
                {
                    if (dep_act_id > act_id)
                    {
                        std::cerr << "[error] same-robot dependency violates per-robot action order: action '" << aid
                                  << "' depends on '" << dep.asString() << "' but appears before it (robot_id=" << rid
                                  << ")\n";
                        return 2;
                    }
                    continue;
                }
                if (dep_rid < 0 || dep_rid >= num_robots)
                {
                    continue;
                }
                if (dep_act_id < 0 ||
                    dep_act_id >= static_cast<int>(acts[static_cast<std::size_t>(dep_rid)].size()))
                {
                    continue;
                }
                act_graph->add_type2_dep(acts[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)],
                                         acts[static_cast<std::size_t>(dep_rid)][static_cast<std::size_t>(dep_act_id)]);
            }
        }

        auto adg = std::make_shared<tpg::ADG>(act_graph);
        if (!adg->init_from_asynctrajs(instance, tpg_config, trajectories))
        {
            std::cerr << "[error] failed to build ADG from skillplan trajectories\n";
            return 2;
        }
        if (!adg->shiftPolicyNodeType2Edges())
        {
            std::cerr << "[error] ADG contains a cycle after shifting policy node type2 edges\n";
            return 2;
        }
        if (!adg->optimize(instance, tpg_config))
        {
            std::cerr << "[error] ADG optimize failed\n";
            return 2;
        }

        if (!args.dot_file.empty())
        {
            adg->saveToDotFile(args.dot_file);
        }

        std::string pb_err;
        if (!mr_planner::graph_proto::write_adg(*adg, args.pb_file, &pb_err))
        {
            std::cerr << "[warn] failed to write ADG protobuf to " << args.pb_file << ": " << pb_err << "\n";
        }
        else
        {
            log("Wrote ADG protobuf to " + args.pb_file, LogLevel::INFO);
        }

        if (args.meshcat)
        {
            instance->enableMeshcat(args.meshcat_host, static_cast<std::uint16_t>(args.meshcat_port));
            // Always reset the Meshcat viewer and restore environment + initial objects.
            instance->resetScene(true);
            vamp_env::add_environment_obstacles(*config_opt, *instance);
            vamp_env::add_environment_attachments(*config_opt, *instance);
            seed_initial_scene(plan, nullptr, nullptr, *instance);

            mr_planner::visualization::MeshcatPlaybackOptions viz;
            viz.real_time_rate = args.meshcat_rate;
            if (!mr_planner::visualization::play_schedule(*instance, *adg, viz))
            {
                log("Meshcat playback ended early (schedule execution failed or exceeded max ticks)", LogLevel::WARN);
            }
        }

        return 0;
    }

    tpg::TPG tpg;
    if (!tpg.init(instance, tpg_trajectories, tpg_config))
    {
        std::cerr << "[error] failed to build TPG from skillplan trajectories\n";
        return 2;
    }

    if (!args.dot_file.empty())
    {
        tpg.saveToDotFile(args.dot_file);
    }

    std::string pb_err;
    if (!mr_planner::graph_proto::write_tpg(tpg, args.pb_file, &pb_err))
    {
        std::cerr << "[warn] failed to write TPG protobuf to " << args.pb_file << ": " << pb_err << "\n";
    }
    else
    {
        log("Wrote TPG protobuf to " + args.pb_file, LogLevel::INFO);
    }

        if (args.meshcat)
        {
            instance->enableMeshcat(args.meshcat_host, static_cast<std::uint16_t>(args.meshcat_port));
            mr_planner::visualization::MeshcatPlaybackOptions viz;
            viz.real_time_rate = args.meshcat_rate;
            instance->resetScene(true);
            vamp_env::add_environment_obstacles(*config_opt, *instance);
            vamp_env::add_environment_attachments(*config_opt, *instance);
            seed_initial_scene(plan, nullptr, nullptr, *instance);

            if (!mr_planner::visualization::play_schedule(*instance, tpg, viz))
            {
                log("Meshcat playback ended early (schedule execution failed or exceeded max ticks)", LogLevel::WARN);
            }
        }

    return 0;
#endif
}
