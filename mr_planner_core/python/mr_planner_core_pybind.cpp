#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mr_planner/backends/vamp_env_factory.h>
#include <mr_planner/core/logger.h>
#include <mr_planner/execution/adg.h>
#include <mr_planner/execution/tpg.h>
#include <mr_planner/io/graph_proto.h>
#include <mr_planner/io/skillplan.h>
#include <mr_planner/planning/composite_rrt.h>
#include <mr_planner/planning/planner.h>
#include <mr_planner/planning/roadmap.h>
#include <mr_planner/planning/shortcutter.h>
#include <mr_planner/visualization/meshcat_playback.h>

#include "mr_planner_graph.pb.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <google/protobuf/util/json_util.h>

#include <chrono>
#include <array>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace
{

std::string read_file_binary(const std::string &path)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open())
    {
        throw std::runtime_error("Failed to open file for reading: " + path);
    }
    std::string data;
    ifs.seekg(0, std::ios::end);
    data.resize(static_cast<std::size_t>(ifs.tellg()));
    ifs.seekg(0, std::ios::beg);
    if (!data.empty())
    {
        ifs.read(&data[0], static_cast<std::streamsize>(data.size()));
    }
    if (!ifs.good() && !ifs.eof())
    {
        throw std::runtime_error("Failed while reading file: " + path);
    }
    return data;
}

void write_file_binary(const std::string &path, const std::string &data)
{
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open())
    {
        throw std::runtime_error("Failed to open file for writing: " + path);
    }
    ofs.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!ofs.good())
    {
        throw std::runtime_error("Failed while writing file: " + path);
    }
}

std::string make_temp_dir(const std::string &prefix)
{
    const auto base = std::filesystem::temp_directory_path();
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<std::uint64_t> dist;
    for (int i = 0; i < 50; ++i)
    {
        std::ostringstream oss;
        oss << prefix << "." << std::hex << dist(gen);
        const auto dir = base / oss.str();
        std::error_code ec;
        if (std::filesystem::create_directories(dir, ec) && !ec)
        {
            return dir.string();
        }
    }
    throw std::runtime_error("Failed to create a temporary directory under: " + base.string());
}

std::optional<std::vector<std::vector<double>>> maybe_joint_matrix(const py::object &obj)
{
    if (obj.is_none())
    {
        return std::nullopt;
    }
    return obj.cast<std::vector<std::vector<double>>>();
}

bool sample_easy_problem(std::shared_ptr<PlanInstance> instance,
                         int num_robots,
                         int dof,
                         int attempts,
                         double max_goal_dist,
                         int seed,
                         std::vector<std::vector<double>> *start,
                         std::vector<std::vector<double>> *goal,
                         std::string *error)
{
    if (!instance || !start || !goal)
    {
        return false;
    }
    if (num_robots <= 0 || dof <= 0)
    {
        if (error)
        {
            *error = "Invalid num_robots/dof for random problem";
        }
        return false;
    }
    if (attempts <= 0)
    {
        attempts = 1;
    }

    instance->setRandomSeed(static_cast<unsigned int>(seed));

    for (int i = 0; i < num_robots; ++i)
    {
        instance->setRobotDOF(i, static_cast<std::size_t>(dof));
    }

    std::vector<RobotPose> start_poses(num_robots);
    std::vector<RobotPose> goal_poses(num_robots);

    for (int attempt = 0; attempt < attempts; ++attempt)
    {
        bool sampled_all = true;
        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotPose pose = instance->initRobotPose(rid);
            if (!instance->sample(pose))
            {
                sampled_all = false;
                break;
            }
            start_poses[rid] = std::move(pose);
        }
        if (!sampled_all)
        {
            continue;
        }
        if (instance->checkCollision(start_poses, true))
        {
            continue;
        }

        bool goals_ok = true;
        for (int rid = 0; rid < num_robots && goals_ok; ++rid)
        {
            RobotPose candidate_goal;
            bool found = false;
            for (int goal_try = 0; goal_try < 50 && !found; ++goal_try)
            {
                RobotPose target = instance->initRobotPose(rid);
                if (!instance->sample(target))
                {
                    continue;
                }
                if (instance->steer(start_poses[rid], target, max_goal_dist, candidate_goal, 0.1))
                {
                    found = true;
                }
            }
            if (!found)
            {
                goals_ok = false;
                break;
            }
            goal_poses[rid] = candidate_goal;
        }
        if (!goals_ok)
        {
            continue;
        }
        if (instance->checkCollision(goal_poses, true))
        {
            continue;
        }

        const bool motion_has_collision = instance->checkMultiRobotMotion(start_poses, goal_poses, 0.1, true);
        if (motion_has_collision)
        {
            continue;
        }

        start->assign(num_robots, std::vector<double>());
        goal->assign(num_robots, std::vector<double>());
        for (int rid = 0; rid < num_robots; ++rid)
        {
            (*start)[rid] = start_poses[rid].joint_values;
            (*goal)[rid] = goal_poses[rid].joint_values;
        }
        return true;
    }

    if (error)
    {
        *error = "Failed to sample a collision-free start/goal pair";
    }
    return false;
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

bool ends_with(const std::string &s, const std::string &suffix)
{
    if (suffix.size() > s.size())
    {
        return false;
    }
    return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

std::optional<std::string> infer_vamp_environment_from_robot_names(const std::vector<std::string> &robot_names)
{
    static const std::vector<std::string> kCandidates = {"dual_gp4",
                                                         "quad_gp4",
                                                         "panda_two",
                                                         "panda_two_rod",
                                                         "panda_three",
                                                         "panda_four",
                                                         "panda_four_bins",
                                                         "panda_six"};

    for (const auto &name : kCandidates)
    {
        const auto cfg = vamp_env::make_environment_config(name);
        if (cfg && cfg->robot_groups == robot_names)
        {
            return name;
        }
    }

    static const std::unordered_set<std::string> kGp4Groups = {"left_arm", "right_arm", "top_arm", "bottom_arm"};
    bool all_gp4 = !robot_names.empty();
    for (const auto &r : robot_names)
    {
        if (kGp4Groups.find(r) == kGp4Groups.end())
        {
            all_gp4 = false;
            break;
        }
    }
    if (all_gp4)
    {
        if (robot_names.size() == 4)
        {
            return "quad_gp4";
        }
        if (robot_names.size() == 2)
        {
            return "dual_gp4";
        }
    }

    bool all_panda = !robot_names.empty();
    for (const auto &r : robot_names)
    {
        if (r.rfind("panda", 0) != 0 || !ends_with(r, "_arm"))
        {
            all_panda = false;
            break;
        }
    }
    if (all_panda)
    {
        if (robot_names.size() == 2)
        {
            return "panda_two";
        }
        if (robot_names.size() == 3)
        {
            return "panda_three";
        }
        if (robot_names.size() == 4)
        {
            return "panda_four";
        }
        if (robot_names.size() == 6)
        {
            return "panda_six";
        }
    }

    return std::nullopt;
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

void seed_initial_scene(const Json::Value &plan,
                        ActivityGraph *act_graph,
                        PlanInstance &instance,
                        std::unordered_map<std::string, ObjPtr> *last_obj_by_name)
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
        }
    }

    instance.updateScene();
}

}  // namespace

class VampEnvironment
{
public:
    VampEnvironment(const std::string &vamp_environment, double vmax = 1.0, int seed = 1)
    {
        auto env_opt = vamp_env::make_environment_config(vamp_environment);
        if (!env_opt)
        {
            throw std::runtime_error("Unsupported VAMP environment: " + vamp_environment);
        }
        config_ = *env_opt;

        instance_ = vamp_env::make_vamp_instance(config_);
        if (!instance_)
        {
            throw std::runtime_error("Failed to create VAMP instance");
        }

        const int num_robots = static_cast<int>(config_.robot_groups.size());
        instance_->setNumberOfRobots(num_robots);
        instance_->setRobotNames(config_.robot_groups);
        if (!config_.hand_groups.empty())
        {
            instance_->setHandNames(config_.hand_groups);
        }
        for (int rid = 0; rid < num_robots; ++rid)
        {
            instance_->setRobotDOF(rid, 7);
        }
        instance_->setVmax(vmax);
        if (seed >= 0)
        {
            instance_->setRandomSeed(static_cast<unsigned int>(seed));
        }

        vamp_env::add_environment_obstacles(config_, *instance_);
        vamp_env::add_environment_attachments(config_, *instance_);

        if (auto defaults = vamp_env::default_base_transforms(config_))
        {
            if (defaults->size() == static_cast<std::size_t>(num_robots))
            {
                for (int rid = 0; rid < num_robots; ++rid)
                {
                    instance_->setRobotBaseTransform(rid, (*defaults)[static_cast<std::size_t>(rid)]);
                }
            }
        }
    }

    const std::string &environment_name() const { return config_.environment_name; }

    py::dict info() const
    {
        py::dict out;
        out["environment_name"] = config_.environment_name;
        out["move_group"] = config_.move_group;
        out["robot_groups"] = config_.robot_groups;
        out["hand_groups"] = config_.hand_groups;
        out["num_robots"] = static_cast<int>(config_.robot_groups.size());
        return out;
    }

    py::dict build_roadmaps(int roadmap_samples, double roadmap_max_dist, int seed)
    {
        const double init_time_sec = rebuild_roadmap_cache(roadmap_samples, roadmap_max_dist, seed);

        py::dict out;
        out["num_robots"] = instance_->getNumberOfRobots();
        out["seed_requested"] = seed;
        out["seed"] = roadmap_cache_seed_;
        out["roadmap_samples"] = roadmap_samples;
        out["roadmap_max_dist"] = roadmap_max_dist;
        out["init_time_sec"] = init_time_sec;
        return out;
    }

    py::dict load_roadmaps(const std::string &path)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (num_robots <= 0)
        {
            throw std::runtime_error("Environment has no robots configured");
        }
        if (path.empty())
        {
            throw std::runtime_error("roadmap file path is empty");
        }

        const auto t0 = std::chrono::steady_clock::now();
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Failed to open roadmap file: " + path);
        }

        std::vector<std::shared_ptr<RoadMap>> loaded;
        try
        {
            boost::archive::binary_iarchive ia(ifs);
            ia >> loaded;
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error(std::string("Failed to deserialize roadmaps from ") + path + ": " + e.what());
        }

        if (static_cast<int>(loaded.size()) != num_robots)
        {
            throw std::runtime_error("Roadmap file robot count mismatch: expected " + std::to_string(num_robots) +
                                     " got " + std::to_string(loaded.size()));
        }

        for (auto &rm : loaded)
        {
            if (!rm)
            {
                throw std::runtime_error("Roadmap file contains null roadmap entries");
            }
            rm->setInstance(instance_);
        }

        roadmap_cache_ = std::move(loaded);
        roadmap_cache_valid_ = true;
        roadmap_cache_seed_ = 0;
        roadmap_cache_samples_ = 0;
        roadmap_cache_max_dist_ = 0.0;
        roadmap_cache_from_file_ = true;
        roadmap_cache_file_ = path;

        const auto t1 = std::chrono::steady_clock::now();
        py::dict out;
        out["num_robots"] = num_robots;
        out["roadmap_file"] = path;
        out["load_time_sec"] = std::chrono::duration<double>(t1 - t0).count();
        return out;
    }

    py::dict save_roadmaps(const std::string &path) const
    {
        if (!roadmap_cache_valid_ || roadmap_cache_.empty())
        {
            throw std::runtime_error("No cached roadmaps to save (call plan/build_roadmaps/load_roadmaps first)");
        }
        if (path.empty())
        {
            throw std::runtime_error("roadmap file path is empty");
        }

        const auto t0 = std::chrono::steady_clock::now();
        std::filesystem::path out_path(path);
        if (!out_path.parent_path().empty())
        {
            std::filesystem::create_directories(out_path.parent_path());
        }

        std::ofstream ofs(path, std::ios::binary);
        if (!ofs.is_open())
        {
            throw std::runtime_error("Failed to open roadmap file for writing: " + path);
        }

        try
        {
            boost::archive::binary_oarchive oa(ofs);
            oa << roadmap_cache_;
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error(std::string("Failed to serialize roadmaps to ") + path + ": " + e.what());
        }

        const auto t1 = std::chrono::steady_clock::now();
        py::dict out;
        out["num_robots"] = static_cast<int>(roadmap_cache_.size());
        out["roadmap_file"] = path;
        out["save_time_sec"] = std::chrono::duration<double>(t1 - t0).count();
        out["source"] = roadmap_cache_from_file_ ? "file" : "built";
        return out;
    }

    py::dict roadmap_cache_stats(bool check_collision, bool self_only) const
    {
        if (!roadmap_cache_valid_ || roadmap_cache_.empty())
        {
            throw std::runtime_error("No cached roadmaps (call plan/build_roadmaps/load_roadmaps first)");
        }

        py::dict out;
        out["source"] = roadmap_cache_from_file_ ? "file" : "built";
        if (roadmap_cache_from_file_)
        {
            out["roadmap_file"] = roadmap_cache_file_;
        }
        else
        {
            out["seed"] = roadmap_cache_seed_;
            out["roadmap_samples"] = roadmap_cache_samples_;
            out["roadmap_max_dist"] = roadmap_cache_max_dist_;
        }

        py::list robots;

        for (std::size_t rid = 0; rid < roadmap_cache_.size(); ++rid)
        {
            const auto &rm = roadmap_cache_[rid];
            if (!rm)
            {
                continue;
            }
            const auto g = rm->getRoadmap();
            if (!g)
            {
                continue;
            }

            const int n_vertices = g->size;
            std::uint64_t degree_sum = 0;
            int n_isolated = 0;
            std::uint64_t edge_half_sum = 0;
            for (int vid = 0; vid < n_vertices; ++vid)
            {
                const std::size_t deg = g->adjList[static_cast<std::size_t>(vid)].size();
                degree_sum += deg;
                edge_half_sum += deg;
                if (deg == 0)
                {
                    ++n_isolated;
                }
            }
            const std::uint64_t n_edges = edge_half_sum / 2;

            // Connected components via BFS.
            int components = 0;
            int largest_component = 0;
            std::vector<char> visited(static_cast<std::size_t>(std::max(0, n_vertices)), 0);
            std::vector<int> queue;
            queue.reserve(static_cast<std::size_t>(std::max(0, n_vertices)));
            for (int start = 0; start < n_vertices; ++start)
            {
                if (visited[static_cast<std::size_t>(start)])
                {
                    continue;
                }
                ++components;
                visited[static_cast<std::size_t>(start)] = 1;
                queue.clear();
                queue.push_back(start);
                int comp_size = 0;
                for (std::size_t qi = 0; qi < queue.size(); ++qi)
                {
                    const int u = queue[qi];
                    ++comp_size;
                    for (const auto &nbr : g->adjList[static_cast<std::size_t>(u)])
                    {
                        if (!nbr)
                        {
                            continue;
                        }
                        const int v = nbr->id;
                        if (v < 0 || v >= n_vertices)
                        {
                            continue;
                        }
                        if (visited[static_cast<std::size_t>(v)])
                        {
                            continue;
                        }
                        visited[static_cast<std::size_t>(v)] = 1;
                        queue.push_back(v);
                    }
                }
                largest_component = std::max(largest_component, comp_size);
            }

            int n_colliding = 0;
            if (check_collision)
            {
                for (const auto &vtx : g->vertices)
                {
                    if (!vtx)
                    {
                        continue;
                    }
                    if (instance_->checkCollision({vtx->pose}, self_only))
                    {
                        ++n_colliding;
                    }
                }
            }

            py::dict r;
            r["robot_id"] = static_cast<int>(rid);
            r["n_vertices"] = n_vertices;
            r["n_edges"] = static_cast<std::uint64_t>(n_edges);
            r["avg_degree"] = (n_vertices > 0) ? (static_cast<double>(degree_sum) / static_cast<double>(n_vertices)) : 0.0;
            r["n_isolated"] = n_isolated;
            r["components"] = components;
            r["largest_component"] = largest_component;
            if (check_collision)
            {
                r["n_colliding_vertices"] = n_colliding;
            }
            robots.append(std::move(r));
        }
        out["robots"] = std::move(robots);
        return out;
    }

    void set_random_seed(int seed)
    {
        if (seed >= 0)
        {
            instance_->setRandomSeed(static_cast<unsigned int>(seed));
        }
        else
        {
            instance_->setRandomSeed(entropy_seed_u32());
        }
        invalidate_roadmap_cache();
    }

    void enable_meshcat(const std::string &host, int port)
    {
        instance_->enableMeshcat(host, static_cast<std::uint16_t>(port));
    }

    void disable_meshcat() { instance_->disableMeshcat(); }

    void update_scene() { instance_->updateScene(); }
    void reset_scene(bool reset_sim)
    {
        instance_->resetScene(reset_sim);
        vamp_env::add_environment_obstacles(config_, *instance_);
        vamp_env::add_environment_attachments(config_, *instance_);
        instance_->updateScene();
        invalidate_roadmap_cache();
    }

    void set_robot_base_transform(int robot_id, const std::vector<std::vector<double>> &transform)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (robot_id < 0 || robot_id >= num_robots)
        {
            throw std::runtime_error("robot_id out of range");
        }
        if (transform.size() != 4U || transform[0].size() != 4U || transform[1].size() != 4U ||
            transform[2].size() != 4U || transform[3].size() != 4U)
        {
            throw std::runtime_error("transform must be a 4x4 matrix");
        }

        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                m(r, c) = transform[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
            }
        }
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.matrix() = m;
        instance_->setRobotBaseTransform(robot_id, tf);
        invalidate_roadmap_cache();
    }

    void set_robot_base_transforms(const std::vector<std::vector<std::vector<double>>> &transforms)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (static_cast<int>(transforms.size()) != num_robots)
        {
            throw std::runtime_error("transforms robot count mismatch");
        }
        for (int rid = 0; rid < num_robots; ++rid)
        {
            set_robot_base_transform(rid, transforms[static_cast<std::size_t>(rid)]);
        }
    }

    void seed_initial_scene_from_skillplan(const std::string &skillplan_path)
    {
        Json::Value plan_json;
        std::string err;
        if (!mr_planner::skillplan::read_json_from_file(skillplan_path, &plan_json, &err))
        {
            throw std::runtime_error("Failed to read skillplan '" + skillplan_path + "': " + err);
        }
        seed_initial_scene(plan_json, nullptr, *instance_, nullptr);
    }

    bool in_collision(const std::vector<std::vector<double>> &joint_positions, bool self_only)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (static_cast<int>(joint_positions.size()) != num_robots)
        {
            throw std::runtime_error("joint_positions robot count mismatch (expected " + std::to_string(num_robots) +
                                     ", got " + std::to_string(joint_positions.size()) + ")");
        }
        std::vector<RobotPose> poses;
        poses.reserve(joint_positions.size());
        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotPose pose = instance_->initRobotPose(rid);
            pose.joint_values = joint_positions[static_cast<std::size_t>(rid)];
            poses.push_back(std::move(pose));
        }
        return instance_->checkCollision(poses, self_only);
    }

    bool in_collision_robot(int robot_id, const std::vector<double> &joint_positions, bool self_only)
    {
        RobotPose pose = instance_->initRobotPose(robot_id);
        pose.joint_values = joint_positions;
        return instance_->checkCollision({pose}, self_only);
    }

    bool motion_in_collision(const std::vector<std::vector<double>> &start,
                             const std::vector<std::vector<double>> &goal,
                             double step_size,
                             bool self_only)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (static_cast<int>(start.size()) != num_robots || static_cast<int>(goal.size()) != num_robots)
        {
            throw std::runtime_error("start/goal robot counts must match environment robots");
        }
        std::vector<RobotPose> start_poses;
        std::vector<RobotPose> goal_poses;
        start_poses.reserve(start.size());
        goal_poses.reserve(goal.size());
        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotPose a = instance_->initRobotPose(rid);
            a.joint_values = start[static_cast<std::size_t>(rid)];
            start_poses.push_back(std::move(a));

            RobotPose b = instance_->initRobotPose(rid);
            b.joint_values = goal[static_cast<std::size_t>(rid)];
            goal_poses.push_back(std::move(b));
        }
        return instance_->checkMultiRobotMotion(start_poses, goal_poses, step_size, self_only);
    }

    bool trajectory_in_collision(const std::vector<std::vector<std::vector<double>>> &joint_trajectories, bool self_only)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (static_cast<int>(joint_trajectories.size()) != num_robots)
        {
            throw std::runtime_error("trajectory robot count mismatch");
        }
        if (joint_trajectories.empty())
        {
            return false;
        }
        const std::size_t steps = joint_trajectories.front().size();
        for (int rid = 0; rid < num_robots; ++rid)
        {
            if (joint_trajectories[static_cast<std::size_t>(rid)].size() != steps)
            {
                throw std::runtime_error("trajectory waypoint counts must match for all robots");
            }
        }

        MRTrajectory traj;
        traj.resize(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotTrajectory rt;
            rt.robot_id = rid;
            rt.trajectory.reserve(steps);
            rt.times.reserve(steps);
            const auto &robot_traj = joint_trajectories[static_cast<std::size_t>(rid)];
            for (std::size_t i = 0; i < steps; ++i)
            {
                RobotPose pose = instance_->initRobotPose(rid);
                pose.joint_values = robot_traj[i];
                rt.trajectory.push_back(std::move(pose));
                rt.times.push_back(static_cast<double>(i));
            }
            traj[static_cast<std::size_t>(rid)] = std::move(rt);
        }

        return instance_->checkMultiRobotTrajectory(traj, self_only);
    }

    std::vector<double> sample_pose(int robot_id)
    {
        RobotPose pose = instance_->initRobotPose(robot_id);
        if (!instance_->sample(pose))
        {
            throw std::runtime_error("Failed to sample pose for robot_id=" + std::to_string(robot_id));
        }
        return pose.joint_values;
    }

    std::vector<double> sample_collision_free_pose(int robot_id, int max_attempts, bool self_only)
    {
        if (max_attempts <= 0)
        {
            max_attempts = 1;
        }
        for (int attempt = 0; attempt < max_attempts; ++attempt)
        {
            auto q = sample_pose(robot_id);
            if (!in_collision_robot(robot_id, q, self_only))
            {
                return q;
            }
        }
        throw std::runtime_error("Failed to sample a collision-free pose for robot_id=" + std::to_string(robot_id));
    }

    std::vector<std::vector<double>> end_effector_transform(int robot_id, const std::vector<double> &joint_positions) const
    {
        RobotPose pose = instance_->initRobotPose(robot_id);
        pose.joint_values = joint_positions;
        const Eigen::Isometry3d tf = instance_->getEndEffectorTransformFromPose(pose);

        std::vector<std::vector<double>> out(4, std::vector<double>(4, 0.0));
        const Eigen::Matrix4d m = tf.matrix();
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                out[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = m(r, c);
            }
        }
        return out;
    }

    std::optional<std::vector<double>> inverse_kinematics(int robot_id,
                                                          const std::vector<std::vector<double>> &target_tf,
                                                          const py::object &seed_obj,
                                                          const py::object &fixed_joints_obj,
                                                          int max_restarts,
                                                          int max_iters,
                                                          double tol_pos,
                                                          double tol_ang,
                                                          double step_scale,
                                                          double damping,
                                                          bool self_only)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (robot_id < 0 || robot_id >= num_robots)
        {
            throw std::runtime_error("robot_id out of range");
        }

        const std::size_t dof = instance_->getRobotDOF(robot_id);
        if (dof == 0)
        {
            throw std::runtime_error("Robot DOF is zero");
        }

        if (target_tf.size() != 4U || target_tf[0].size() != 4U || target_tf[1].size() != 4U ||
            target_tf[2].size() != 4U || target_tf[3].size() != 4U)
        {
            throw std::runtime_error("target_tf must be a 4x4 matrix");
        }

        Eigen::Matrix4d target_m = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                target_m(r, c) = target_tf[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
            }
        }
        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        target.matrix() = target_m;

        PlanInstance::InverseKinematicsOptions options;
        options.max_restarts = max_restarts;
        options.max_iters = max_iters;
        options.tol_pos = tol_pos;
        options.tol_ang = tol_ang;
        options.step_scale = step_scale;
        options.damping = damping;
        options.self_only = self_only;

        if (!seed_obj.is_none())
        {
            options.seed = seed_obj.cast<std::vector<double>>();
            if (options.seed->size() != dof)
            {
                throw std::runtime_error("seed joint vector has wrong dimension");
            }
        }

        if (auto v = maybe_joint_matrix(fixed_joints_obj))
        {
            options.fixed_joints = std::move(*v);
            if (static_cast<int>(options.fixed_joints->size()) != num_robots)
            {
                throw std::runtime_error("fixed_joints robot count mismatch");
            }
        }

        return instance_->inverseKinematics(robot_id, target, options);
    }

    void add_object(const Object &obj)
    {
        instance_->addMoveableObject(obj);
        invalidate_roadmap_cache();
    }
    void move_object(const Object &obj)
    {
        instance_->moveObject(obj);
        invalidate_roadmap_cache();
    }
    void remove_object(const std::string &name)
    {
        instance_->removeObject(name);
        invalidate_roadmap_cache();
    }
    bool has_object(const std::string &name) const { return instance_->hasObject(name); }
    Object get_object(const std::string &name) const { return instance_->getObject(name); }

    void attach_object(const std::string &name,
                       int robot_id,
                       const std::vector<double> &joint_positions,
                       const std::string &link_name)
    {
        RobotPose pose = instance_->initRobotPose(robot_id);
        pose.joint_values = joint_positions;
        instance_->attachObjectToRobot(name, robot_id, link_name, pose);
        invalidate_roadmap_cache();
    }

    void detach_object(const std::string &name, int robot_id, const std::vector<double> &joint_positions)
    {
        RobotPose pose = instance_->initRobotPose(robot_id);
        pose.joint_values = joint_positions;
        instance_->detachObjectFromRobot(name, pose);
        invalidate_roadmap_cache();
    }

    bool set_allowed_collision(const std::string &obj_name, const std::string &link_name, bool allow)
    {
        const bool ok = instance_->setCollision(obj_name, link_name, allow);
        invalidate_roadmap_cache();
        return ok;
    }

    void print_known_objects() const { instance_->printKnownObjects(); }

    py::dict plan(const std::string &planner,
                  double planning_time,
                  double shortcut_time,
                  int seed,
                  int sample_attempts,
                  double max_goal_dist,
                  double dt,
                  double vmax,
                  const py::object &start_obj,
                  const py::object &goal_obj,
                  const std::string &output_dir,
                  bool write_tpg,
                  int roadmap_samples,
                  double roadmap_max_dist,
                  bool write_files,
                  bool return_trajectories,
                  std::optional<int> roadmap_seed)
    {
        instance_->setVmax(vmax);
        if (seed >= 0)
        {
            instance_->setRandomSeed(static_cast<unsigned int>(seed));
        }

        std::vector<std::vector<double>> start;
        std::vector<std::vector<double>> goal;
        if (auto v = maybe_joint_matrix(start_obj))
        {
            start = std::move(*v);
        }
        if (auto v = maybe_joint_matrix(goal_obj))
        {
            goal = std::move(*v);
        }

        const int num_robots = instance_->getNumberOfRobots();
        if (!start.empty() || !goal.empty())
        {
            if (start.empty() || goal.empty())
            {
                throw std::runtime_error("Both start and goal must be provided (or omit both)");
            }
            if (static_cast<int>(start.size()) != num_robots || static_cast<int>(goal.size()) != num_robots)
            {
                throw std::runtime_error("start/goal robot counts must match environment robots");
            }
        }
        else
        {
            std::string sample_err;
            if (!sample_easy_problem(instance_,
                                     num_robots,
                                     7,
                                     sample_attempts,
                                     max_goal_dist,
                                     seed,
                                     &start,
                                     &goal,
                                     &sample_err))
            {
                throw std::runtime_error(sample_err);
            }
        }

        for (int i = 0; i < num_robots; ++i)
        {
            if (start[static_cast<std::size_t>(i)].size() != goal[static_cast<std::size_t>(i)].size())
            {
                throw std::runtime_error("Robot " + std::to_string(i) + " DOF mismatch between start and goal");
            }
            instance_->setRobotDOF(i, start[static_cast<std::size_t>(i)].size());
            instance_->setStartPose(i, start[static_cast<std::size_t>(i)]);
            instance_->setGoalPose(i, goal[static_cast<std::size_t>(i)]);
        }

        const bool need_output_dir = write_files || write_tpg;
        const std::string resolved_output_dir =
            need_output_dir ? (output_dir.empty() ? make_temp_dir("mr_planner_py_plan") : output_dir) : std::string();
        if (need_output_dir)
        {
            std::filesystem::create_directories(resolved_output_dir);
        }

        PlannerOptions options;
        options.max_planning_time = planning_time;
        options.rrt_max_planning_time = planning_time;
        options.max_planning_iterations = 10000;
        options.rrt_seed = seed;

        MRTrajectory solution;
        double planner_time = 0.0;
        double init_time = 0.0;
        bool roadmap_cache_hit = false;
        if (planner == "composite_rrt")
        {
            options.max_dist = roadmap_max_dist;
            CompositeRRTCPlanner planner_impl(instance_);
            if (seed >= 0)
            {
                planner_impl.setSeed(seed);
            }
            if (!planner_impl.plan(options))
            {
                throw std::runtime_error("Planning failed");
            }
            planner_time = planner_impl.getPlanTime();
            if (!planner_impl.getPlan(solution))
            {
                throw std::runtime_error("Planner returned no solution");
            }
        }
        else if (planner == "cbs_prm")
        {
            const int effective_roadmap_seed = roadmap_seed.value_or(seed);

            // PRM edge validation uses `options.max_dist` directly, while RoadMap uses
            // `2 * max_dist_` in its `validateMotion` check. ROS demos rely on the
            // default `PlannerOptions::max_dist` (4.0) paired with RoadMap's default
            // `max_dist_` (2.0). Mirror that behavior by scaling here so passing
            // `roadmap_max_dist=2.0` produces the same planning behavior as ROS.
            options.max_dist = 2.0 * roadmap_max_dist;
            options.num_samples = roadmap_samples;

            auto cache = get_cached_or_build_roadmaps(roadmap_samples, roadmap_max_dist, effective_roadmap_seed);
            roadmap_cache_hit = cache.cache_hit;
            init_time = cache.init_time_sec;

            if (seed >= 0)
            {
                instance_->setRandomSeed(static_cast<unsigned int>(seed));
            }

            CBSPlanner planner_impl(instance_, std::move(cache.roadmaps));
            if (!planner_impl.plan(options))
            {
                throw std::runtime_error("CBS PRM planning failed");
            }
            planner_time = planner_impl.getPlanTime();
            if (!planner_impl.getPlan(solution))
            {
                throw std::runtime_error("CBS PRM returned no solution");
            }
        }
        else
        {
            throw std::runtime_error("Unknown planner: " + planner);
        }

        int num_nodes_expanded = 0;
        int num_col_checks = 0;
        for (const auto &rt : solution)
        {
            num_nodes_expanded = std::max(num_nodes_expanded, rt.num_nodes_expanded);
            num_col_checks = std::max(num_col_checks, rt.num_col_checks);
        }

        MRTrajectory discrete;
        rediscretizeSolution(instance_, solution, discrete, dt);
        std::string raw_csv;
        if (write_files)
        {
            raw_csv = (std::filesystem::path(resolved_output_dir) / "solution_raw.csv").string();
            saveSolution(instance_, discrete, raw_csv);
        }

        MRTrajectory final_solution = discrete;
        if (shortcut_time > 0.0)
        {
            ShortcutOptions sc;
            sc.t_limit = shortcut_time;
            sc.dt = dt;
            sc.seed = seed;
            sc.thompson_selector = true;
            sc.progress_file = write_files ?
                                   (std::filesystem::path(resolved_output_dir) / "shortcut_progress.csv").string() :
                                   std::string();

            Shortcutter shortcutter(instance_, sc);
            MRTrajectory shortcut;
            if (shortcutter.shortcutSolution(final_solution, shortcut))
            {
                final_solution = std::move(shortcut);
            }
        }

        std::string out_csv;
        if (write_files)
        {
            out_csv = (std::filesystem::path(resolved_output_dir) / "solution.csv").string();
            saveSolution(instance_, final_solution, out_csv);
        }

        std::string skillplan_path;
        if (write_files)
        {
            skillplan_path = (std::filesystem::path(resolved_output_dir) / "skillplan.json").string();
            std::vector<mr_planner::skillplan::RobotSpec> robots;
            robots.reserve(static_cast<std::size_t>(num_robots));
            for (int i = 0; i < num_robots; ++i)
            {
                mr_planner::skillplan::RobotSpec spec;
                spec.id = i;
                spec.name = instance_->getRobotName(i);
                spec.dof = static_cast<int>(instance_->getRobotDOF(i));
                robots.push_back(std::move(spec));
            }

            mr_planner::skillplan::ExportOptions opts;
            opts.plan_name = "mr_planner_python";
            opts.environment_name = config_.environment_name;
            opts.backend_type = instance_->instanceType();
            opts.l1_vmax = vmax;

            const auto json = mr_planner::skillplan::make_simple_plan(robots, final_solution, opts);
            std::string err;
            if (!mr_planner::skillplan::write_json_to_file(json, skillplan_path, &err))
            {
                throw std::runtime_error("Failed to write skillplan.json: " + err);
            }
        }

        std::string tpg_pb_path;
        if (write_tpg)
        {
            tpg::TPGConfig cfg;
            cfg.dt = dt;
            cfg.output_dir = resolved_output_dir;
            cfg.shortcut = false;
            cfg.random_shortcut = false;
            cfg.parallel = true;

            tpg::TPG tpg;
            if (tpg.init(instance_, final_solution, cfg))
            {
                tpg_pb_path = (std::filesystem::path(resolved_output_dir) / "tpg.pb").string();
                std::string pb_err;
                if (!mr_planner::graph_proto::write_tpg(tpg, tpg_pb_path, &pb_err))
                {
                    throw std::runtime_error("Failed to write tpg.pb: " + pb_err);
                }
            }
        }

        py::dict out;
        out["output_dir"] = resolved_output_dir;
        out["start"] = start;
        out["goal"] = goal;
        out["planner_time_sec"] = planner_time;
        out["init_time_sec"] = init_time;
        out["roadmap_cache_hit"] = roadmap_cache_hit;
        out["roadmap_source"] = roadmap_cache_from_file_ ? "file" : "built";
        if (roadmap_cache_from_file_)
        {
            out["roadmap_file"] = roadmap_cache_file_;
        }
        else
        {
            out["roadmap_seed"] = roadmap_cache_seed_;
            out["roadmap_samples"] = roadmap_cache_samples_;
            out["roadmap_max_dist"] = roadmap_cache_max_dist_;
        }
        out["n_node_expanded"] = num_nodes_expanded;
        out["n_col_checks"] = num_col_checks;
        out["solution_raw_csv"] = raw_csv;
        out["solution_csv"] = out_csv;
        out["skillplan_json"] = skillplan_path;
        out["tpg_pb"] = tpg_pb_path;

        if (return_trajectories)
        {
            auto pack = [&](const MRTrajectory &traj,
                            std::vector<double> *times_out,
                            std::vector<std::vector<std::vector<double>>> *joints_out) {
                if (!times_out || !joints_out)
                {
                    return;
                }
                times_out->clear();
                joints_out->clear();
                joints_out->resize(static_cast<std::size_t>(num_robots));
                if (traj.empty())
                {
                    return;
                }
                const auto &rt0 = traj.front();
                times_out->assign(rt0.times.begin(), rt0.times.end());
                for (int rid = 0; rid < num_robots; ++rid)
                {
                    const auto &rt = traj[static_cast<std::size_t>(rid)];
                    auto &out_r = (*joints_out)[static_cast<std::size_t>(rid)];
                    out_r.reserve(rt.trajectory.size());
                    for (const auto &pose : rt.trajectory)
                    {
                        out_r.push_back(pose.joint_values);
                    }
                }
            };

            std::vector<double> times_raw_out;
            std::vector<std::vector<std::vector<double>>> traj_raw_out;
            pack(discrete, &times_raw_out, &traj_raw_out);
            std::vector<double> times_out;
            std::vector<std::vector<std::vector<double>>> traj_out;
            pack(final_solution, &times_out, &traj_out);

            out["times_raw"] = times_raw_out;
            out["traj_raw"] = traj_raw_out;
            out["times"] = times_out;
            out["traj"] = traj_out;
        }
        return out;
    }

    py::dict shortcut_trajectory(const std::vector<std::vector<std::vector<double>>> &joint_trajectories,
                                 double shortcut_time,
                                 double dt,
                                 int seed,
                                 const std::string &method)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (static_cast<int>(joint_trajectories.size()) != num_robots)
        {
            throw std::runtime_error("trajectory robot count mismatch");
        }

        MRTrajectory in;
        in.resize(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotTrajectory rt;
            rt.robot_id = rid;
            const auto &robot_traj = joint_trajectories[static_cast<std::size_t>(rid)];
            rt.trajectory.reserve(robot_traj.size());
            rt.times.reserve(robot_traj.size());
            for (std::size_t i = 0; i < robot_traj.size(); ++i)
            {
                RobotPose pose = instance_->initRobotPose(rid);
                pose.joint_values = robot_traj[i];
                rt.trajectory.push_back(std::move(pose));
                rt.times.push_back(static_cast<double>(i) * dt);
            }
            in[static_cast<std::size_t>(rid)] = std::move(rt);
        }

        ShortcutOptions sc;
        sc.t_limit = shortcut_time;
        sc.dt = dt;
        sc.seed = seed;
        sc.progress_file.clear();
        sc.thompson_selector = false;
        sc.auto_selector = false;
        sc.round_robin = false;
        sc.comp_shortcut = false;
        sc.prioritized_shortcut = false;
        sc.path_shortcut = false;

        if (method == "thompson")
        {
            sc.thompson_selector = true;
        }
        else if (method == "auto")
        {
            sc.auto_selector = true;
        }
        else if (method == "round_robin")
        {
            sc.round_robin = true;
        }
        else if (method == "comp")
        {
            sc.comp_shortcut = true;
        }
        else if (method == "prioritized")
        {
            sc.prioritized_shortcut = true;
        }
        else if (method == "path")
        {
            sc.path_shortcut = true;
        }
        else
        {
            throw std::runtime_error("Unknown shortcut method: " + method);
        }

        Shortcutter shortcutter(instance_, sc);
        MRTrajectory out;
        const bool ok = shortcutter.shortcutSolution(in, out);

        std::vector<std::vector<std::vector<double>>> out_traj;
        out_traj.resize(static_cast<std::size_t>(num_robots));
        if (ok)
        {
            for (int rid = 0; rid < num_robots; ++rid)
            {
                const auto &rt = out[static_cast<std::size_t>(rid)];
                out_traj[static_cast<std::size_t>(rid)].reserve(rt.trajectory.size());
                for (const auto &pose : rt.trajectory)
                {
                    out_traj[static_cast<std::size_t>(rid)].push_back(pose.joint_values);
                }
            }
        }

        py::dict res;
        res["success"] = ok;
        res["trajectory"] = out_traj;
        return res;
    }

    bool play_trajectory(const std::vector<std::vector<std::vector<double>>> &joint_trajectories,
                         double dt,
                         double rate)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (static_cast<int>(joint_trajectories.size()) != num_robots)
        {
            throw std::runtime_error("trajectory robot count mismatch");
        }

        MRTrajectory traj;
        traj.resize(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotTrajectory rt;
            rt.robot_id = rid;
            const auto &robot_traj = joint_trajectories[static_cast<std::size_t>(rid)];
            rt.trajectory.reserve(robot_traj.size());
            rt.times.reserve(robot_traj.size());
            for (std::size_t i = 0; i < robot_traj.size(); ++i)
            {
                RobotPose pose = instance_->initRobotPose(rid);
                pose.joint_values = robot_traj[i];
                rt.trajectory.push_back(std::move(pose));
                rt.times.push_back(static_cast<double>(i) * dt);
            }
            traj[static_cast<std::size_t>(rid)] = std::move(rt);
        }

        mr_planner::visualization::MeshcatPlaybackOptions viz;
        viz.real_time_rate = rate;
        return mr_planner::visualization::play_synchronized_trajectory(*instance_, traj, dt, viz);
    }

    bool play_solution_csv(const std::string &csv_path, double rate)
    {
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open CSV: " + csv_path);
        }

        const int num_robots = instance_->getNumberOfRobots();
        int total_dof = 0;
        for (int rid = 0; rid < num_robots; ++rid)
        {
            total_dof += static_cast<int>(instance_->getRobotDOF(rid));
        }

        std::string line;
        if (!std::getline(file, line))
        {
            throw std::runtime_error("Empty CSV: " + csv_path);
        }

        std::vector<RobotPose> poses(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            poses[static_cast<std::size_t>(rid)] = instance_->initRobotPose(rid);
        }

        double last_time = 0.0;
        bool have_last_time = false;

        auto sleep_sec = [&](double sec) {
            if (sec <= 0.0)
            {
                return;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(sec));
        };

        while (std::getline(file, line))
        {
            if (line.empty())
            {
                continue;
            }
            std::vector<std::string> tokens;
            std::stringstream ss(line);
            std::string tok;
            while (std::getline(ss, tok, ','))
            {
                tokens.push_back(tok);
            }
            if (!tokens.empty() && tokens.back().empty())
            {
                tokens.pop_back();
            }
            if (static_cast<int>(tokens.size()) != (1 + total_dof))
            {
                throw std::runtime_error("Invalid CSV row width in " + csv_path);
            }

            const double t = std::stod(tokens[0]);
            int idx = 1;
            for (int rid = 0; rid < num_robots; ++rid)
            {
                auto &pose = poses[static_cast<std::size_t>(rid)];
                const int dof = static_cast<int>(instance_->getRobotDOF(rid));
                pose.joint_values.resize(static_cast<std::size_t>(dof));
                for (int d = 0; d < dof; ++d)
                {
                    pose.joint_values[static_cast<std::size_t>(d)] = std::stod(tokens[static_cast<std::size_t>(idx++)]);
                }
                instance_->moveRobot(rid, pose);
            }
            instance_->updateScene();

            if (have_last_time)
            {
                const double dt = t - last_time;
                sleep_sec((rate > 0.0) ? (dt / rate) : 0.0);
            }
            last_time = t;
            have_last_time = true;
        }
        return true;
    }

    bool play_execution_graph(const std::string &pb_path, double rate)
    {
        mr_planner::proto::GraphFile file;
        const auto data = read_file_binary(pb_path);
        if (!file.ParseFromString(data))
        {
            throw std::runtime_error("Failed to parse GraphFile protobuf: " + pb_path);
        }

        const mr_planner::proto::TPGGraph *tpg = nullptr;
        if (file.has_tpg())
        {
            tpg = &file.tpg();
        }
        else if (file.has_adg())
        {
            tpg = &file.adg().base();
        }
        else
        {
            throw std::runtime_error("GraphFile has no tpg/adg payload: " + pb_path);
        }

        const int num_robots = instance_->getNumberOfRobots();
        if (tpg->robots_size() != num_robots)
        {
            throw std::runtime_error("GraphFile robot count mismatch (expected " + std::to_string(num_robots) +
                                     ", got " + std::to_string(tpg->robots_size()) + ")");
        }

        std::vector<std::size_t> idx(static_cast<std::size_t>(num_robots), 0);
        std::vector<RobotPose> last_pose(static_cast<std::size_t>(num_robots));
        int max_step = 0;
        for (int rid = 0; rid < num_robots; ++rid)
        {
            const auto &timeline = tpg->robots(rid);
            if (timeline.nodes_size() > 0)
            {
                max_step = std::max(max_step, timeline.nodes(timeline.nodes_size() - 1).time_step());
            }
            last_pose[static_cast<std::size_t>(rid)] = instance_->getStartPose(rid);
            instance_->moveRobot(rid, last_pose[static_cast<std::size_t>(rid)]);
        }
        instance_->updateScene();

        const double dt = (tpg->dt() > 0.0) ? tpg->dt() : 0.1;
        auto sleep_sec = [&](double sec) {
            if (sec <= 0.0)
            {
                return;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(sec));
        };

        for (int step = 0; step <= max_step; ++step)
        {
            for (int rid = 0; rid < num_robots; ++rid)
            {
                const auto &timeline = tpg->robots(rid);
                auto &cur_idx = idx[static_cast<std::size_t>(rid)];
                if (cur_idx < static_cast<std::size_t>(timeline.nodes_size()))
                {
                    const auto &node = timeline.nodes(static_cast<int>(cur_idx));
                    if (node.time_step() == step)
                    {
                        RobotPose pose = instance_->initRobotPose(rid);
                        pose.joint_values.assign(node.joint_values().begin(), node.joint_values().end());
                        last_pose[static_cast<std::size_t>(rid)] = std::move(pose);
                        ++cur_idx;
                    }
                }
                instance_->moveRobot(rid, last_pose[static_cast<std::size_t>(rid)]);
            }
            instance_->updateScene();
            sleep_sec((rate > 0.0) ? (dt / rate) : 0.0);
        }

        return true;
    }

private:
    void invalidate_roadmap_cache()
    {
        roadmap_cache_.clear();
        roadmap_cache_valid_ = false;
        roadmap_cache_seed_ = 0;
        roadmap_cache_samples_ = 0;
        roadmap_cache_max_dist_ = 0.0;
        roadmap_cache_from_file_ = false;
        roadmap_cache_file_.clear();
    }

    static std::uint32_t entropy_seed_u32()
    {
        std::random_device rd;
        const std::uint64_t seed64 =
            (static_cast<std::uint64_t>(rd()) << 32) ^ static_cast<std::uint64_t>(rd());
        return static_cast<std::uint32_t>((seed64 >> 32) ^ (seed64 & 0xffffffffu));
    }

    struct RoadmapCacheResult
    {
        std::vector<std::shared_ptr<RoadMap>> roadmaps;
        bool cache_hit{false};
        double init_time_sec{0.0};
    };

    double rebuild_roadmap_cache(int roadmap_samples, double roadmap_max_dist, int seed)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (num_robots <= 0)
        {
            throw std::runtime_error("Environment has no robots configured");
        }

        const std::uint32_t seed_used = (seed >= 0) ? static_cast<std::uint32_t>(seed) : entropy_seed_u32();
        instance_->setRandomSeed(static_cast<unsigned int>(seed_used));

        const auto t0 = std::chrono::steady_clock::now();
        roadmap_cache_.clear();
        roadmap_cache_.reserve(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            auto rm = std::make_shared<RoadMap>(instance_, rid);
            rm->setNumSamples(roadmap_samples);
            rm->setMaxDist(roadmap_max_dist);
            rm->buildRoadmap();
            roadmap_cache_.push_back(std::move(rm));
        }
        const auto t1 = std::chrono::steady_clock::now();

        roadmap_cache_valid_ = true;
        roadmap_cache_seed_ = seed_used;
        roadmap_cache_samples_ = roadmap_samples;
        roadmap_cache_max_dist_ = roadmap_max_dist;
        roadmap_cache_from_file_ = false;
        roadmap_cache_file_.clear();

        return std::chrono::duration<double>(t1 - t0).count();
    }

    RoadmapCacheResult get_cached_or_build_roadmaps(int roadmap_samples, double roadmap_max_dist, int seed)
    {
        const int num_robots = instance_->getNumberOfRobots();
        if (num_robots <= 0)
        {
            throw std::runtime_error("Environment has no robots configured");
        }

        RoadmapCacheResult result;
        result.cache_hit =
            roadmap_cache_valid_ && static_cast<int>(roadmap_cache_.size()) == num_robots &&
            (roadmap_cache_from_file_ ||
             ((seed < 0 || roadmap_cache_seed_ == static_cast<std::uint32_t>(seed)) &&
              roadmap_cache_samples_ == roadmap_samples &&
              std::abs(roadmap_cache_max_dist_ - roadmap_max_dist) < 1e-12));

        if (result.cache_hit)
        {
            result.roadmaps = roadmap_cache_;
            return result;
        }

        result.init_time_sec = rebuild_roadmap_cache(roadmap_samples, roadmap_max_dist, seed);
        result.roadmaps = roadmap_cache_;
        return result;
    }

    vamp_env::EnvironmentConfig config_;
    std::shared_ptr<PlanInstance> instance_;

    std::vector<std::shared_ptr<RoadMap>> roadmap_cache_;
    bool roadmap_cache_valid_{false};
    std::uint32_t roadmap_cache_seed_{0};
    int roadmap_cache_samples_{0};
    double roadmap_cache_max_dist_{0.0};
    bool roadmap_cache_from_file_{false};
    std::string roadmap_cache_file_;
};

py::dict plan(const std::string &vamp_environment,
              const std::string &planner,
              double planning_time,
              double shortcut_time,
              int seed,
              int num_robots,
              int dof,
              int sample_attempts,
              double max_goal_dist,
              double dt,
              double vmax,
              const py::object &start_obj,
              const py::object &goal_obj,
              const std::string &output_dir,
              bool write_tpg,
              int roadmap_samples,
              double roadmap_max_dist)
{
    auto env_opt = vamp_env::make_environment_config(vamp_environment);
    if (!env_opt)
    {
        throw std::runtime_error("Unsupported VAMP environment: " + vamp_environment);
    }

    auto instance = vamp_env::make_vamp_instance(*env_opt);
    if (!instance)
    {
        throw std::runtime_error("Failed to create VAMP instance");
    }

    std::vector<std::vector<double>> start;
    std::vector<std::vector<double>> goal;
    if (auto v = maybe_joint_matrix(start_obj))
    {
        start = std::move(*v);
    }
    if (auto v = maybe_joint_matrix(goal_obj))
    {
        goal = std::move(*v);
    }

    int desired_num_robots = num_robots;
    if (!start.empty() || !goal.empty())
    {
        if (start.empty() || goal.empty())
        {
            throw std::runtime_error("Both start and goal must be provided (or omit both)");
        }
        if (start.size() != goal.size())
        {
            throw std::runtime_error("start and goal robot counts differ");
        }
        desired_num_robots = static_cast<int>(start.size());
    }

    if (desired_num_robots <= 0)
    {
        throw std::runtime_error("No robots specified");
    }

    std::vector<std::string> robot_names;
    if (static_cast<int>(env_opt->robot_groups.size()) == desired_num_robots)
    {
        robot_names = env_opt->robot_groups;
    }
    else if (static_cast<int>(env_opt->robot_groups.size()) > desired_num_robots)
    {
        robot_names.assign(env_opt->robot_groups.begin(), env_opt->robot_groups.begin() + desired_num_robots);
    }
    else
    {
        robot_names = env_opt->robot_groups;
        for (int i = static_cast<int>(robot_names.size()); i < desired_num_robots; ++i)
        {
            robot_names.push_back("robot_" + std::to_string(i));
        }
    }

    instance->setNumberOfRobots(desired_num_robots);
    instance->setRobotNames(robot_names);
    if (!env_opt->hand_groups.empty())
    {
        instance->setHandNames(env_opt->hand_groups);
    }
    instance->setVmax(vmax);
    instance->setRandomSeed(static_cast<unsigned int>(seed));

    vamp_env::add_environment_obstacles(*env_opt, *instance);
    vamp_env::add_environment_attachments(*env_opt, *instance);

    if (auto defaults = vamp_env::default_base_transforms(*env_opt))
    {
        if (defaults->size() == static_cast<std::size_t>(desired_num_robots))
        {
            for (int i = 0; i < desired_num_robots; ++i)
            {
                instance->setRobotBaseTransform(i, (*defaults)[static_cast<std::size_t>(i)]);
            }
        }
    }

    if (start.empty() && goal.empty())
    {
        std::string sample_err;
        if (!sample_easy_problem(instance,
                                 desired_num_robots,
                                 dof,
                                 sample_attempts,
                                 max_goal_dist,
                                 seed,
                                 &start,
                                 &goal,
                                 &sample_err))
        {
            throw std::runtime_error(sample_err);
        }
    }

    const int actual_num_robots = static_cast<int>(start.size());
    for (int i = 0; i < actual_num_robots; ++i)
    {
        if (start[static_cast<std::size_t>(i)].size() != goal[static_cast<std::size_t>(i)].size())
        {
            throw std::runtime_error("Robot " + std::to_string(i) + " DOF mismatch between start and goal");
        }
        instance->setRobotDOF(i, start[static_cast<std::size_t>(i)].size());
        instance->setStartPose(i, start[static_cast<std::size_t>(i)]);
        instance->setGoalPose(i, goal[static_cast<std::size_t>(i)]);
    }

    const std::string resolved_output_dir = output_dir.empty() ? make_temp_dir("mr_planner_py_plan") : output_dir;
    std::filesystem::create_directories(resolved_output_dir);

    PlannerOptions options;
    options.max_planning_time = planning_time;
    options.rrt_max_planning_time = planning_time;
    options.max_planning_iterations = 10000;
    options.max_dist = roadmap_max_dist;
    options.num_samples = roadmap_samples;
    options.rrt_seed = seed;

    MRTrajectory solution;
    double planner_time = 0.0;
    double init_time = 0.0;
    if (planner == "composite_rrt")
    {
        CompositeRRTCPlanner planner_impl(instance);
        planner_impl.setSeed(seed);
        if (!planner_impl.plan(options))
        {
            throw std::runtime_error("Planning failed");
        }
        planner_time = planner_impl.getPlanTime();
        if (!planner_impl.getPlan(solution))
        {
            throw std::runtime_error("Planner returned no solution");
        }
    }
    else if (planner == "cbs_prm")
    {
        const auto init_t0 = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<RoadMap>> roadmaps;
        roadmaps.reserve(static_cast<std::size_t>(actual_num_robots));
        for (int rid = 0; rid < actual_num_robots; ++rid)
        {
            auto rm = std::make_shared<RoadMap>(instance, rid);
            rm->setNumSamples(roadmap_samples);
            rm->setMaxDist(roadmap_max_dist);
            rm->buildRoadmap();
            roadmaps.push_back(std::move(rm));
        }
        const auto init_t1 = std::chrono::steady_clock::now();
        init_time = std::chrono::duration<double>(init_t1 - init_t0).count();

        CBSPlanner planner_impl(instance, roadmaps);
        if (!planner_impl.plan(options))
        {
            throw std::runtime_error("CBS PRM planning failed");
        }
        planner_time = planner_impl.getPlanTime();
        if (!planner_impl.getPlan(solution))
        {
            throw std::runtime_error("CBS PRM returned no solution");
        }
    }
    else
    {
        throw std::runtime_error("Unknown planner: " + planner);
    }

    int num_nodes_expanded = 0;
    int num_col_checks = 0;
    for (const auto &rt : solution)
    {
        num_nodes_expanded = std::max(num_nodes_expanded, rt.num_nodes_expanded);
        num_col_checks = std::max(num_col_checks, rt.num_col_checks);
    }

    MRTrajectory discrete;
    rediscretizeSolution(instance, solution, discrete, dt);
    const std::string raw_csv = (std::filesystem::path(resolved_output_dir) / "solution_raw.csv").string();
    saveSolution(instance, discrete, raw_csv);

    MRTrajectory final_solution = discrete;
    if (shortcut_time > 0.0)
    {
        ShortcutOptions sc;
        sc.t_limit = shortcut_time;
        sc.dt = dt;
        sc.seed = seed;
        sc.thompson_selector = true;
        sc.progress_file = (std::filesystem::path(resolved_output_dir) / "shortcut_progress.csv").string();

        Shortcutter shortcutter(instance, sc);
        MRTrajectory shortcut;
        if (shortcutter.shortcutSolution(final_solution, shortcut))
        {
            final_solution = std::move(shortcut);
        }
    }

    const std::string out_csv = (std::filesystem::path(resolved_output_dir) / "solution.csv").string();
    saveSolution(instance, final_solution, out_csv);

    const std::string skillplan_path = (std::filesystem::path(resolved_output_dir) / "skillplan.json").string();
    {
        std::vector<mr_planner::skillplan::RobotSpec> robots;
        robots.reserve(static_cast<std::size_t>(actual_num_robots));
        for (int i = 0; i < actual_num_robots; ++i)
        {
            mr_planner::skillplan::RobotSpec spec;
            spec.id = i;
            if (i < static_cast<int>(env_opt->robot_groups.size()))
            {
                spec.name = env_opt->robot_groups[static_cast<std::size_t>(i)];
            }
            else
            {
                spec.name = "robot_" + std::to_string(i);
            }
            spec.dof = static_cast<int>(instance->getRobotDOF(i));
            robots.push_back(std::move(spec));
        }

        mr_planner::skillplan::ExportOptions opts;
        opts.plan_name = "mr_planner_python";
        opts.environment_name = env_opt->environment_name;
        opts.backend_type = instance->instanceType();
        opts.l1_vmax = vmax;

        const auto json = mr_planner::skillplan::make_simple_plan(robots, final_solution, opts);
        std::string err;
        if (!mr_planner::skillplan::write_json_to_file(json, skillplan_path, &err))
        {
            throw std::runtime_error("Failed to write skillplan.json: " + err);
        }
    }

    std::string tpg_pb_path;
    if (write_tpg)
    {
        tpg::TPGConfig cfg;
        cfg.dt = dt;
        cfg.output_dir = resolved_output_dir;
        cfg.shortcut = false;
        cfg.random_shortcut = false;
        cfg.parallel = true;

        tpg::TPG tpg;
        if (tpg.init(instance, final_solution, cfg))
        {
            tpg_pb_path = (std::filesystem::path(resolved_output_dir) / "tpg.pb").string();
            std::string pb_err;
            if (!mr_planner::graph_proto::write_tpg(tpg, tpg_pb_path, &pb_err))
            {
                throw std::runtime_error("Failed to write tpg.pb: " + pb_err);
            }
        }
    }

    py::dict out;
    out["output_dir"] = resolved_output_dir;
    out["start"] = start;
    out["goal"] = goal;
    out["planner_time_sec"] = planner_time;
    out["init_time_sec"] = init_time;
    out["n_node_expanded"] = num_nodes_expanded;
    out["n_col_checks"] = num_col_checks;
    out["solution_raw_csv"] = raw_csv;
    out["solution_csv"] = out_csv;
    out["skillplan_json"] = skillplan_path;
    out["tpg_pb"] = tpg_pb_path;
    return out;
}

py::dict skillplan_to_graph(const std::string &skillplan_path,
                            const std::string &graph_type,
                            const std::string &vamp_environment,
                            const std::string &output_dir,
                            const std::string &pb_file,
                            const std::string &dot_file,
                            double dt,
                            double vmax,
                            double shortcut_time)
{
    if (shortcut_time < 0.0)
    {
        throw std::runtime_error("shortcut_time must be >= 0");
    }
    const double resolved_dt = (dt > 0.0) ? dt : (0.05 / std::max(vmax, 1e-6));

    Json::Value plan_json;
    std::string err;
    if (!mr_planner::skillplan::read_json_from_file(skillplan_path, &plan_json, &err))
    {
        throw std::runtime_error("Failed to read skillplan: " + err);
    }

    MRTrajectory trajectories;
    if (!mr_planner::skillplan::extract_robot_trajectories(plan_json, &trajectories, &err))
    {
        throw std::runtime_error("Failed to extract trajectories from skillplan: " + err);
    }

    const auto robot_names = robot_names_from_skillplan(plan_json);
    const auto robot_dofs = robot_dofs_from_skillplan(plan_json);
    const int num_robots = static_cast<int>(robot_names.size());
    if (num_robots <= 0)
    {
        throw std::runtime_error("skillplan robots[] is empty");
    }

    const std::string skillplan_environment = vamp_environment_from_skillplan(plan_json);
    const bool has_override_environment = !vamp_environment.empty();
    const bool has_skillplan_environment = !skillplan_environment.empty();
    std::string resolved_environment = has_override_environment ? vamp_environment : skillplan_environment;

    std::optional<vamp_env::EnvironmentConfig> env_opt;
    if (!resolved_environment.empty())
    {
        env_opt = vamp_env::make_environment_config(resolved_environment);
    }
    if (!env_opt && !has_override_environment)
    {
        if (auto inferred = infer_vamp_environment_from_robot_names(robot_names))
        {
            resolved_environment = *inferred;
            env_opt = vamp_env::make_environment_config(resolved_environment);
        }
    }
    if (!env_opt && !has_override_environment && !has_skillplan_environment)
    {
        resolved_environment = "dual_gp4";
        env_opt = vamp_env::make_environment_config(resolved_environment);
    }
    if (!env_opt)
    {
        if (resolved_environment.empty())
        {
            throw std::runtime_error("Failed to resolve VAMP environment from skillplan (provide vamp_environment=...)");
        }
        if (has_skillplan_environment && skillplan_environment == resolved_environment)
        {
            throw std::runtime_error(
                "Unsupported VAMP environment: " + resolved_environment +
                " (from skillplan.environment.name; provide vamp_environment=... override)");
        }
        throw std::runtime_error("Unsupported VAMP environment: " + resolved_environment);
    }

    auto instance = vamp_env::make_vamp_instance(*env_opt);
    if (!instance)
    {
        throw std::runtime_error("Failed to create VAMP instance");
    }

    instance->setNumberOfRobots(num_robots);
    instance->setRobotNames(robot_names);
    for (int i = 0; i < num_robots; ++i)
    {
        const int dof_i = (i < static_cast<int>(robot_dofs.size())) ? robot_dofs[static_cast<std::size_t>(i)] : 0;
        if (dof_i > 0)
        {
            instance->setRobotDOF(i, static_cast<std::size_t>(dof_i));
        }
    }
    instance->setVmax(vmax);

    vamp_env::add_environment_obstacles(*env_opt, *instance);
    vamp_env::add_environment_attachments(*env_opt, *instance);

    if (auto defaults = vamp_env::default_base_transforms(*env_opt))
    {
        if (defaults->size() == static_cast<std::size_t>(num_robots))
        {
            for (int i = 0; i < num_robots; ++i)
            {
                instance->setRobotBaseTransform(i, (*defaults)[static_cast<std::size_t>(i)]);
            }
        }
    }

    for (int i = 0; i < num_robots; ++i)
    {
        if (!trajectories[static_cast<std::size_t>(i)].trajectory.empty())
        {
            instance->setStartPose(i, trajectories[static_cast<std::size_t>(i)].trajectory.front().joint_values);
            instance->setGoalPose(i, trajectories[static_cast<std::size_t>(i)].trajectory.back().joint_values);
        }
    }

    const std::string resolved_output_dir = output_dir.empty() ? make_temp_dir("mr_planner_py_skillplan") : output_dir;
    std::filesystem::create_directories(resolved_output_dir);

    std::string resolved_pb = pb_file;
    if (resolved_pb.empty())
    {
        resolved_pb = (std::filesystem::path(resolved_output_dir) /
                       (graph_type == "adg" ? "adg_from_skillplan.pb" : "tpg_from_skillplan.pb"))
                          .string();
    }

    tpg::TPGConfig tpg_cfg;
    tpg_cfg.dt = resolved_dt;
    tpg_cfg.output_dir = resolved_output_dir;
    tpg_cfg.parallel = false;
    tpg_cfg.use_sweep_type2 = true;
    tpg_cfg.shortcut_time = shortcut_time;
    // Match LEGO planning defaults: do not force cross-robot synchronous task execution.
    tpg_cfg.sync_task = false;
    // Keep the default pruning of skippable activities during ADG reconstruction (matches LEGO planning).
    tpg_cfg.preserve_skippable_acts = false;

    if (graph_type == "adg")
    {
        const auto actions_json = plan_json["actions"];
        if (!actions_json.isArray())
        {
            throw std::runtime_error("skillplan actions[] must be an array for ADG");
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
                throw std::runtime_error("actions[" + std::to_string(i) + "] missing required fields (id, robot.id)");
            }
            const std::string aid = action["id"].asString();
            const int rid = action["robot"]["id"].asInt();
            if (rid < 0 || rid >= num_robots)
            {
                throw std::runtime_error("actions[" + std::to_string(i) + "].robot.id out of range");
            }
            const int act_id = next_act_id[static_cast<std::size_t>(rid)]++;
            action_to_act[aid] = ActKey{rid, act_id};
            action_json_by_id.emplace(aid, &action);
            actions_by_robot[static_cast<std::size_t>(rid)].push_back(aid);
        }

        auto act_graph = std::make_shared<ActivityGraph>(num_robots);
        std::vector<std::vector<ActPtr>> act_ptrs(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            const auto &act_ids = actions_by_robot[static_cast<std::size_t>(rid)];
            if (act_ids.empty())
            {
                throw std::runtime_error("no actions found for robot_id=" + std::to_string(rid));
            }
            act_ptrs[static_cast<std::size_t>(rid)].reserve(act_ids.size());
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
                act_ptrs[static_cast<std::size_t>(rid)].push_back(act);
            }
        }

        for (int rid = 0; rid < num_robots; ++rid)
        {
            const auto &traj = trajectories[static_cast<std::size_t>(rid)];
            if (traj.trajectory.size() != traj.act_ids.size())
            {
                throw std::runtime_error("robot " + std::to_string(rid) + " trajectory.act_ids size mismatch");
            }
            const int num_acts = static_cast<int>(act_ptrs[static_cast<std::size_t>(rid)].size());
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
                    throw std::runtime_error("robot " + std::to_string(rid) + " missing samples for act_id=" +
                                             std::to_string(act_id));
                }
                auto &act = act_ptrs[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)];
                act->start_pose = traj.trajectory[static_cast<std::size_t>(first)];
                act->end_pose = traj.trajectory[static_cast<std::size_t>(last)];
            }
        }

        // Seed object nodes + instance scene from initial_scene.objects (optional).
        std::unordered_map<std::string, ObjPtr> last_obj_by_name;
        seed_initial_scene(plan_json, act_graph.get(), *instance, &last_obj_by_name);

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
            if (act_id < 0 || act_id >= static_cast<int>(act_ptrs[static_cast<std::size_t>(rid)].size()))
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
                                                  act_ptrs[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)]);
                        }
                        else if (kind == "detach")
                        {
                            act_graph->detach_obj(oit->second,
                                                  act_ptrs[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)]);
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
                            act_ptrs[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)],
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
                const std::string dep_id = dep.asString();
                const auto dep_it = action_to_act.find(dep_id);
                if (dep_it == action_to_act.end())
                {
                    continue;
                }
                const int from_rid = dep_it->second.robot_id;
                const int from_act = dep_it->second.act_id;
                if (from_rid < 0 || from_rid >= num_robots)
                {
                    continue;
                }
                if (from_act < 0 || from_act >= static_cast<int>(act_ptrs[static_cast<std::size_t>(from_rid)].size()))
                {
                    continue;
                }
                act_ptrs[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)]->type2_prev.push_back(
                    act_ptrs[static_cast<std::size_t>(from_rid)][static_cast<std::size_t>(from_act)]);
            }
        }

        tpg::ADG adg(act_graph);
        if (!adg.init_from_asynctrajs(instance, tpg_cfg, trajectories))
        {
            throw std::runtime_error("Failed to initialize ADG from skillplan trajectories");
        }
        if (!adg.shiftPolicyNodeType2Edges())
        {
            throw std::runtime_error("ADG contains a cycle after shifting policy node type2 edges");
        }
        if (!adg.optimize(instance, tpg_cfg))
        {
            throw std::runtime_error("ADG optimize failed");
        }
        if (!dot_file.empty())
        {
            adg.saveToDotFile(dot_file);
        }
        std::string pb_err;
        if (!mr_planner::graph_proto::write_adg(adg, resolved_pb, &pb_err))
        {
            throw std::runtime_error("Failed to write ADG protobuf: " + pb_err);
        }
    }
    else if (graph_type == "tpg")
    {
        tpg::TPG tpg;
        if (!tpg.init(instance, trajectories, tpg_cfg))
        {
            throw std::runtime_error("Failed to initialize TPG from skillplan trajectories");
        }
        if (!dot_file.empty())
        {
            tpg.saveToDotFile(dot_file);
        }
        std::string pb_err;
        if (!mr_planner::graph_proto::write_tpg(tpg, resolved_pb, &pb_err))
        {
            throw std::runtime_error("Failed to write TPG protobuf: " + pb_err);
        }
    }
    else
    {
        throw std::runtime_error("Invalid graph_type (expected 'tpg' or 'adg'): " + graph_type);
    }

    py::dict out;
    out["output_dir"] = resolved_output_dir;
    out["pb_path"] = resolved_pb;
    out["environment"] = resolved_environment;
    out["graph_type"] = graph_type;
    out["num_robots"] = num_robots;
    return out;
}

py::dict vamp_environment_info(const std::string &vamp_environment)
{
    auto env_opt = vamp_env::make_environment_config(vamp_environment);
    if (!env_opt)
    {
        throw std::runtime_error("Unsupported VAMP environment: " + vamp_environment);
    }

    py::dict out;
    out["environment_name"] = env_opt->environment_name;
    out["move_group"] = env_opt->move_group;
    out["robot_groups"] = env_opt->robot_groups;
    out["hand_groups"] = env_opt->hand_groups;
    out["num_robots"] = static_cast<int>(env_opt->robot_groups.size());
    return out;
}

py::dict shortcut_trajectory(const std::vector<std::vector<std::vector<double>>> &trajectory,
                             const std::string &vamp_environment,
                             double shortcut_time,
                             double dt,
                             int seed,
                             const std::string &method,
                             double vmax)
{
    VampEnvironment env(vamp_environment, vmax, seed);
    return env.shortcut_trajectory(trajectory, shortcut_time, dt, seed, method);
}

std::string graphfile_to_json(const std::string &pb_path)
{
    mr_planner::proto::GraphFile file;
    const auto data = read_file_binary(pb_path);
    if (!file.ParseFromString(data))
    {
        throw std::runtime_error("Failed to parse GraphFile protobuf: " + pb_path);
    }
    google::protobuf::util::JsonPrintOptions opts;
    opts.add_whitespace = true;
    opts.always_print_primitive_fields = true;
    opts.preserve_proto_field_names = true;
    std::string json;
    const auto st = google::protobuf::util::MessageToJsonString(file, &json, opts);
    if (!st.ok())
    {
        throw std::runtime_error("Failed to convert GraphFile protobuf to JSON: " + st.ToString());
    }
    return json;
}

void graphfile_from_json(const std::string &json, const std::string &pb_path)
{
    mr_planner::proto::GraphFile file;
    google::protobuf::util::JsonParseOptions opts;
    opts.ignore_unknown_fields = false;
    const auto st = google::protobuf::util::JsonStringToMessage(json, &file, opts);
    if (!st.ok())
    {
        throw std::runtime_error("Failed to parse GraphFile JSON: " + st.ToString());
    }
    std::string data;
    if (!file.SerializeToString(&data))
    {
        throw std::runtime_error("Failed to serialize GraphFile protobuf");
    }
    write_file_binary(pb_path, data);
}

PYBIND11_MODULE(_mr_planner_core, m)
{
    m.doc() = "mr_planner_core python bindings (planning, environment/collision, skillplan->TPG/ADG, protobuf I/O)";

    py::class_<Object> obj(m, "Object");
    obj.def(py::init<>())
        .def_readwrite("name", &Object::name)
        .def_readwrite("state", &Object::state)
        .def_readwrite("parent_link", &Object::parent_link)
        .def_readwrite("robot_id", &Object::robot_id)
        .def_readwrite("x", &Object::x)
        .def_readwrite("y", &Object::y)
        .def_readwrite("z", &Object::z)
        .def_readwrite("qx", &Object::qx)
        .def_readwrite("qy", &Object::qy)
        .def_readwrite("qz", &Object::qz)
        .def_readwrite("qw", &Object::qw)
        .def_readwrite("x_attach", &Object::x_attach)
        .def_readwrite("y_attach", &Object::y_attach)
        .def_readwrite("z_attach", &Object::z_attach)
        .def_readwrite("qx_attach", &Object::qx_attach)
        .def_readwrite("qy_attach", &Object::qy_attach)
        .def_readwrite("qz_attach", &Object::qz_attach)
        .def_readwrite("qw_attach", &Object::qw_attach)
        .def_readwrite("shape", &Object::shape)
        .def_readwrite("radius", &Object::radius)
        .def_readwrite("length", &Object::length)
        .def_readwrite("width", &Object::width)
        .def_readwrite("height", &Object::height)
        .def_readwrite("mesh_path", &Object::mesh_path);

    py::enum_<Object::State>(obj, "State")
        .value("Static", Object::State::Static)
        .value("Attached", Object::State::Attached)
        .value("Supported", Object::State::Supported)
        .value("Handover", Object::State::Handover)
        .export_values();

    py::enum_<Object::Shape>(obj, "Shape")
        .value("Box", Object::Shape::Box)
        .value("Sphere", Object::Shape::Sphere)
        .value("Cylinder", Object::Shape::Cylinder)
        .value("Mesh", Object::Shape::Mesh)
        .export_values();

    py::class_<VampEnvironment> env(m, "VampEnvironment");
    env.def(py::init<const std::string &, double, int>(),
            py::arg("vamp_environment"),
            py::arg("vmax") = 1.0,
            py::arg("seed") = 1)
        .def_property_readonly("environment_name", &VampEnvironment::environment_name)
        .def("info", &VampEnvironment::info)
        .def("set_random_seed", &VampEnvironment::set_random_seed, py::arg("seed"))
        .def("build_roadmaps",
             &VampEnvironment::build_roadmaps,
             py::arg("roadmap_samples") = 500,
             py::arg("roadmap_max_dist") = 2.0,
             py::arg("seed") = 1)
        .def("load_roadmaps", &VampEnvironment::load_roadmaps, py::arg("path"))
        .def("save_roadmaps", &VampEnvironment::save_roadmaps, py::arg("path"))
        .def("roadmap_cache_stats",
             &VampEnvironment::roadmap_cache_stats,
             py::arg("check_collision") = false,
             py::arg("self_only") = false)
        .def("enable_meshcat",
             &VampEnvironment::enable_meshcat,
             py::arg("host") = "127.0.0.1",
             py::arg("port") = 7600)
        .def("disable_meshcat", &VampEnvironment::disable_meshcat)
        .def("update_scene", &VampEnvironment::update_scene)
        .def("reset_scene", &VampEnvironment::reset_scene, py::arg("reset_sim") = true)
        .def("set_robot_base_transform",
             &VampEnvironment::set_robot_base_transform,
             py::arg("robot_id"),
             py::arg("transform"))
        .def("set_robot_base_transforms", &VampEnvironment::set_robot_base_transforms, py::arg("transforms"))
        .def("seed_initial_scene_from_skillplan",
             &VampEnvironment::seed_initial_scene_from_skillplan,
             py::arg("skillplan_path"))
        .def("in_collision", &VampEnvironment::in_collision, py::arg("joint_positions"), py::arg("self_only") = false)
        .def("in_collision_robot",
             &VampEnvironment::in_collision_robot,
             py::arg("robot_id"),
             py::arg("joint_positions"),
             py::arg("self_only") = false)
        .def("motion_in_collision",
             &VampEnvironment::motion_in_collision,
             py::arg("start"),
             py::arg("goal"),
             py::arg("step_size") = 0.1,
             py::arg("self_only") = false)
        .def("trajectory_in_collision",
             &VampEnvironment::trajectory_in_collision,
             py::arg("trajectory"),
             py::arg("self_only") = false)
        .def("sample_pose", &VampEnvironment::sample_pose, py::arg("robot_id"))
        .def("sample_collision_free_pose",
             &VampEnvironment::sample_collision_free_pose,
             py::arg("robot_id"),
             py::arg("max_attempts") = 200,
             py::arg("self_only") = false)
        .def("end_effector_transform",
             &VampEnvironment::end_effector_transform,
             py::arg("robot_id"),
             py::arg("joint_positions"))
        .def("inverse_kinematics",
             &VampEnvironment::inverse_kinematics,
             py::arg("robot_id"),
             py::arg("target_tf"),
             py::arg("seed") = py::none(),
             py::arg("fixed_joints") = py::none(),
             py::arg("max_restarts") = 25,
             py::arg("max_iters") = 120,
             py::arg("tol_pos") = 0.025,
             py::arg("tol_ang") = 0.2617993877991494,
             py::arg("step_scale") = 1.0,
             py::arg("damping") = 1e-3,
             py::arg("self_only") = false)
        .def("add_object", &VampEnvironment::add_object, py::arg("obj"))
        .def("move_object", &VampEnvironment::move_object, py::arg("obj"))
        .def("remove_object", &VampEnvironment::remove_object, py::arg("name"))
        .def("has_object", &VampEnvironment::has_object, py::arg("name"))
        .def("get_object", &VampEnvironment::get_object, py::arg("name"))
        .def("attach_object",
             &VampEnvironment::attach_object,
             py::arg("name"),
             py::arg("robot_id"),
             py::arg("joint_positions"),
             py::arg("link_name") = "")
        .def("detach_object", &VampEnvironment::detach_object, py::arg("name"), py::arg("robot_id"), py::arg("joint_positions"))
        .def("set_allowed_collision",
             &VampEnvironment::set_allowed_collision,
             py::arg("obj_name"),
             py::arg("link_name"),
             py::arg("allow"))
        .def("print_known_objects", &VampEnvironment::print_known_objects)
        .def("plan",
             &VampEnvironment::plan,
             py::arg("planner") = "composite_rrt",
             py::arg("planning_time") = 3.0,
             py::arg("shortcut_time") = 0.1,
             py::arg("seed") = 1,
             py::arg("sample_attempts") = 200,
             py::arg("max_goal_dist") = 2.0,
             py::arg("dt") = 0.1,
             py::arg("vmax") = 1.0,
             py::arg("start") = py::none(),
             py::arg("goal") = py::none(),
             py::arg("output_dir") = "",
             py::arg("write_tpg") = true,
             py::arg("roadmap_samples") = 500,
             py::arg("roadmap_max_dist") = 2.0,
             py::arg("write_files") = true,
             py::arg("return_trajectories") = false,
             py::arg("roadmap_seed") = py::none())
        .def("shortcut_trajectory",
             &VampEnvironment::shortcut_trajectory,
             py::arg("trajectory"),
             py::arg("shortcut_time") = 0.1,
             py::arg("dt") = 0.1,
             py::arg("seed") = 1,
             py::arg("method") = "thompson")
        .def("play_trajectory",
             &VampEnvironment::play_trajectory,
             py::arg("trajectory"),
             py::arg("dt") = 0.1,
             py::arg("rate") = 1.0)
        .def("play_solution_csv",
             &VampEnvironment::play_solution_csv,
             py::arg("csv_path"),
             py::arg("rate") = 1.0)
        .def("play_execution_graph",
             &VampEnvironment::play_execution_graph,
             py::arg("pb_path"),
             py::arg("rate") = 1.0);

    m.def("plan",
          &plan,
          py::arg("vamp_environment") = "dual_gp4",
          py::arg("planner") = "composite_rrt",
          py::arg("planning_time") = 3.0,
          py::arg("shortcut_time") = 0.1,
          py::arg("seed") = 1,
          py::arg("num_robots") = 2,
          py::arg("dof") = 7,
          py::arg("sample_attempts") = 200,
          py::arg("max_goal_dist") = 2.0,
          py::arg("dt") = 0.1,
          py::arg("vmax") = 1.0,
          py::arg("start") = py::none(),
          py::arg("goal") = py::none(),
          py::arg("output_dir") = "",
          py::arg("write_tpg") = true,
          py::arg("roadmap_samples") = 500,
          py::arg("roadmap_max_dist") = 2.0);

    m.def("skillplan_to_graph",
          &skillplan_to_graph,
          py::arg("skillplan_path"),
          py::arg("graph_type") = "tpg",
          py::arg("vamp_environment") = "",
          py::arg("output_dir") = "",
          py::arg("pb_file") = "",
          py::arg("dot_file") = "",
          py::arg("dt") = 0.0,
          py::arg("vmax") = 1.0,
          py::arg("shortcut_time") = 0.1);

    m.def("skillplan_to_execution_graph",
          &skillplan_to_graph,
          py::arg("skillplan_path"),
          py::arg("graph_type") = "tpg",
          py::arg("vamp_environment") = "",
          py::arg("output_dir") = "",
          py::arg("pb_file") = "",
          py::arg("dot_file") = "",
          py::arg("dt") = 0.0,
          py::arg("vmax") = 1.0,
          py::arg("shortcut_time") = 0.1);

    m.def("vamp_environment_info", &vamp_environment_info, py::arg("vamp_environment"));

    m.def("shortcut_trajectory",
          &shortcut_trajectory,
          py::arg("trajectory"),
          py::arg("vamp_environment") = "dual_gp4",
          py::arg("shortcut_time") = 0.1,
          py::arg("dt") = 0.1,
          py::arg("seed") = 1,
          py::arg("method") = "thompson",
          py::arg("vmax") = 1.0);

    m.def("graphfile_to_json", &graphfile_to_json, py::arg("pb_path"));
    m.def("graphfile_from_json", &graphfile_from_json, py::arg("json"), py::arg("pb_path"));
}
