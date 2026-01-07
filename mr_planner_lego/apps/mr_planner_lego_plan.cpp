#include <mr_planner/applications/lego/lego/Lego.hpp>
#include <mr_planner/applications/lego/lego_config_utils.h>
#include <mr_planner/backends/vamp_env_factory.h>
#include <mr_planner/core/logger.h>
#include <mr_planner/execution/adg.h>
#include <mr_planner/io/graph_proto.h>
#include <mr_planner/io/skillplan.h>
#include <mr_planner/planning/planner.h>
#include <mr_planner/planning/shortcutter.h>
#include <mr_planner/visualization/meshcat_playback.h>

#include <cstdint>
#include <jsoncpp/json/json.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
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
    std::string task{"test"};
    std::string root_pwd{"."};
    std::string config_fname{"config/lego_tasks/user_config.json"};
    std::string steps_dir{"config/lego_tasks/steps"};
    std::string output_dir{};
    int num_robots{2};
    std::string vamp_environment{"dual_gp4"};
    double vmax{1.0};
    double planning_time_limit{5.0};
    double shortcut_time{1.0};
    int seed{1};
    bool parallel{false};
    bool print_debug{false};
    bool export_portable_graph{true};
    bool meshcat{false};
    std::string meshcat_host{"127.0.0.1"};
    int meshcat_port{7600};
    double meshcat_rate{1.0};
};

void usage(const char *prog)
{
    std::cerr << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --task <name>               Task name (default: test)\n"
              << "  --root <path>               Repo root (default: .)\n"
              << "  --config <path>             user_config.json (default: config/lego_tasks/user_config.json)\n"
              << "  --steps-dir <path>          Directory containing <task>_steps.csv and <task>_seq.json\n"
              << "                              (default: config/lego_tasks/steps)\n"
              << "  --output-dir <path>         Planning output directory (default: outputs/lego/<task>)\n"
              << "  --num-robots <int>          Robot count (default: 2)\n"
              << "  --vamp-environment <name>   VAMP environment (default: dual_gp4)\n"
              << "  --vmax <val>                L1 vmax (default: 1.0)\n"
              << "  --planning-time <sec>       RRT segment time limit (default: 5.0)\n"
              << "  --shortcut-time <sec>       ADG shortcut time (default: 1.0; 0 disables)\n"
              << "  --seed <int>                RNG seed (default: 1)\n"
              << "  --parallel                  Enable parallel collision deps\n"
              << "  --no-export-portable        Do not export adg.pb\n"
              << "  --meshcat                    Visualize the resulting schedule via Meshcat\n"
              << "  --meshcat-host <host>        Meshcat bridge host (default: 127.0.0.1)\n"
              << "  --meshcat-port <port>        Meshcat bridge port (default: 7600)\n"
              << "  --meshcat-rate <x>           Playback rate (default: 1.0; 0 disables sleeping)\n"
              << "  --print-debug               Enable debug logging\n";
}

Args parse_args(int argc, char **argv)
{
    Args args;
    bool output_dir_set = false;
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
        if (a == "--task")
        {
            args.task = require_value(a);
        }
        else if (a == "--root")
        {
            args.root_pwd = require_value(a);
        }
        else if (a == "--config")
        {
            args.config_fname = require_value(a);
        }
        else if (a == "--steps-dir")
        {
            args.steps_dir = require_value(a);
        }
        else if (a == "--output-dir")
        {
            args.output_dir = require_value(a);
            output_dir_set = true;
        }
        else if (a == "--num-robots")
        {
            args.num_robots = std::stoi(require_value(a));
        }
        else if (a == "--vamp-environment")
        {
            args.vamp_environment = require_value(a);
        }
        else if (a == "--vmax")
        {
            args.vmax = std::stod(require_value(a));
        }
        else if (a == "--planning-time")
        {
            args.planning_time_limit = std::stod(require_value(a));
        }
        else if (a == "--shortcut-time")
        {
            args.shortcut_time = std::stod(require_value(a));
            if (args.shortcut_time < 0.0)
            {
                throw std::runtime_error("Invalid --shortcut-time");
            }
        }
        else if (a == "--seed")
        {
            args.seed = std::stoi(require_value(a));
        }
        else if (a == "--parallel")
        {
            args.parallel = true;
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
            args.meshcat_port = std::stoi(require_value(a));
            if (args.meshcat_port <= 0 || args.meshcat_port > 65535)
            {
                throw std::runtime_error("Invalid --meshcat-port");
            }
        }
        else if (a == "--meshcat-rate")
        {
            args.meshcat_rate = std::stod(require_value(a));
            if (args.meshcat_rate < 0.0)
            {
                throw std::runtime_error("Invalid --meshcat-rate");
            }
        }
        else if (a == "--no-export-portable")
        {
            args.export_portable_graph = false;
        }
        else if (a == "--print-debug")
        {
            args.print_debug = true;
        }
        else
        {
            throw std::runtime_error("Unknown argument: " + a);
        }
    }
    if (!output_dir_set)
    {
        args.output_dir = (std::filesystem::path("outputs") / "lego" / args.task).string();
    }
    return args;
}

struct GoalPose
{
    bool use_robot{false};
    std::vector<double> joint_values;
};

std::vector<std::string> split_tokens(const std::string &csv_line)
{
    std::vector<std::string> tokens;
    std::stringstream ss(csv_line);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        if (!token.empty() && token.back() == '\r')
        {
            token.pop_back();
        }
        tokens.push_back(token);
    }
    return tokens;
}

bool parse_pose_line(const std::string &line, std::vector<GoalPose> &poses, std::size_t expected_robot_count)
{
    const std::vector<std::string> tokens = split_tokens(line);
    if (tokens.empty())
    {
        return false;
    }

    // Skip headers/non-numeric rows.
    try
    {
        (void)std::stod(tokens.front());
    }
    catch (const std::exception &)
    {
        return false;
    }

    constexpr std::size_t pose_fields = 8;
    const std::size_t total_tokens = tokens.size();
    std::size_t robot_count = expected_robot_count;
    if (robot_count > 0 && total_tokens % robot_count != 0)
    {
        robot_count = 0;
    }
    if (robot_count == 0)
    {
        for (std::size_t dof_candidate : {7ul, 6ul})
        {
            const std::size_t per_robot = pose_fields + dof_candidate;
            if (per_robot > 0 && total_tokens % per_robot == 0)
            {
                robot_count = total_tokens / per_robot;
                break;
            }
        }
    }
    if (robot_count == 0 || total_tokens % robot_count != 0)
    {
        throw std::runtime_error("Unexpected number of columns (" + std::to_string(total_tokens) + ") in steps row");
    }

    const std::size_t tokens_per_robot = total_tokens / robot_count;
    if (tokens_per_robot < pose_fields)
    {
        throw std::runtime_error("Insufficient tokens per robot row");
    }
    const std::size_t dof = tokens_per_robot - pose_fields;

    poses.clear();
    poses.resize(robot_count);
    std::size_t idx = 0;
    for (std::size_t rid = 0; rid < robot_count; ++rid)
    {
        GoalPose pose;
        pose.use_robot = std::stoi(tokens[idx++]) != 0;
        idx += 7;  // skip pose xyz+quat
        pose.joint_values.resize(dof, 0.0);
        for (std::size_t j = 0; j < dof && idx < tokens.size(); ++j)
        {
            pose.joint_values[j] = std::stod(tokens[idx++]) * M_PI / 180.0;
        }
        poses[rid] = std::move(pose);
    }
    return true;
}

void read_steps_file(const std::string &file_path,
                     std::vector<std::vector<GoalPose>> &all_poses,
                     std::size_t expected_robot_count)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open steps file: " + file_path);
    }

    all_poses.clear();
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }
        std::vector<GoalPose> poses;
        if (parse_pose_line(line, poses, expected_robot_count))
        {
            all_poses.push_back(std::move(poses));
        }
    }
    if (all_poses.empty())
    {
        throw std::runtime_error("No poses parsed from steps file: " + file_path);
    }
}

bool read_json(const std::string &path, Json::Value *out)
{
    if (!out)
    {
        return false;
    }
    std::ifstream ifs(path, std::ifstream::binary);
    if (!ifs.is_open())
    {
        return false;
    }
    ifs >> *out;
    return true;
}

class LegoPlanner
{
public:
    LegoPlanner(const std::shared_ptr<PlanInstance> &instance,
                const std::string &output_dir,
                const std::vector<std::string> &group_names,
                const std::vector<std::string> &eof_links,
                double planning_time_limit,
                int seed)
        : instance_(instance),
          output_dir_(output_dir),
          group_names_(group_names),
          eof_links_(eof_links),
          planning_time_limit_(planning_time_limit),
          rrt_seed_(seed)
    {
        if (!instance_)
        {
            throw std::runtime_error("LegoPlanner requires a valid PlanInstance");
        }
        current_joints_.clear();
        for (int rid = 0; rid < instance_->getNumberOfRobots(); ++rid)
        {
            const int dof = static_cast<int>(instance_->getRobotDOF(rid));
            robot_offsets_.push_back(static_cast<int>(current_joints_.size()));
            robot_dofs_.push_back(dof);
            current_joints_.insert(current_joints_.end(), dof, 0.0);
        }
    }

    void setPlanningTimeLimit(double t) { planning_time_limit_ = t; }
    void setEnvironmentName(std::string name) { environment_name_ = std::move(name); }
    void setL1Vmax(double vmax) { l1_vmax_ = vmax; }

    void setCurrentJointsFromRow(const std::vector<GoalPose> &row)
    {
        for (std::size_t rid = 0; rid < row.size() && rid < robot_offsets_.size(); ++rid)
        {
            const auto &q = row[rid].joint_values;
            const int dof = robot_dofs_[rid];
            const int off = robot_offsets_[rid];
            for (int j = 0; j < dof && j < static_cast<int>(q.size()); ++j)
            {
                current_joints_[static_cast<std::size_t>(off + j)] = q[static_cast<std::size_t>(j)];
            }
        }
    }

    bool setLegoFactory(const std::string &config_fname,
                        const std::string &root_pwd,
                        const std::string &task_fname,
                        const std::string &task_name)
    {
        Json::Value config;
        if (!read_json(config_fname, &config))
        {
            throw std::runtime_error("Failed to read config: " + config_fname);
        }
        auto robot_calibrations = lego_manipulation::lego::loadRobotCalibrations(config, root_pwd);

        const std::string plate_calibration_fname =
            lego_manipulation::lego::resolveRepoPath(root_pwd, config["plate_calibration_fname"].asString());
        const std::string env_setup_folder =
            lego_manipulation::lego::resolveRepoPath(root_pwd, config["env_setup_folder"].asString());
        const std::string env_setup_fname =
            (std::filesystem::path(env_setup_folder) / ("env_setup_" + task_name + ".json")).string();
        const std::string lego_lib_fname =
            lego_manipulation::lego::resolveRepoPath(root_pwd, config["lego_lib_fname"].asString());
        const std::string world_base_fname =
            lego_manipulation::lego::resolveRepoPath(root_pwd, config["world_base_fname"].asString());

        std::ifstream task_file(task_fname, std::ifstream::binary);
        if (!task_file.is_open())
        {
            throw std::runtime_error("Failed to open task file: " + task_fname);
        }
        task_file >> task_json_;
        num_tasks_ = static_cast<int>(task_json_.size());

        const bool assemble = config["Start_with_Assemble"].asBool();
        lego_ptr_ = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr_->setup(env_setup_fname,
                         lego_lib_fname,
                         plate_calibration_fname,
                         assemble,
                         task_json_,
                         world_base_fname,
                         robot_calibrations);
        plan_name_ = task_name;
        return true;
    }

    bool initLegoPositions()
    {
        if (!lego_ptr_)
        {
            return false;
        }
        addCollisionObject("table");
        instance_->setObjectColor("table", 1.0, 1.0, 1.0, 1.0);
        for (const auto &name : lego_ptr_->get_brick_names())
        {
            if (name.find("station") != std::string::npos)
            {
                continue;
            }
            addCollisionObject(name);
        }
        return true;
    }

    bool resetSceneForPlayback()
    {
        if (!instance_ || !lego_ptr_)
        {
            return false;
        }

        // Reset the instance scene and clear the Meshcat viewer, then re-add all
        // LEGO objects so playback starts from a clean, deterministic scene.
        instance_->resetScene(true);
        return initLegoPositions();
    }

    std::shared_ptr<tpg::ADG> getAdg() const { return adg_; }
    const std::vector<MRTrajectory> &plans() const { return plans_; }

    bool sequential_plan(int start_task_idx,
                         const std::vector<std::vector<GoalPose>> &all_poses,
                         const ShortcutOptions &sc_options,
                         const ShortcutOptions &async_sc_options,
                         const tpg::TPGConfig &tpg_config)
    {
        const int robot_count = static_cast<int>(group_names_.size());
        auto act_graph = std::make_shared<ActivityGraph>(robot_count);
        int task_idx = start_task_idx;
        int manip_type = getManipType(task_idx);
        int mode = 0;
        std::string brick_name;
        std::vector<std::string> bottom_bricks, side_bricks, around_bricks, top_bricks, sup_side_bricks;
        int robot_id = -1;
        int sup_robot = -1;
        ActPtr last_drop_act, last_receive_act;

        std::vector<std::vector<double>> robot_init_joints(robot_count);
        for (int rid = 0; rid < robot_count; ++rid)
        {
            robot_init_joints[rid] = getRobotJointValues(rid);
            if (robot_init_joints[rid].empty())
            {
                robot_init_joints[rid].assign(instance_->getRobotDOF(rid), 0.0);
            }
        }

        std::vector<Activity::Type> last_act_type(robot_count, Activity::Type::home);

        auto addIdleHome = [&](std::initializer_list<int> active_ids) {
            std::vector<bool> skip(robot_count, false);
            for (int rid : active_ids)
            {
                if (rid >= 0 && rid < robot_count)
                {
                    skip[rid] = true;
                }
            }
            for (int rid = 0; rid < robot_count; ++rid)
            {
                if (!skip[rid])
                {
                    act_graph->add_act(rid, Activity::Type::home);
                }
            }
        };

        ObjPtr table = act_graph->add_obj(instance_->getObject("table"));
        table->fixed = true;
        for (const auto &fixed_brick : lego_ptr_->get_fixed_bricks_names())
        {
            ObjPtr brick = act_graph->add_obj(instance_->getObject(fixed_brick));
            brick->fixed = true;
        }

        int i = 0;
        while (task_idx <= num_tasks_)
        {
            std::vector<RobotPose> start_poses;
            start_poses.reserve(robot_count);
            for (int rid = 0; rid < robot_count; ++rid)
            {
                RobotPose pose = instance_->initRobotPose(rid);
                pose.joint_values = robot_init_joints[rid];
                start_poses.push_back(pose);
            }

            getLegoBrickName(task_idx, brick_name);
            const bool from_station = (brick_name.find("station") != std::string::npos);

            if (mode == 0)
            {
                for (int rid = 0; rid < robot_count; ++rid)
                {
                    act_graph->add_act(rid, Activity::Type::home);
                }
            }
            if (mode == 1)
            {
                robot_id = getRobot(task_idx);
                sup_robot = getSupportRobot(task_idx);

                Object obj = getLegoStart(brick_name);
                ObjPtr obj_node = act_graph->add_obj(obj);
                if (from_station)
                {
                    obj_node->vanish = true;
                    addCollisionObject(brick_name);
                }

                act_graph->add_act(robot_id, Activity::Type::pick_tilt_up);
                addIdleHome({robot_id});
            }
            if (mode == 2)
            {
                act_graph->add_act(robot_id, Activity::Type::pick_up);
                addIdleHome({robot_id});
                act_graph->set_collision(brick_name,
                                         eof_links_[robot_id],
                                         act_graph->get_last_act(robot_id, Activity::Type::pick_up),
                                         true);
            }
            if (mode == 3)
            {
                act_graph->add_act(robot_id, Activity::Type::pick_down);
                addIdleHome({robot_id});
                getLegoBottom(brick_name, task_idx, false, bottom_bricks);
                for (const auto &bottom_brick : bottom_bricks)
                {
                    act_graph->set_collision(bottom_brick,
                                             eof_links_[robot_id],
                                             act_graph->get_last_act(robot_id, Activity::Type::pick_down),
                                             true);
                    act_graph->set_collision(bottom_brick,
                                             brick_name,
                                             act_graph->get_last_act(robot_id, Activity::Type::pick_down),
                                             true);
                }
            }
            if (mode == 4)
            {
                act_graph->add_act(robot_id, Activity::Type::pick_twist);
                act_graph->attach_obj(act_graph->get_last_obj(brick_name),
                                      eof_links_[robot_id],
                                      act_graph->get_last_act(robot_id, Activity::Type::pick_twist));
                addIdleHome({robot_id});
            }
            if (mode == 5)
            {
                act_graph->add_act(robot_id, Activity::Type::pick_twist_up);
                addIdleHome({robot_id});
            }
            if (mode == 6)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                addIdleHome({robot_id});
                for (const auto &bottom_brick : bottom_bricks)
                {
                    act_graph->set_collision(bottom_brick,
                                             eof_links_[robot_id],
                                             act_graph->get_last_act(robot_id, Activity::Type::home),
                                             false);
                }
            }
            if (mode == 7)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot, Activity::Type::support_pre);
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
            }
            if (mode == 8)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1)
                {
                    auto sup_act = act_graph->add_act(sup_robot, Activity::Type::support);
                    if (last_drop_act != nullptr && last_drop_act->robot_id != sup_robot)
                    {
                        act_graph->add_type2_dep(sup_act, last_drop_act);
                    }
                    getLegoSuppNearby(task_idx, sup_side_bricks);
                    for (const auto &sup_side_brick : sup_side_bricks)
                    {
                        act_graph->set_collision(sup_side_brick,
                                                 eof_links_[sup_robot],
                                                 act_graph->get_last_act(sup_robot, Activity::Type::support),
                                                 true);
                    }
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
            }
            if (mode == 9)
            {
                act_graph->add_act(robot_id, Activity::Type::drop_tilt_up);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
            }
            if (mode == 10)
            {
                act_graph->add_act(robot_id, Activity::Type::drop_up);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
                getLegoAround(task_idx, brick_name, around_bricks);
                for (const auto &around_brick : around_bricks)
                {
                    act_graph->set_collision(around_brick,
                                             brick_name,
                                             act_graph->get_last_act(robot_id, Activity::Type::drop_up),
                                             true);
                }
            }
            if (mode == 11)
            {
                if (sup_robot > -1)
                {
                    act_graph->add_act(robot_id, Activity::Type::drop_down, act_graph->get_last_act(sup_robot, Activity::Type::support));
                    act_graph->add_act(sup_robot, Activity::Type::support);
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    act_graph->add_act(robot_id, Activity::Type::drop_down);
                    addIdleHome({robot_id});
                }
                if (last_drop_act != nullptr && last_drop_act->robot_id != robot_id)
                {
                    act_graph->add_type2_dep(act_graph->get_last_act(robot_id, Activity::Type::drop_down), last_drop_act);
                }
                getLegoBottom(brick_name, task_idx, true, bottom_bricks);
                for (const auto &bottom_brick : bottom_bricks)
                {
                    act_graph->set_collision(bottom_brick,
                                             brick_name,
                                             act_graph->get_last_act(robot_id, Activity::Type::drop_down),
                                             true);
                }
                getLegoTwistNext(task_idx, brick_name, side_bricks);
                for (const auto &side_brick : side_bricks)
                {
                    act_graph->set_collision(side_brick,
                                             eof_links_[robot_id],
                                             act_graph->get_last_act(robot_id, Activity::Type::drop_down),
                                             true);
                }
            }
            if (mode == 12)
            {
                Object obj = getLegoTarget(task_idx);
                act_graph->add_obj(obj);
                act_graph->add_act(robot_id, Activity::Type::drop_twist);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
                act_graph->detach_obj(act_graph->get_last_obj(brick_name),
                                      act_graph->get_last_act(robot_id, Activity::Type::drop_twist));
            }
            if (mode == 13)
            {
                last_drop_act = act_graph->add_act(robot_id, Activity::Type::drop_twist_up);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
                for (const auto &side_brick : side_bricks)
                {
                    act_graph->set_collision(side_brick,
                                             eof_links_[robot_id],
                                             act_graph->get_last_act(robot_id, Activity::Type::drop_twist_up),
                                             false);
                }
            }
            if (mode == 14)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot, Activity::Type::support);
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
                act_graph->set_collision(brick_name,
                                         eof_links_[robot_id],
                                         act_graph->get_last_act(robot_id, Activity::Type::home),
                                         false);
            }
            if (mode == 15)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot,
                                       Activity::Type::support_pre,
                                       act_graph->get_last_act(robot_id, Activity::Type::drop_twist_up));
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
            }
            if (mode == 16)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                if (sup_robot > -1)
                {
                    act_graph->add_act(sup_robot, Activity::Type::home);
                    for (const auto &sup_side_brick : sup_side_bricks)
                    {
                        act_graph->set_collision(sup_side_brick,
                                                 eof_links_[sup_robot],
                                                 act_graph->get_last_act(sup_robot, Activity::Type::home),
                                                 false);
                    }
                    addIdleHome({robot_id, sup_robot});
                }
                else
                {
                    addIdleHome({robot_id});
                }
            }

            // Handover modes (manipulate_type==1)
            if (mode == 17)
            {
                robot_id = getRobot(task_idx);
                sup_robot = getSupportRobot(task_idx);

                if (robot_id < 0 || robot_id >= robot_count)
                {
                    log("Invalid primary robot (" + std::to_string(robot_id) + ") for task " + std::to_string(task_idx) +
                            " when building handover plan",
                        LogLevel::ERROR);
                    return false;
                }
                if (sup_robot < 0 || sup_robot >= robot_count || sup_robot == robot_id)
                {
                    log("Invalid support robot (" + std::to_string(sup_robot) + ") for task " + std::to_string(task_idx) +
                            " (manipulate_type==1 requires two distinct robots)",
                        LogLevel::ERROR);
                    return false;
                }

                Object obj = getLegoStart(brick_name);
                ObjPtr obj_node = act_graph->add_obj(obj);
                if (from_station)
                {
                    obj_node->vanish = true;
                    addCollisionObject(brick_name);
                }
                obj_node->handover = true;

                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_tilt_up);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 18)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_up);
                act_graph->set_collision(brick_name,
                                         eof_links_[sup_robot],
                                         act_graph->get_last_act(sup_robot, Activity::Type::pick_up),
                                         true);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 19)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_down);
                getLegoBottom(brick_name, task_idx, false, bottom_bricks);
                for (const auto &bottom_brick : bottom_bricks)
                {
                    act_graph->set_collision(bottom_brick,
                                             eof_links_[sup_robot],
                                             act_graph->get_last_act(sup_robot, Activity::Type::pick_down),
                                             true);
                    act_graph->set_collision(bottom_brick,
                                             brick_name,
                                             act_graph->get_last_act(sup_robot, Activity::Type::pick_down),
                                             true);
                }
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 20)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_twist);
                act_graph->attach_obj(act_graph->get_last_obj(brick_name),
                                      eof_links_[sup_robot],
                                      act_graph->get_last_act(sup_robot, Activity::Type::pick_twist));
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 21)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::pick_twist_up);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 22)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::home);
                for (const auto &bottom_brick : bottom_bricks)
                {
                    act_graph->set_collision(bottom_brick,
                                             eof_links_[sup_robot],
                                             act_graph->get_last_act(sup_robot, Activity::Type::home),
                                             false);
                }
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 23)
            {
                act_graph->add_act(robot_id, Activity::Type::home_receive);
                act_graph->add_act(sup_robot, Activity::Type::home);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 24)
            {
                act_graph->add_act(robot_id, Activity::Type::home_receive);
                act_graph->add_act(sup_robot, Activity::Type::home_handover);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 25)
            {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::home_handover);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 26)
            {
                last_receive_act = act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_up);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 27)
            {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_down, last_receive_act);
                act_graph->set_collision(brick_name,
                                         eof_links_[robot_id],
                                         act_graph->get_last_act(robot_id, Activity::Type::receive),
                                         true);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 28)
            {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_twist);
                Object obj = getLegoHandover(task_idx, start_poses[sup_robot]);
                ObjPtr objptr = act_graph->add_obj(obj);
                objptr->handover = true;
                act_graph->detach_obj(act_graph->get_last_obj(brick_name),
                                      act_graph->get_last_act(sup_robot, Activity::Type::handover_twist));
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 29)
            {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::handover_twist_up);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 30)
            {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::home_handover);
                act_graph->set_collision(brick_name,
                                         eof_links_[sup_robot],
                                         act_graph->get_last_act(sup_robot, Activity::Type::home_handover),
                                         false);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 31)
            {
                act_graph->add_act(robot_id, Activity::Type::receive);
                act_graph->add_act(sup_robot, Activity::Type::home);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 32)
            {
                act_graph->add_act(robot_id,
                                   Activity::Type::home_receive,
                                   act_graph->get_last_act(sup_robot, Activity::Type::handover_twist_up));
                act_graph->add_act(sup_robot, Activity::Type::home);
                act_graph->attach_obj(act_graph->get_last_obj(brick_name),
                                      eof_links_[robot_id],
                                      act_graph->get_last_act(robot_id, Activity::Type::home_receive));
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 33)
            {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down_pre);
                act_graph->add_act(sup_robot, Activity::Type::home);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 34)
            {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down);
                act_graph->add_act(sup_robot, Activity::Type::home);
                addIdleHome({robot_id, sup_robot});

                getLegoAround(task_idx, brick_name, around_bricks);
                for (const auto &around_brick : around_bricks)
                {
                    act_graph->set_collision(around_brick,
                                             brick_name,
                                             act_graph->get_last_act(robot_id, Activity::Type::place_tilt_down),
                                             true);
                }
            }
            if (mode == 35)
            {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down);
                act_graph->add_act(sup_robot, Activity::Type::press_up);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 36)
            {
                act_graph->add_act(robot_id, Activity::Type::place_tilt_down);
                auto press_act = act_graph->add_act(sup_robot, Activity::Type::press_down);
                if (last_drop_act != nullptr && last_drop_act->robot_id != sup_robot)
                {
                    act_graph->add_type2_dep(press_act, last_drop_act);
                }
                getLegoTop(brick_name, task_idx, true, top_bricks);
                for (const auto &top_brick : top_bricks)
                {
                    act_graph->set_collision(top_brick,
                                             eof_links_[sup_robot],
                                             act_graph->get_last_act(sup_robot, Activity::Type::press_down),
                                             true);
                }
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 37)
            {
                act_graph->add_act(robot_id, Activity::Type::place_down);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 38)
            {
                act_graph->add_act(robot_id,
                                   Activity::Type::place_up,
                                   act_graph->get_last_act(sup_robot, Activity::Type::press_down));
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                if (last_drop_act != nullptr && last_drop_act->robot_id != robot_id)
                {
                    act_graph->add_type2_dep(act_graph->get_last_act(robot_id, Activity::Type::place_up), last_drop_act);
                }
                for (const auto &top_brick : top_bricks)
                {
                    act_graph->set_collision(top_brick,
                                             brick_name,
                                             act_graph->get_last_act(robot_id, Activity::Type::place_up),
                                             true);
                }
                getLegoTwistNext(task_idx, brick_name, side_bricks);
                for (const auto &side_brick : side_bricks)
                {
                    act_graph->set_collision(side_brick,
                                             eof_links_[robot_id],
                                             act_graph->get_last_act(robot_id, Activity::Type::place_up),
                                             true);
                }
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 39)
            {
                Object obj = getLegoTarget(task_idx);
                ObjPtr objptr = act_graph->add_obj(obj);
                objptr->handover = true;
                act_graph->add_act(robot_id, Activity::Type::place_twist);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                act_graph->detach_obj(act_graph->get_last_obj(brick_name),
                                      act_graph->get_last_act(robot_id, Activity::Type::place_twist));
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 40)
            {
                last_drop_act = act_graph->add_act(robot_id, Activity::Type::place_twist_down);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 41)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::press_down);
                act_graph->set_collision(brick_name,
                                         eof_links_[robot_id],
                                         act_graph->get_last_act(robot_id, Activity::Type::home),
                                         false);
                for (const auto &side_brick : side_bricks)
                {
                    act_graph->set_collision(side_brick,
                                             eof_links_[robot_id],
                                             act_graph->get_last_act(robot_id, Activity::Type::place_twist_down),
                                             false);
                }
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 42)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot,
                                   Activity::Type::press_up,
                                   act_graph->get_last_act(robot_id, Activity::Type::place_twist_down));
                addIdleHome({robot_id, sup_robot});
            }
            if (mode == 43)
            {
                act_graph->add_act(robot_id, Activity::Type::home);
                act_graph->add_act(sup_robot, Activity::Type::home);
                for (const auto &top_brick : top_bricks)
                {
                    act_graph->set_collision(top_brick,
                                             eof_links_[sup_robot],
                                             act_graph->get_last_act(sup_robot, Activity::Type::home),
                                             false);
                }
                addIdleHome({robot_id, sup_robot});
            }

            int plan_robot_id = -1;
            std::vector<ActPtr> current_acts(robot_count);
            for (int rid = 0; rid < robot_count; ++rid)
            {
                current_acts[rid] = act_graph->get_last_act(rid);
                auto act = current_acts[rid];
                if (!act)
                {
                    continue;
                }
                if (act->type != last_act_type[rid])
                {
                    if (plan_robot_id != -1 && plan_robot_id != rid)
                    {
                        log("More than one robot moving in sequential planning (mode " + std::to_string(mode) + ")",
                            LogLevel::WARN);
                    }
                    plan_robot_id = rid;
                }
            }

            std::vector<RobotPose> goal_poses(robot_count);
            for (int rid = 0; rid < robot_count; ++rid)
            {
                RobotPose start_pose = start_poses[rid];
                auto act = current_acts[rid];
                if (!act)
                {
                    continue;
                }
                act->start_pose = start_pose;
                RobotPose end_pose = start_pose;
                if (i < static_cast<int>(all_poses.size()) && rid < static_cast<int>(all_poses[i].size()))
                {
                    end_pose.joint_values = all_poses[static_cast<std::size_t>(i)][static_cast<std::size_t>(rid)].joint_values;
                }
                act->end_pose = end_pose;
                goal_poses[rid] = end_pose;
                robot_init_joints[rid] = end_pose.joint_values;
                last_act_type[rid] = act->type;
            }

            // Update instance scene for this mode before planning.
            for (int rid = 0; rid < robot_count; ++rid)
            {
                auto act = current_acts[rid];
                if (!act)
                {
                    continue;
                }
                for (const auto &obj_node : act->obj_attached)
                {
                    attachCollisionObject(obj_node->name(), rid, eof_links_[rid], act->start_pose);
                }
                for (const auto &obj_node : act->obj_detached)
                {
                    detachCollisionObject(obj_node->name(), act->start_pose);
                }
                for (const auto &col_node : act->collision_nodes)
                {
                    setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
                }
            }

            MRTrajectory solution;
            double t_plan = 0.0;
            bool success = planAndMove(goal_poses, tpg_config, plan_robot_id, current_acts, solution, &t_plan);
            if (!success)
            {
                log("Failed to plan mode " + std::to_string(mode) + " task " + std::to_string(task_idx), LogLevel::ERROR);
                return false;
            }

            auto tic = std::chrono::high_resolution_clock::now();
            double max_diff = 0.0;
            for (int rid = 0; rid < robot_count && rid < static_cast<int>(solution.size()); ++rid)
            {
                const auto &traj = solution[rid];
                if (traj.trajectory.empty())
                {
                    continue;
                }
                double expected = instance_->computeDistance(traj.trajectory.front(), traj.trajectory.back()) / instance_->getVMax(rid);
                double diff = traj.cost - expected;
                max_diff = std::max(max_diff, diff);
            }

            MRTrajectory rediscretized_solution;
            rediscretizeSolution(instance_, solution, rediscretized_solution, tpg_config.dt);
            MRTrajectory smoothed_solution;
            Shortcutter shortcutter(instance_, sc_options);
            if (max_diff > 0.1)
            {
                const bool success_sc = shortcutter.shortcutSolution(rediscretized_solution, smoothed_solution);
                if (!success_sc || smoothed_solution.empty())
                {
                    smoothed_solution = rediscretized_solution;
                }
            }
            else
            {
                smoothed_solution = rediscretized_solution;
            }
            removeWait(instance_, smoothed_solution);

            auto toc = std::chrono::high_resolution_clock::now();
            double t_smooth = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
            planning_time_ += t_plan + t_smooth;

            plans_.push_back(std::move(smoothed_solution));

            if (mode == 0 && manip_type == 1)
            {
                mode = 17;
            }
            else
            {
                mode++;
                if (mode == 17 || mode == 44)
                {
                    task_idx++;
                    manip_type = getManipType(task_idx);
                    mode = (manip_type == 0) ? 1 : 17;
                }
            }
            i++;
        }

        // Add any remaining bricks as scene objects.
        for (const auto &name : lego_ptr_->get_brick_names())
        {
            if (name.find("station") != std::string::npos)
            {
                continue;
            }
            if (act_graph->get_last_obj(name) == nullptr)
            {
                act_graph->add_obj(instance_->getObject(name));
            }
        }

        log("Planning time total " + std::to_string(planning_time_) + " s", LogLevel::INFO);

        auto adg_tic = std::chrono::high_resolution_clock::now();
        MRTrajectory async_sols = async_shortcut(async_sc_options, act_graph);

        {
            std::vector<mr_planner::skillplan::RobotSpec> robots;
            robots.reserve(static_cast<std::size_t>(instance_->getNumberOfRobots()));
            for (int i = 0; i < instance_->getNumberOfRobots(); ++i)
            {
                mr_planner::skillplan::RobotSpec spec;
                spec.id = i;
                spec.name =
                    (i >= 0 && i < static_cast<int>(group_names_.size())) ? group_names_[static_cast<std::size_t>(i)]
                                                                          : ("robot_" + std::to_string(i));
                spec.dof = static_cast<int>(instance_->getRobotDOF(i));
                if (i >= 0 && i < static_cast<int>(eof_links_.size()))
                {
                    spec.end_effector_link = eof_links_[static_cast<std::size_t>(i)];
                }
                robots.push_back(std::move(spec));
            }

            mr_planner::skillplan::ExportOptions opts;
            opts.plan_name = plan_name_;
            opts.environment_name = environment_name_;
            opts.backend_type = instance_->instanceType();
            opts.l1_vmax = l1_vmax_;

            mr_planner::skillplan::ActivityGraphExportOptions sp_opts;
            sp_opts.prune_stationary = true;
            sp_opts.pad_to_next_activity = true;
            const auto json = mr_planner::skillplan::make_activity_graph_plan(robots, act_graph, async_sols, opts, sp_opts);
            std::string err;
            const std::string out_path = output_dir_ + "/skillplan.json";
            if (!mr_planner::skillplan::write_json_to_file(json, out_path, &err))
            {
                log("Failed to write skillplan.json to " + out_path + ": " + err, LogLevel::WARN);
            }
            else
            {
                log("Wrote skillplan.json to " + out_path, LogLevel::INFO);
            }
        }

        adg_ = std::make_shared<tpg::ADG>(act_graph);
        if (!adg_->init_from_asynctrajs(instance_, tpg_config, async_sols))
        {
            adg_->saveToDotFile(output_dir_ + "/adg.dot");
            return false;
        }
        if (!adg_->shiftPolicyNodeType2Edges())
        {
            return false;
        }
        auto adg_toc = std::chrono::high_resolution_clock::now();
        double t_adg = std::chrono::duration_cast<std::chrono::nanoseconds>(adg_toc - adg_tic).count() * 1e-9;
        log("ADG build time " + std::to_string(t_adg) + " s", LogLevel::INFO);
        return true;
    }

private:
    std::vector<double> getRobotJointValues(int robot_id) const
    {
        if (robot_id < 0 || robot_id >= static_cast<int>(robot_offsets_.size()))
        {
            return {};
        }
        const int off = robot_offsets_[static_cast<std::size_t>(robot_id)];
        const int dof = robot_dofs_[static_cast<std::size_t>(robot_id)];
        if (off < 0 || off + dof > static_cast<int>(current_joints_.size()))
        {
            return {};
        }
        return std::vector<double>(current_joints_.begin() + off, current_joints_.begin() + off + dof);
    }

    void addCollisionObject(const std::string &name)
    {
        Object obj = getLegoStart(name);
        instance_->addMoveableObject(obj);
        instance_->updateScene();
        instance_->setCollision(name, name, true);
        instance_->updateScene();
    }

    void attachCollisionObject(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose)
    {
        instance_->attachObjectToRobot(name, robot_id, link_name, pose);
        instance_->updateScene();
    }

    void detachCollisionObject(const std::string &name, const RobotPose &pose)
    {
        instance_->detachObjectFromRobot(name, pose);
        instance_->updateScene();
    }

    bool setCollision(const std::string &object_id, const std::string &link_name, bool allow)
    {
        instance_->setCollision(object_id, link_name, allow);
        if (instance_->instanceType() == "VampInstance" && object_id != link_name)
        {
            // VAMP treats attached objects as robot-local attachments. To allow collisions between an attached object and
            // another robot's link, we also register the reverse mapping (link_name -> object_id) so the attachment
            // override path is hit when the object becomes attached.
            instance_->setCollision(link_name, object_id, allow);

            // Additionally, register a canonicalized "<robot>::<link>" form so the collision target matches the
            // naming used by VAMP collision/debug tooling (e.g., "left_arm::link_tool").
            std::string canonical = link_name;
            for (const auto &robot_name : group_names_)
            {
                const std::string prefix = robot_name + "_";
                if (link_name.rfind(prefix, 0) == 0 && link_name.size() > prefix.size())
                {
                    canonical = robot_name + "::" + link_name.substr(prefix.size());
                    break;
                }
            }
            if (canonical != link_name)
            {
                instance_->setCollision(canonical, object_id, allow);
            }
        }
        instance_->updateScene();
        return true;
    }

    Object getLegoStart(const std::string &brick_name)
    {
        Object obj;
        obj.name = brick_name;
        obj.state = Object::State::Static;
        obj.parent_link = "world";
        obj.shape = Object::Shape::Box;

        lego_manipulation::Pose box_pose;
        if (brick_name == "table")
        {
            lego_ptr_->get_table_size(obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_table_pose();
        }
        else
        {
            lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_init_brick_pose(brick_name);
        }
        obj.x = box_pose.position.x();
        obj.y = box_pose.position.y();
        obj.z = box_pose.position.z() - obj.height / 2;
        obj.qx = box_pose.orientation.x();
        obj.qy = box_pose.orientation.y();
        obj.qz = box_pose.orientation.z();
        obj.qw = box_pose.orientation.w();
        return obj;
    }

    Object getLegoTarget(int task_idx)
    {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        std::string brick_name =
            lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asString());

        Eigen::Matrix4d brick_pose_mtx;
        lego_ptr_->calc_bric_asssemble_pose(brick_name,
                                            cur_graph_node["x"].asInt(),
                                            cur_graph_node["y"].asInt(),
                                            cur_graph_node["z"].asInt(),
                                            cur_graph_node["ori"].asInt(),
                                            brick_pose_mtx);

        Object obj = instance_->getObject(brick_name);
        obj.state = Object::State::Static;
        obj.parent_link = "world";
        lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);
        obj.x = brick_pose_mtx(0, 3);
        obj.y = brick_pose_mtx(1, 3);
        obj.z = brick_pose_mtx(2, 3) - obj.height / 2;
        Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
        obj.qx = quat.x();
        obj.qy = quat.y();
        obj.qz = quat.z();
        obj.qw = quat.w();
        return obj;
    }

    Object getLegoHandover(int task_idx, const RobotPose &start_pose)
    {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        const std::string brick_name =
            lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asString());

        const int robot_id = start_pose.robot_id;
        const int brick_id = cur_graph_node["brick_id"].asInt();
        const int press_side = cur_graph_node["press_side"].asInt();
        const int press_offset = cur_graph_node["press_offset"].asInt();

        Object obj = instance_->getObject(brick_name);
        obj.name = brick_name;
        obj.state = Object::State::Handover;
        if (robot_id >= 0 && robot_id < static_cast<int>(eof_links_.size()))
        {
            obj.parent_link = eof_links_[robot_id];
        }
        obj.shape = Object::Shape::Box;
        lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);

        Eigen::Matrix4d brick_loc = Eigen::Matrix4d::Identity();
        lego_manipulation::math::VectorJd press_joints = Eigen::MatrixXd::Zero(6, 1);
        for (int i = 0; i < 6 && i < static_cast<int>(start_pose.joint_values.size()); ++i)
        {
            press_joints(i, 0) = start_pose.joint_values[static_cast<std::size_t>(i)];
        }
        press_joints = press_joints * 180.0 / M_PI;
        lego_ptr_->lego_pose_from_press_pose(press_joints, robot_id, brick_id, press_side, press_offset, brick_loc);

        obj.x = brick_loc(0, 3);
        obj.y = brick_loc(1, 3);
        obj.z = brick_loc(2, 3) - obj.height / 2.0;
        Eigen::Quaterniond quat(brick_loc.block<3, 3>(0, 0));
        obj.qx = quat.x();
        obj.qy = quat.y();
        obj.qz = quat.z();
        obj.qw = quat.w();
        return obj;
    }

    void getLegoBrickName(int task_idx, std::string &brick_name)
    {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asString());
    }

    int getRobot(int task_idx) const
    {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        int manip_type = cur_graph_node.get("manipulate_type", 0).asInt();
        int rid = cur_graph_node.get("robot_id", 0).asInt();
        if (manip_type == 1)
        {
            rid = cur_graph_node.get("sup_robot_id", rid).asInt();
        }
        return rid - 1;
    }

    int getSupportRobot(int task_idx) const
    {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        int manip_type = cur_graph_node.get("manipulate_type", 0).asInt();
        int sid = cur_graph_node.get("sup_robot_id", 0).asInt();
        if (manip_type == 1)
        {
            sid = cur_graph_node.get("robot_id", sid).asInt();
        }
        return sid - 1;
    }

    int getManipType(int task_idx) const
    {
        if (task_idx <= 0 || task_idx > num_tasks_)
        {
            return 0;
        }
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        return cur_graph_node.get("manipulate_type", 0).asInt();
    }

    void getLegoBottom(const std::string &brick_name, int task_idx, bool target_pose, std::vector<std::string> &bot_objects)
    {
        bot_objects.clear();
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        const int brick_type = cur_graph_node["brick_id"].asInt();
        const int brick_ori = cur_graph_node["ori"].asInt();
        const int x = cur_graph_node["x"].asInt();
        const int y = cur_graph_node["y"].asInt();
        int z = cur_graph_node["z"].asInt();
        if (!target_pose)
        {
            z = 1;
        }
        std::vector<std::vector<std::vector<std::string>>> world_grid =
            lego_ptr_->gen_world_grid_from_graph(task_json_, task_idx, 48, 48, 48);
        lego_ptr_->get_lego_below(x, y, z, brick_ori, brick_type, world_grid, bot_objects);
        for (auto &o : bot_objects)
        {
            if (o == brick_name)
            {
                o.clear();
            }
        }
        bot_objects.erase(std::remove_if(bot_objects.begin(), bot_objects.end(), [](const std::string &s) { return s.empty(); }),
                          bot_objects.end());
    }

    void getLegoTop(const std::string &brick_name, int task_idx, bool target_pose, std::vector<std::string> &top_objects)
    {
        top_objects.clear();
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        const int brick_type = cur_graph_node["brick_id"].asInt();
        const int brick_ori = cur_graph_node["ori"].asInt();
        const int x = cur_graph_node["x"].asInt();
        const int y = cur_graph_node["y"].asInt();
        int z = cur_graph_node["z"].asInt();
        if (!target_pose)
        {
            z = 1;
        }

        std::vector<std::vector<std::vector<std::string>>> world_grid =
            lego_ptr_->gen_world_grid_from_graph(task_json_, task_idx, 48, 48, 48);
        lego_ptr_->get_lego_above(x, y, z + 2, brick_ori, brick_type, world_grid, top_objects);
        for (auto &o : top_objects)
        {
            if (o == brick_name)
            {
                o.clear();
            }
        }
        top_objects.erase(std::remove_if(top_objects.begin(), top_objects.end(), [](const std::string &s) { return s.empty(); }),
                          top_objects.end());
    }

    void getLegoTwistNext(int task_idx, const std::string &brick_name, std::vector<std::string> &side_bricks)
    {
        lego_ptr_->get_lego_twist_next(task_json_, task_idx, brick_name, side_bricks);
    }

    void getLegoAround(int task_idx, const std::string &brick_name, std::vector<std::string> &around_bricks)
    {
        around_bricks.clear();
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        const int brick_id = cur_graph_node["brick_id"].asInt();
        const int brick_ori = cur_graph_node["ori"].asInt();
        const int x = cur_graph_node["x"].asInt();
        const int y = cur_graph_node["y"].asInt();
        const int z = cur_graph_node["z"].asInt();

        std::vector<std::vector<std::vector<std::string>>> world_grid =
            lego_ptr_->gen_world_grid_from_graph(task_json_, task_idx, 48, 48, 48);

        lego_ptr_->get_lego_around(x, y, z, brick_ori, brick_id, brick_name, world_grid, around_bricks);
    }

    void getLegoSuppNearby(int task_idx, std::vector<std::string> &side_bricks)
    {
        side_bricks.clear();
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        if (!cur_graph_node.isMember("support_x"))
        {
            return;
        }
        const int support_x = cur_graph_node["support_x"].asInt();
        const int support_y = cur_graph_node["support_y"].asInt();
        const int support_z = cur_graph_node["support_z"].asInt();
        const int support_ori = cur_graph_node["support_ori"].asInt();
        if (support_x == -1)
        {
            return;
        }
        int sup_press_side, sup_brick_ori;
        lego_ptr_->get_sup_side_ori(support_ori, sup_press_side, sup_brick_ori);
        std::vector<std::vector<std::vector<std::string>>> world_grid =
            lego_ptr_->gen_world_grid_from_graph(task_json_, task_idx, 48, 48, 48);
        std::vector<std::string> above_bricks, side_low_bricks;
        lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid, side_bricks);
        lego_ptr_->get_lego_next(support_x, support_y, support_z - 1, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid, side_low_bricks);
        lego_ptr_->get_lego_above(support_x, support_y, support_z, sup_brick_ori, 9, world_grid, above_bricks);
        side_bricks.insert(side_bricks.end(), above_bricks.begin(), above_bricks.end());
        side_bricks.insert(side_bricks.end(), side_low_bricks.begin(), side_low_bricks.end());
    }

    bool segment_plan_rrt(int robot_id,
                          const std::vector<double> &goal_pose,
                          const ActPtr &act,
                          RobotTrajectory &solution)
    {
        (void)act;
        RRTConnect rrt_connect_planner(instance_, robot_id);
        PlannerOptions options;
        options.rrt_max_planning_time = planning_time_limit_;
        options.max_dist = 2.0;
        options.rrt_seed = (rrt_seed_ >= 0) ? (rrt_seed_ + robot_id) : rrt_seed_;

        instance_->setStartPose(robot_id, getRobotJointValues(robot_id));
        instance_->setGoalPose(robot_id, goal_pose);

        std::vector<RobotPose> static_obstacles;
        for (int other_id = 0; other_id < static_cast<int>(group_names_.size()); ++other_id)
        {
            if (other_id == robot_id)
            {
                continue;
            }
            std::vector<double> other_start = getRobotJointValues(other_id);
            if (other_start.empty())
            {
                continue;
            }
            RobotPose other_pose = instance_->initRobotPose(other_id);
            other_pose.joint_values = other_start;
            static_obstacles.push_back(other_pose);
        }
        if (!static_obstacles.empty())
        {
            rrt_connect_planner.setStaticObstacles(static_obstacles);
        }

        const bool success = rrt_connect_planner.plan(options);
        if (success)
        {
            RobotTrajectory solution_raw;
            rrt_connect_planner.getPlan(solution_raw);
            retimeSolution(instance_, solution_raw, solution, robot_id);
            return true;
        }
        return false;
    }

    bool fake_execute(const MRTrajectory &solution)
    {
        if (static_cast<int>(solution.size()) != instance_->getNumberOfRobots())
        {
            return false;
        }
        for (int rid = 0; rid < instance_->getNumberOfRobots(); ++rid)
        {
            if (solution[rid].trajectory.empty())
            {
                continue;
            }
            const RobotPose &pose = solution[rid].trajectory.back();
            instance_->moveRobot(rid, pose);
            instance_->updateScene();
            const int off = robot_offsets_[static_cast<std::size_t>(rid)];
            for (int j = 0; j < static_cast<int>(pose.joint_values.size()) && off + j < static_cast<int>(current_joints_.size()); ++j)
            {
                current_joints_[static_cast<std::size_t>(off + j)] = pose.joint_values[static_cast<std::size_t>(j)];
            }
        }
        return true;
    }

    bool planAndMove(const std::vector<RobotPose> &goal_poses,
                     const tpg::TPGConfig &tpg_config,
                     int plan_robot_id,
                     const std::vector<ActPtr> &acts,
                     MRTrajectory &solution,
                     double *planning_time_sec)
    {
        const int robot_count = instance_->getNumberOfRobots();
        auto t_start = std::chrono::high_resolution_clock::now();
        bool success = true;
        solution.clear();
        solution.reserve(robot_count);
        for (int rid = 0; rid < robot_count; ++rid)
        {
            RobotTrajectory traj;
            traj.robot_id = rid;
            if (rid != plan_robot_id)
            {
                traj.trajectory = {goal_poses[rid]};
                traj.times = {0.0};
                traj.cost = 0.0;
            }
            else
            {
                success = segment_plan_rrt(rid, goal_poses[rid].joint_values, acts[rid], traj);
            }
            solution.push_back(std::move(traj));
            if (!success)
            {
                break;
            }
        }
        const double t_plan =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t_start).count() *
            1e-9;
        if (planning_time_sec)
        {
            *planning_time_sec = t_plan;
        }
        planning_colcheck_ += instance_->numCollisionChecks();
        if (success)
        {
            success &= fake_execute(solution);
            tpg_colcheck_ += instance_->numCollisionChecks();
        }
        (void)tpg_config;
        return success;
    }

    MRTrajectory async_shortcut(const ShortcutOptions &options, const std::shared_ptr<ActivityGraph> &act_graph)
    {
        MRTrajectory sync_solution;
        concatSyncSolution(instance_, act_graph, plans_, options.dt, sync_solution);
        ShortcutterMT sc(instance_, act_graph, options);
        MRTrajectory asynced_sol;
        sc.shortcutSolution(sync_solution, asynced_sol);
        return asynced_sol;
    }

private:
    std::shared_ptr<PlanInstance> instance_;
    std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
    Json::Value task_json_;
    int num_tasks_{0};

    std::string output_dir_;
    std::vector<std::string> group_names_;
    std::vector<std::string> eof_links_;
    double planning_time_limit_{5.0};
    int rrt_seed_{1};
    std::string plan_name_;
    std::string environment_name_;
    double l1_vmax_{1.0};

    std::vector<int> robot_offsets_;
    std::vector<int> robot_dofs_;
    std::vector<double> current_joints_;

    std::vector<MRTrajectory> plans_;
    std::shared_ptr<tpg::ADG> adg_;

    double planning_time_{0.0};
    int planning_colcheck_{0};
    int tpg_colcheck_{0};
};

}  // namespace

int main(int argc, char **argv)
{
    try
    {
        const Args args = parse_args(argc, argv);
        setLogLevel(args.print_debug ? LogLevel::DEBUG : LogLevel::INFO);

        std::filesystem::create_directories(args.output_dir);

        const std::string steps_file = (std::filesystem::path(args.steps_dir) / (args.task + "_steps.csv")).string();
        const std::string task_file = (std::filesystem::path(args.steps_dir) / (args.task + "_seq.json")).string();
        if (!std::filesystem::exists(steps_file))
        {
            throw std::runtime_error("Missing steps file: " + steps_file);
        }
        if (!std::filesystem::exists(task_file))
        {
            throw std::runtime_error("Missing task file: " + task_file);
        }

        const std::vector<std::string> default_group_names = {"left_arm", "right_arm", "top_arm", "bottom_arm"};
        std::vector<std::string> group_names;
        group_names.reserve(static_cast<std::size_t>(args.num_robots));
        for (int i = 0; i < args.num_robots; ++i)
        {
            if (i < static_cast<int>(default_group_names.size()))
            {
                group_names.push_back(default_group_names[static_cast<std::size_t>(i)]);
            }
            else
            {
                group_names.push_back("arm_" + std::to_string(i));
            }
        }
        std::vector<std::string> eof_links;
        eof_links.reserve(group_names.size());
        for (const auto &g : group_names)
        {
            eof_links.push_back(g + "_link_tool");
        }

        auto env_opt = vamp_env::make_environment_config(args.vamp_environment);
        if (!env_opt)
        {
            throw std::runtime_error("Unsupported VAMP environment: " + args.vamp_environment);
        }
        auto instance = vamp_env::make_vamp_instance(*env_opt);
        if (!instance)
        {
            throw std::runtime_error("Failed to create VAMP instance");
        }

        instance->setNumberOfRobots(static_cast<int>(group_names.size()));
        instance->setRobotNames(group_names);
        instance->setVmax(args.vmax);
        instance->setRandomSeed(static_cast<unsigned int>(args.seed));
        for (int rid = 0; rid < static_cast<int>(group_names.size()); ++rid)
        {
            instance->setRobotDOF(rid, 7);
        }

        vamp_env::add_environment_obstacles(*env_opt, *instance);
        vamp_env::add_environment_attachments(*env_opt, *instance);
        if (auto defaults = vamp_env::default_base_transforms(*env_opt))
        {
            if (defaults->size() == group_names.size())
            {
                for (int i = 0; i < static_cast<int>(group_names.size()); ++i)
                {
                    instance->setRobotBaseTransform(i, (*defaults)[static_cast<std::size_t>(i)]);
                }
            }
        }

        std::vector<std::vector<GoalPose>> all_poses;
        read_steps_file(steps_file, all_poses, static_cast<std::size_t>(args.num_robots));

        tpg::TPGConfig tpg_config;
        tpg_config.dt = 0.05 / std::max(args.vmax, 1e-6);
        tpg_config.seed = args.seed;
        tpg_config.output_dir = args.output_dir;
        tpg_config.parallel = args.parallel;
        tpg_config.sync_task = false;
        tpg_config.use_sweep_type2 = true;
        tpg_config.shortcut_time = args.shortcut_time;

        ShortcutOptions sc_options;
        sc_options.t_limit = 0.1;
        sc_options.prioritized_shortcut = true;
        sc_options.dt = tpg_config.dt;
        sc_options.tight_shortcut = true;
        sc_options.seed = args.seed;

        ShortcutOptions async_sc_options;
        async_sc_options.t_limit = 0.0;
        async_sc_options.path_shortcut = true;
        async_sc_options.dt = tpg_config.dt;
        async_sc_options.seed = args.seed;

        LegoPlanner planner(instance, args.output_dir, group_names, eof_links, args.planning_time_limit, args.seed);
        planner.setCurrentJointsFromRow(all_poses.front());

        const std::string config_path = (std::filesystem::path(args.root_pwd) / args.config_fname).string();
        planner.setLegoFactory(config_path, args.root_pwd, task_file, args.task);
        planner.setEnvironmentName(env_opt->environment_name);
        planner.setL1Vmax(args.vmax);
        planner.initLegoPositions();

        if (!planner.sequential_plan(1, all_poses, sc_options, async_sc_options, tpg_config))
        {
            throw std::runtime_error("LEGO planning failed");
        }

        auto adg = planner.getAdg();
        if (!adg)
        {
            throw std::runtime_error("No ADG produced");
        }

        if (!adg->optimize(instance, tpg_config))
        {
            throw std::runtime_error("ADG optimize failed");
        }

        if (args.export_portable_graph)
        {
            const std::string pb_path = (std::filesystem::path(args.output_dir) / "adg.pb").string();
            std::string pb_err;
            if (!mr_planner::graph_proto::write_adg(*adg, pb_path, &pb_err))
            {
                log("Failed to write portable ADG protobuf to " + pb_path + ": " + pb_err, LogLevel::WARN);
            }
            else
            {
                log("Saved portable ADG protobuf to " + pb_path, LogLevel::INFO);
            }
        }

        if (args.meshcat)
        {
            instance->enableMeshcat(args.meshcat_host, static_cast<std::uint16_t>(args.meshcat_port));
            mr_planner::visualization::MeshcatPlaybackOptions viz;
            viz.real_time_rate = args.meshcat_rate;
            if (!planner.resetSceneForPlayback())
            {
                log("Failed to reset planning scene for Meshcat playback", LogLevel::WARN);
            }
            if (!mr_planner::visualization::play_schedule(*instance, *adg, viz))
            {
                log("Meshcat playback ended early (schedule execution failed or exceeded max ticks)", LogLevel::WARN);
            }
        }

        log("Planning completed successfully", LogLevel::INFO);
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[error] " << e.what() << "\n";
        return 1;
    }
}
