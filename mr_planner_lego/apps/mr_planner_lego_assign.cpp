#include <mr_planner/applications/lego/lego/Lego.hpp>
#include <mr_planner/applications/lego/lego_config_utils.h>
#include <mr_planner/applications/lego/lego_primitive.h>
#include <mr_planner/backends/vamp_env_factory.h>
#include <mr_planner/core/logger.h>

#include <cstdint>
#include <jsoncpp/json/json.h>

#include <Eigen/Geometry>

#include <algorithm>
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
#include <cctype>
#include <cstdlib>

namespace
{
std::string shell_escape(const std::string &value)
{
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('\'');
    for (const char c : value)
    {
        if (c == '\'')
        {
            escaped += "'\"'\"'";
        }
        else
        {
            escaped.push_back(c);
        }
    }
    escaped.push_back('\'');
    return escaped;
}

struct Args
{
    std::string task{"test"};
    std::string root_pwd{"."};
    std::string config_fname{"config/lego_tasks/user_config.json"};
    std::string task_fname{""};
    std::string output_dir{"config/lego_tasks/steps"};
    int num_robots{2};
    std::string vamp_environment{"dual_gp4"};
    bool optimize_poses{false};
    bool print_debug{false};
    bool use_gen3_camera{true};
    bool optimize_brickseq{true};
    bool meshcat{false};
    std::string meshcat_host{"127.0.0.1"};
    int meshcat_port{7600};
};

std::vector<std::string> find_above_bricks(const std::shared_ptr<lego_manipulation::lego::Lego> &lego_ptr,
                                          const std::string &brick_name,
                                          bool recursive)
{
    if (!lego_ptr)
    {
        return {};
    }
    std::vector<std::string> above = lego_ptr->get_brick_above(brick_name);
    if (!recursive)
    {
        return above;
    }
    for (std::size_t i = 0; i < above.size(); ++i)
    {
        const auto extra = find_above_bricks(lego_ptr, above[i], true);
        above.insert(above.end(), extra.begin(), extra.end());
    }
    return above;
}

Object make_start_object(const std::shared_ptr<lego_manipulation::lego::Lego> &lego_ptr, const std::string &name)
{
    if (!lego_ptr)
    {
        return Object();
    }

    Object obj;
    obj.name = name;
    obj.state = Object::State::Static;
    obj.parent_link = "world";
    obj.shape = Object::Shape::Box;

    lego_manipulation::Pose box_pose;
    if (name == "table")
    {
        lego_ptr->get_table_size(obj.length, obj.width, obj.height);
        box_pose = lego_ptr->get_table_pose();
    }
    else
    {
        lego_ptr->get_brick_sizes(name, obj.length, obj.width, obj.height);
        box_pose = lego_ptr->get_init_brick_pose(name);
    }

    obj.x = box_pose.position.x();
    obj.y = box_pose.position.y();
    obj.z = box_pose.position.z() - obj.height / 2.0;
    obj.qx = box_pose.orientation.x();
    obj.qy = box_pose.orientation.y();
    obj.qz = box_pose.orientation.z();
    obj.qw = box_pose.orientation.w();

    return obj;
}

Object make_target_object(const std::shared_ptr<lego_manipulation::lego::Lego> &lego_ptr,
                          const Json::Value &task_json,
                          int task_idx)
{
    if (!lego_ptr || !task_json.isMember(std::to_string(task_idx)))
    {
        return Object();
    }

    const auto cur_graph_node = task_json[std::to_string(task_idx)];
    const int brick_id = cur_graph_node.get("brick_id", 0).asInt();
    const auto brick_names = lego_ptr->get_brick_names_by_type(brick_id);
    if (brick_names.empty())
    {
        throw std::runtime_error("No bricks found for task " + std::to_string(task_idx));
    }

    Eigen::Matrix4d brick_pose_mtx;
    lego_ptr->calc_bric_asssemble_pose(brick_names.front(),
                                       cur_graph_node["x"].asInt(),
                                       cur_graph_node["y"].asInt(),
                                       cur_graph_node["z"].asInt(),
                                       cur_graph_node["ori"].asInt(),
                                       brick_pose_mtx);

    Object obj;
    obj.name = "b" + std::to_string(brick_id) + "_t" + std::to_string(task_idx);
    obj.state = Object::State::Static;
    obj.parent_link = "world";
    obj.shape = Object::Shape::Box;
    lego_ptr->get_brick_sizes(brick_names.front(), obj.length, obj.width, obj.height);

    obj.x = brick_pose_mtx(0, 3);
    obj.y = brick_pose_mtx(1, 3);
    obj.z = brick_pose_mtx(2, 3) - obj.height / 2.0;
    Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
    obj.qx = quat.x();
    obj.qy = quat.y();
    obj.qz = quat.z();
    obj.qw = quat.w();
    return obj;
}

double calculate_cost_l1(const lego_manipulation::math::VectorJd &q1,
                         const lego_manipulation::math::VectorJd &q2)
{
    double cost = 0.0;
    for (int i = 0; i < 6; ++i)
    {
        cost += std::abs(q1(i, 0) - q2(i, 0));
    }
    return cost;
}

void run_task_assignment_python(const std::filesystem::path &repo_root,
                                const std::filesystem::path &output_dir,
                                const std::filesystem::path &task_config_path,
                                const std::string &output_fname)
{
    auto script = repo_root / "mr_planner_lego/scripts/task_assignment.py";
    if (!std::filesystem::exists(script))
    {
        script = repo_root / "scripts/lego/task_assignment.py";
    }
    if (!std::filesystem::exists(script))
    {
        throw std::runtime_error("Missing task assignment script: " + script.string());
    }

    std::ostringstream cmd;
    cmd << "python3 " << shell_escape(script.string()) << " --output-dir " << shell_escape(output_dir.string())
        << " --task-config-path " << shell_escape(task_config_path.string()) << " --output-fname "
        << shell_escape(output_fname);
    const int rc = std::system(cmd.str().c_str());
    if (rc != 0)
    {
        throw std::runtime_error("task_assignment.py failed with exit code " + std::to_string(rc));
    }
}

void usage(const char *prog)
{
    std::cerr << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --task <name>               Task name (default: test)\n"
              << "  --root <path>               Repo root (default: .)\n"
              << "  --config <path>             user_config.json (default: config/lego_tasks/user_config.json)\n"
              << "  --task-file <path>          assembly_tasks/<task>.json (default: derived from --task)\n"
              << "  --output-dir <path>         Output steps directory (default: config/lego_tasks/steps)\n"
              << "  --num-robots <int>          Robot count (default: 2)\n"
              << "  --vamp-environment <name>   VAMP environment (default: dual_gp4)\n"
              << "  --meshcat                    Visualize environment in Meshcat\n"
              << "  --meshcat-host <host>        Meshcat bridge host (default: 127.0.0.1)\n"
              << "  --meshcat-port <port>        Meshcat bridge port (default: 7600)\n"
              << "  --optimize-poses            Enable pose optimization search\n"
              << "  --no-optimize-brickseq      Keep brick_seq as provided\n"
              << "  --no-gen3-camera            Disable Gen3 camera offsets\n"
              << "  --print-debug               Enable debug logging\n";
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
        else if (a == "--task-file")
        {
            args.task_fname = require_value(a);
        }
        else if (a == "--output-dir")
        {
            args.output_dir = require_value(a);
        }
        else if (a == "--num-robots")
        {
            args.num_robots = std::stoi(require_value(a));
        }
        else if (a == "--vamp-environment")
        {
            args.vamp_environment = require_value(a);
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
        else if (a == "--optimize-poses")
        {
            args.optimize_poses = true;
        }
        else if (a == "--no-optimize-brickseq")
        {
            args.optimize_brickseq = false;
        }
        else if (a == "--no-gen3-camera")
        {
            args.use_gen3_camera = false;
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
    return args;
}

bool read_json(const std::string &path, Json::Value *out, std::string *error = nullptr)
{
    if (!out)
    {
        return false;
    }
    std::ifstream ifs(path, std::ifstream::binary);
    if (!ifs.is_open())
    {
        if (error)
        {
            *error = "Failed to open " + path;
        }
        return false;
    }
    try
    {
        ifs >> *out;
        return true;
    }
    catch (const std::exception &e)
    {
        if (error)
        {
            *error = std::string("Failed to parse ") + path + ": " + e.what();
        }
        return false;
    }
}

bool write_json(const Json::Value &json, const std::string &path, std::string *error = nullptr)
{
    std::ofstream ofs(path, std::ofstream::binary);
    if (!ofs.is_open())
    {
        if (error)
        {
            *error = "Failed to open for write " + path;
        }
        return false;
    }
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "    ";
    ofs << Json::writeString(builder, json);
    return true;
}

void write_robot_manifest(const std::string &output_dir,
                          int num_tasks,
                          int num_bricks,
                          int num_grasps,
                          int num_offsets,
                          const std::string &steps_file,
                          const std::string &task_file,
                          const std::vector<std::string> &group_names,
                          const std::vector<std::string> &eof_links,
                          int dof,
                          const std::vector<std::string> &cost_files,
                          const std::vector<std::string> &support_files)
{
    Json::Value manifest;
    manifest["num_tasks"] = num_tasks;
    manifest["num_bricks"] = num_bricks;
    manifest["num_grasps"] = num_grasps;
    manifest["num_offsets"] = num_offsets;
    manifest["steps_file"] = steps_file;
    manifest["task_file"] = task_file;
    Json::Value robots(Json::arrayValue);
    for (int rid = 0; rid < static_cast<int>(group_names.size()); ++rid)
    {
        Json::Value entry;
        entry["id"] = rid;
        entry["group_name"] = group_names[rid];
        entry["eof_link"] = (rid < static_cast<int>(eof_links.size())) ? eof_links[rid] : std::string();
        entry["dof"] = dof;
        if (rid < static_cast<int>(cost_files.size()))
        {
            entry["cost_matrix"] = cost_files[static_cast<std::size_t>(rid)];
        }
        if (rid < static_cast<int>(support_files.size()))
        {
            entry["support_matrix"] = support_files[static_cast<std::size_t>(rid)];
        }
        robots.append(entry);
    }
    manifest["robots"] = robots;

    std::string err;
    if (!write_json(manifest, (std::filesystem::path(output_dir) / "robots_manifest.json").string(), &err))
    {
        log("Failed to write robots_manifest.json: " + err, LogLevel::WARN);
    }
}

void write_record_row(std::vector<std::vector<double>> &records,
                      const std::shared_ptr<lego_manipulation::lego::Lego> &lego_ptr,
                      const std::vector<int> &robot_dofs,
                      const std::vector<int> &use_flags,
                      const std::vector<lego_manipulation::math::VectorJd> &robot_goals)
{
    const int robot_count = static_cast<int>(robot_dofs.size());
    if (!lego_ptr || robot_count == 0 || static_cast<int>(robot_goals.size()) < robot_count)
    {
        return;
    }

    std::vector<double> row;
    row.reserve(robot_count * 16);
    for (int rid = 0; rid < robot_count; ++rid)
    {
        const int use_flag = (rid < static_cast<int>(use_flags.size())) ? use_flags[rid] : 0;
        row.push_back(static_cast<double>(use_flag));
        Eigen::MatrixXd T = lego_manipulation::math::FK(robot_goals[rid],
                                                        lego_ptr->robot_DH(rid),
                                                        lego_ptr->robot_base(rid),
                                                        false);
        Eigen::Quaterniond quat(T.block<3, 3>(0, 0));
        row.push_back(T(0, 3));
        row.push_back(T(1, 3));
        row.push_back(T(2, 3));
        row.push_back(quat.x());
        row.push_back(quat.y());
        row.push_back(quat.z());
        row.push_back(quat.w());

        const int dof = robot_dofs[rid];
        for (int j = 0; j < dof; ++j)
        {
            double val = (robot_goals[rid].rows() > j) ? robot_goals[rid](j, 0) : 0.0;
            row.push_back(val);
        }
    }

    records.push_back(std::move(row));
}

void write_pose_file(const std::string &path, const std::vector<int> &robot_dofs, const std::vector<std::vector<double>> &records)
{
    const int robot_count = static_cast<int>(robot_dofs.size());
    if (robot_count == 0)
    {
        return;
    }

    std::vector<std::string> headers;
    for (int rid = 0; rid < robot_count; ++rid)
    {
        headers.push_back("r" + std::to_string(rid) + "_use");
        headers.push_back("r" + std::to_string(rid) + "_px");
        headers.push_back("r" + std::to_string(rid) + "_py");
        headers.push_back("r" + std::to_string(rid) + "_pz");
        headers.push_back("r" + std::to_string(rid) + "_qx");
        headers.push_back("r" + std::to_string(rid) + "_qy");
        headers.push_back("r" + std::to_string(rid) + "_qz");
        headers.push_back("r" + std::to_string(rid) + "_qw");
        for (int j = 0; j < robot_dofs[rid]; ++j)
        {
            headers.push_back("r" + std::to_string(rid) + "_joint_" + std::to_string(j));
        }
    }

    std::ofstream ofs(path);
    for (std::size_t i = 0; i < headers.size(); ++i)
    {
        if (i > 0)
        {
            ofs << ",";
        }
        ofs << headers[i];
    }
    ofs << "\n";

    const std::size_t expected_cols = headers.size();
    for (const auto &row : records)
    {
        for (std::size_t i = 0; i < expected_cols; ++i)
        {
            if (i > 0)
            {
                ofs << ",";
            }
            ofs << ((i < row.size()) ? row[i] : 0.0);
        }
        ofs << "\n";
    }
}

}  // namespace

int main(int argc, char **argv)
{
    try
    {
        const Args args = parse_args(argc, argv);
        setLogLevel(args.print_debug ? LogLevel::DEBUG : LogLevel::INFO);

        std::filesystem::create_directories(args.output_dir);

        std::string task_file = args.task_fname;
        if (task_file.empty())
        {
            task_file = (std::filesystem::path(args.root_pwd) / "config/lego_tasks/assembly_tasks" /
                         (args.task + ".json"))
                            .string();
        }

        Json::Value config;
        std::string err;
        if (!read_json((std::filesystem::path(args.root_pwd) / args.config_fname).string(), &config, &err))
        {
            throw std::runtime_error(err);
        }

        Json::Value task_json;
        if (!read_json(task_file, &task_json, &err))
        {
            throw std::runtime_error(err);
        }

        const int num_tasks = static_cast<int>(task_json.size());
        if (num_tasks <= 0)
        {
            throw std::runtime_error("Task file is empty: " + task_file);
        }

        // Match ROS assignment behavior:
        // - Ensure manipulate_type exists.
        // - If optimizing brick sequencing, clear brick_seq so the MILP can choose.
        for (int tid = 1; tid <= num_tasks; ++tid)
        {
            auto &node = task_json[std::to_string(tid)];
            if (!node.isObject())
            {
                continue;
            }
            if (!node.isMember("manipulate_type"))
            {
                node["manipulate_type"] = 0;
            }
            if (args.optimize_brickseq && node.isMember("brick_seq"))
            {
                node.removeMember("brick_seq");
            }
        }

        const auto robot_calibrations = lego_manipulation::lego::loadRobotCalibrations(config, args.root_pwd);

        const std::string plate_calibration_fname =
            lego_manipulation::lego::resolveRepoPath(args.root_pwd, config["plate_calibration_fname"].asString());
        const std::string env_setup_folder =
            lego_manipulation::lego::resolveRepoPath(args.root_pwd, config["env_setup_folder"].asString());
        const std::string env_setup_fname =
            (std::filesystem::path(env_setup_folder) / ("env_setup_" + args.task + ".json")).string();
        const std::string lego_lib_fname =
            lego_manipulation::lego::resolveRepoPath(args.root_pwd, config["lego_lib_fname"].asString());
        const std::string world_base_fname =
            lego_manipulation::lego::resolveRepoPath(args.root_pwd, config["world_base_fname"].asString());
        const bool assemble = config["Start_with_Assemble"].asBool();

        auto lego_ptr = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr->setup(env_setup_fname,
                        lego_lib_fname,
                        plate_calibration_fname,
                        assemble,
                        task_json,
                        world_base_fname,
                        robot_calibrations);

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

        // Add table and bricks as collision objects.
        auto add_object = [&](const std::string &name) { instance->addMoveableObject(make_start_object(lego_ptr, name)); };

        add_object("table");
        for (const auto &name : lego_ptr->get_brick_names())
        {
            add_object(name);
        }

        if (args.meshcat)
        {
            instance->enableMeshcat(args.meshcat_host, static_cast<std::uint16_t>(args.meshcat_port));
        }
        instance->updateScene();

        const double twist_rad = config.get("Twist_Deg", 0).asInt() * M_PI / 180.0;
        const double handover_twist_rad = config.get("Handover_Twist_Deg", 0).asInt() * M_PI / 180.0;
        const double sup_down_offset = config.get("Sup_Down_Offset", 0.0).asDouble();

        LegoPrimitive prim(lego_ptr,
                           instance,
                           eof_links,
                           args.optimize_poses,
                           false,
                           args.print_debug,
                           twist_rad,
                           handover_twist_rad,
                           args.use_gen3_camera,
                           args.output_dir);
        prim.setSupDownOffset(sup_down_offset);

        const int robot_count = static_cast<int>(group_names.size());
        if (robot_count < 2)
        {
            throw std::runtime_error("mr_planner_lego_assign requires at least two robots (got " +
                                     std::to_string(robot_count) + ")");
        }

        const auto brick_names = lego_ptr->get_active_bricks_names();
        const int num_bricks = static_cast<int>(brick_names.size());
        if (num_bricks <= 0)
        {
            throw std::runtime_error("No active bricks found in environment");
        }

        const int num_offsets = prim.getNumOffsets();
        const int num_sides = 4;
        const int num_grasps = num_sides * num_offsets;
        const double kLargeCost = 1000000.0;

        // Cost matrices for MILP solver.
        std::vector<vec3d<double>> cost_matrices(
            robot_count,
            vec3d<double>(num_tasks, vec2d<double>(num_bricks, vec<double>(num_grasps, kLargeCost))));
        vec3d<double> support_matrices(robot_count, vec2d<double>(num_tasks, vec<double>(num_grasps, kLargeCost)));
        vec2d<int> delta_matrix(num_tasks, vec<int>(num_bricks, 0));
        vec2d<int> precedence;
        vec2d<int> sup_required(num_tasks, vec<int>(robot_count, 0));

        vec3d<bool> lego_reachable(robot_count, vec2d<bool>(num_bricks, vec<bool>(num_grasps, false)));
        std::vector<vec2d<vecgoal>> robot_lego_goals(
            robot_count, vec2d<vecgoal>(num_bricks, vec<vecgoal>(num_grasps)));
        vec3d<double> cost_pick(robot_count, vec2d<double>(num_bricks, vec<double>(num_grasps, kLargeCost)));

        std::vector<vec2d<vecgoal>> robot_task_goals(
            robot_count, vec2d<vecgoal>(num_tasks, vec<vecgoal>(num_grasps)));
        std::vector<vec2d<vecgoal>> robot_support_goals(
            robot_count, vec2d<vecgoal>(num_tasks, vec<vecgoal>(num_grasps)));
        vec2d<vecgoal> robot_handover_goals(robot_count, vec<vecgoal>(robot_count));

        const lego_manipulation::math::VectorJd home_qjd =
            static_cast<lego_manipulation::math::VectorJd>(prim.getHomeQ());
        const lego_manipulation::math::VectorJd idle_qjd =
            static_cast<lego_manipulation::math::VectorJd>(prim.getIdleQ());
        const lego_manipulation::math::VectorJd receive_qjd =
            static_cast<lego_manipulation::math::VectorJd>(prim.getReceiveQ());
        const lego_manipulation::math::VectorJd home_receive_qjd =
            static_cast<lego_manipulation::math::VectorJd>(prim.getHomeReceiveQ());
        const lego_manipulation::math::VectorJd home_handover_qjd =
            static_cast<lego_manipulation::math::VectorJd>(prim.getHomeHandoverQ());
        const lego_manipulation::math::VectorJd &default_standby_qjd = (robot_count > 2) ? idle_qjd : home_qjd;

        std::vector<double> cost_receive_per_robot(robot_count, kLargeCost);
        vec2d<double> cost_handover_per_robot(robot_count, vec<double>(robot_count, kLargeCost));
        for (int rid = 0; rid < robot_count; ++rid)
        {
            for (int receive_rid = 0; receive_rid < robot_count; ++receive_rid)
            {
                if (rid == receive_rid)
                {
                    continue;
                }
                const bool handover_feasible =
                    prim.calculateHandoverPoses(rid, receive_rid, robot_handover_goals[rid][receive_rid]);
                if (handover_feasible && !robot_handover_goals[rid][receive_rid].empty())
                {
                    cost_handover_per_robot[rid][receive_rid] =
                        calculate_cost_l1(home_qjd, robot_handover_goals[rid][receive_rid][0]);
                }
            }
            cost_receive_per_robot[rid] = calculate_cost_l1(home_qjd, receive_qjd);
        }

        // Compute pick IK reachability for every brick and grasp.
        for (int brick_idx = 0; brick_idx < num_bricks; ++brick_idx)
        {
            const std::string &brick_name = brick_names[static_cast<std::size_t>(brick_idx)];
            for (int g = 0; g < num_grasps; ++g)
            {
                const int press_side = g / num_offsets + 1;
                const int press_offset = g % num_offsets;
                if (!prim.isPressPtInBound(brick_name, press_side, press_offset))
                {
                    continue;
                }

                std::vector<bool> reachables;
                std::vector<vecgoal> robot_goals;
                prim.calculatePickGoalsMulti(brick_name, press_side, press_offset, reachables, robot_goals);
                for (int rid = 0; rid < robot_count; ++rid)
                {
                    const bool reachable = (rid < static_cast<int>(reachables.size())) ? reachables[rid] : false;
                    lego_reachable[rid][brick_idx][g] = reachable;
                    if (reachable && rid < static_cast<int>(robot_goals.size()))
                    {
                        robot_lego_goals[rid][brick_idx][g] = robot_goals[rid];
                    }
                    else
                    {
                        robot_lego_goals[rid][brick_idx][g].clear();
                    }
                }
            }

            // find any blocks on top, add precedence
            const auto above_bricks = find_above_bricks(lego_ptr, brick_name, false);
            for (const auto &above_brick : above_bricks)
            {
                const int above_idx =
                    static_cast<int>(std::find(brick_names.begin(), brick_names.end(), above_brick) - brick_names.begin());
                if (above_idx >= 0 && above_idx < num_bricks)
                {
                    precedence.push_back({above_idx, brick_idx});
                }
            }
        }

        // Cost of picking each brick grasp (L1 in joint space from home to pick pose).
        for (int brick_idx = 0; brick_idx < num_bricks; ++brick_idx)
        {
            for (int g = 0; g < num_grasps; ++g)
            {
                for (int rid = 0; rid < robot_count; ++rid)
                {
                    const auto &goals = robot_lego_goals[rid][brick_idx][g];
                    if (lego_reachable[rid][brick_idx][g] && goals.size() > 2)
                    {
                        cost_pick[rid][brick_idx][g] = calculate_cost_l1(home_qjd, goals[2]);
                    }
                }
            }
        }

        vec2d<int> robot_press_poses(robot_count);
        vec2d<vecgoal> robot_press_goals(robot_count);
        std::vector<bool> robot_stable(robot_count, false);
        vec3d<bool> support_goal_cached(robot_count, vec2d<bool>(num_tasks, vec<bool>(num_grasps, false)));

        // Calculate the cost matrix for each task.
        for (int i = 0; i < num_tasks; ++i)
        {
            const int task_idx = i + 1;
            const auto cur_graph_node = task_json[std::to_string(task_idx)];
            const int brick_id = cur_graph_node.get("brick_id", 0).asInt();
            const int manip_type = cur_graph_node.get("manipulate_type", 0).asInt();
            const int brick_seq_provided = cur_graph_node.isMember("brick_seq") ? cur_graph_node["brick_seq"].asInt() : -1;

            auto world_grid = lego_ptr->gen_world_grid_from_graph(task_json, i, 48, 48, 48);
            prim.setWorldGrid(world_grid);

            const Object obj = make_target_object(lego_ptr, task_json, task_idx);
            instance->addMoveableObject(obj);
            instance->updateScene();
            for (int rid = 0; rid < robot_count; ++rid)
            {
                prim.allowToolObjectCollision(obj.name, rid, true);
            }

            for (int rid = 0; rid < robot_count; ++rid)
            {
                robot_press_poses[rid].clear();
                robot_press_goals[rid].clear();
                robot_stable[rid] = prim.findBestPlacePoses(task_idx,
                                                           rid,
                                                           brick_names,
                                                           cur_graph_node,
                                                           manip_type,
                                                           robot_press_poses[rid],
                                                           robot_press_goals[rid]);
            }

            for (int rid = 0; rid < robot_count; ++rid)
            {
                prim.allowToolObjectCollision(obj.name, rid, false);
            }

            bool any_press_pose = false;
            for (int rid = 0; rid < robot_count; ++rid)
            {
                if (!robot_press_poses[rid].empty())
                {
                    any_press_pose = true;
                    break;
                }
            }
            if (!any_press_pose)
            {
                log("No press sides found for task " + std::to_string(task_idx) + " with any robot.", LogLevel::ERROR);
                continue;
            }

            std::vector<int> candidate_bricks;
            const std::string req_type = "b" + std::to_string(brick_id) + "_";
            for (int t = 0; t < num_bricks; ++t)
            {
                const std::string &brick_name = brick_names[static_cast<std::size_t>(t)];
                if (brick_name.find(req_type) != 0)
                {
                    continue;
                }

                int brick_seq = -1;
                const std::size_t pos = brick_name.find('_');
                if (pos != std::string::npos && pos + 1 < brick_name.size())
                {
                    const std::string seq_str = brick_name.substr(pos + 1);
                    if (!seq_str.empty() && std::isdigit(static_cast<unsigned char>(seq_str[0])))
                    {
                        brick_seq = std::stoi(seq_str);
                    }
                }

                if (!args.optimize_brickseq && brick_seq_provided != -1 && brick_seq != brick_seq_provided)
                {
                    continue;
                }
                delta_matrix[i][t] = 1;
                candidate_bricks.push_back(t);
            }
            if (candidate_bricks.empty())
            {
                log("No candidate bricks found in library for task " + std::to_string(task_idx), LogLevel::WARN);
                continue;
            }

            bool task_feasible = false;
            for (int primary = 0; primary < robot_count; ++primary)
            {
                const auto &press_list = robot_press_poses[primary];
                const auto &goal_list = robot_press_goals[primary];
                for (std::size_t press_idx = 0; press_idx < press_list.size(); ++press_idx)
                {
                    const int g = press_list[press_idx];
                    if (g < 0 || g >= num_grasps || press_idx >= goal_list.size())
                    {
                        continue;
                    }
                    const int press_side = g / num_offsets + 1;
                    const int press_offset = g % num_offsets;

                    robot_task_goals[primary][i][g] = goal_list[press_idx];
                    if (goal_list[press_idx].size() <= 2)
                    {
                        continue;
                    }
                    const double place_cost = calculate_cost_l1(home_qjd, goal_list[press_idx][2]);

                    if (manip_type == 0)
                    {
                        for (const int brick_idx : candidate_bricks)
                        {
                            if (lego_reachable[primary][brick_idx][g])
                            {
                                const double total = cost_pick[primary][brick_idx][g] + place_cost;
                                cost_matrices[primary][i][brick_idx][g] =
                                    std::min(cost_matrices[primary][i][brick_idx][g], total);
                                if (robot_stable[primary])
                                {
                                    task_feasible = true;
                                }
                            }
                        }
                        if (!robot_stable[primary])
                        {
                            sup_required[i][primary] = 1;
                        }
                    }

                    if (manip_type == 1)
                    {
                        support_matrices[primary][i][g] =
                            std::min(support_matrices[primary][i][g], cost_receive_per_robot[primary] + place_cost);
                        for (int picker = 0; picker < robot_count; ++picker)
                        {
                            if (picker == primary)
                            {
                                continue;
                            }
                            bool picker_feasible = false;
                            for (const int brick_idx : candidate_bricks)
                            {
                                if (!lego_reachable[picker][brick_idx][g])
                                {
                                    continue;
                                }
                                if (!support_goal_cached[picker][i][g])
                                {
                                    Eigen::MatrixXd support_T = Eigen::MatrixXd::Identity(4, 4);
                                    Eigen::MatrixXd support_pre_T = Eigen::MatrixXd::Identity(4, 4);
                                    support_goal_cached[picker][i][g] = prim.findStableSupportPose(
                                        press_side,
                                        press_offset,
                                        cur_graph_node,
                                        picker,
                                        task_idx,
                                        support_T,
                                        support_pre_T,
                                        robot_support_goals[picker][i][g]);
                                }
                                if (!support_goal_cached[picker][i][g] || robot_support_goals[picker][i][g].empty())
                                {
                                    continue;
                                }
                                const double support_cost = calculate_cost_l1(home_qjd, robot_support_goals[picker][i][g][0]);
                                const double total = cost_pick[picker][brick_idx][g] +
                                                     cost_handover_per_robot[picker][primary] + support_cost;
                                cost_matrices[picker][i][brick_idx][g] =
                                    std::min(cost_matrices[picker][i][brick_idx][g], total);
                                task_feasible = true;
                                picker_feasible = true;
                            }
                            if (picker_feasible)
                            {
                                sup_required[i][picker] = 1;
                            }
                        }
                    }
                    else if (!robot_stable[primary])
                    {
                        for (int support = 0; support < robot_count; ++support)
                        {
                            if (support == primary)
                            {
                                continue;
                            }
                            if (!support_goal_cached[support][i][g])
                            {
                                Eigen::MatrixXd support_T = Eigen::MatrixXd::Identity(4, 4);
                                Eigen::MatrixXd support_pre_T = Eigen::MatrixXd::Identity(4, 4);
                                support_goal_cached[support][i][g] = prim.findStableSupportPose(
                                    press_side,
                                    press_offset,
                                    cur_graph_node,
                                    support,
                                    task_idx,
                                    support_T,
                                    support_pre_T,
                                    robot_support_goals[support][i][g]);
                            }
                            if (!support_goal_cached[support][i][g] || robot_support_goals[support][i][g].empty())
                            {
                                continue;
                            }
                            const double support_cost =
                                calculate_cost_l1(home_qjd, robot_support_goals[support][i][g][0]);
                            support_matrices[support][i][g] =
                                std::min(support_matrices[support][i][g], support_cost);
                            task_feasible = true;
                        }
                    }
                    else
                    {
                        task_feasible = true;
                    }
                }
            }

            double min_cost = kLargeCost;
            for (int rid = 0; rid < robot_count; ++rid)
            {
                for (const int brick_idx : candidate_bricks)
                {
                    for (int g = 0; g < num_grasps; ++g)
                    {
                        min_cost = std::min(min_cost, cost_matrices[rid][i][brick_idx][g]);
                    }
                }
            }
            if (min_cost >= kLargeCost && !task_feasible)
            {
                log("No feasible solution found for task " + std::to_string(task_idx), LogLevel::ERROR);
            }
        }

        // Write matrices for python MILP solver.
        std::vector<std::string> cost_matrix_files;
        std::vector<std::string> support_matrix_files;
        cost_matrix_files.reserve(static_cast<std::size_t>(robot_count));
        support_matrix_files.reserve(static_cast<std::size_t>(robot_count));
        for (int rid = 0; rid < robot_count; ++rid)
        {
            const std::string cost_filename = "cost_matrix_r" + std::to_string(rid) + ".csv";
            cost_matrix_files.push_back(cost_filename);
            std::ofstream cost_file((std::filesystem::path(args.output_dir) / cost_filename).string());
            for (int j = 0; j < num_bricks; ++j)
            {
                for (int g = 0; g < num_grasps; ++g)
                {
                    cost_file << brick_names[static_cast<std::size_t>(j)] << "@g_" << g << ",";
                }
            }
            cost_file << "\n";
            for (int i = 0; i < num_tasks; ++i)
            {
                for (int j = 0; j < num_bricks; ++j)
                {
                    for (int g = 0; g < num_grasps; ++g)
                    {
                        cost_file << cost_matrices[rid][i][j][g] << ",";
                    }
                }
                cost_file << "\n";
            }

            const std::string support_filename = "support_matrix_r" + std::to_string(rid) + ".csv";
            support_matrix_files.push_back(support_filename);
            std::ofstream support_file((std::filesystem::path(args.output_dir) / support_filename).string());
            for (int g = 0; g < num_grasps; ++g)
            {
                support_file << "g_" << g << ",";
            }
            support_file << "\n";
            for (int j = 0; j < num_tasks; ++j)
            {
                for (int g = 0; g < num_grasps; ++g)
                {
                    support_file << support_matrices[rid][j][g] << ",";
                }
                support_file << "\n";
            }
        }

        {
            std::ofstream delta_file((std::filesystem::path(args.output_dir) / "delta_matrix.csv").string());
            for (int j = 0; j < num_bricks; ++j)
            {
                delta_file << brick_names[static_cast<std::size_t>(j)] << ",";
            }
            delta_file << "\n";
            for (int i = 0; i < num_tasks; ++i)
            {
                for (int j = 0; j < num_bricks; ++j)
                {
                    delta_file << delta_matrix[i][j] << ",";
                }
                delta_file << "\n";
            }
        }

        {
            std::ofstream support_req_file((std::filesystem::path(args.output_dir) / "support_req.csv").string());
            for (int rid = 0; rid < robot_count; ++rid)
            {
                support_req_file << "r" << rid << ",";
            }
            support_req_file << "\n";
            for (int i = 0; i < num_tasks; ++i)
            {
                for (int rid = 0; rid < robot_count; ++rid)
                {
                    support_req_file << sup_required[i][rid] << ",";
                }
                support_req_file << "\n";
            }
        }

        write_robot_manifest(args.output_dir,
                             num_tasks,
                             num_bricks,
                             num_grasps,
                             num_offsets,
                             args.task + "_steps.csv",
                             args.task + "_seq.json",
                             group_names,
                             eof_links,
                             7,
                             cost_matrix_files,
                             support_matrix_files);

        {
            std::ofstream precedence_file((std::filesystem::path(args.output_dir) / "precedence.csv").string());
            for (const auto &edge : precedence)
            {
                if (edge.size() >= 2)
                {
                    precedence_file << edge[0] << "," << edge[1] << ",\n";
                }
            }
        }

        // Solve the MILP assignment using the existing python implementation.
        const std::filesystem::path repo_root = std::filesystem::path(args.root_pwd);
        const std::filesystem::path out_dir = std::filesystem::path(args.output_dir);
        const std::filesystem::path task_config_path = std::filesystem::path(task_file);
        const std::string output_fname = args.task + "_seq.json";
        run_task_assignment_python(repo_root, out_dir, task_config_path, output_fname);

        const std::filesystem::path seq_out_path = out_dir / output_fname;
        Json::Value assigned_task_json;
        if (!read_json(seq_out_path.string(), &assigned_task_json, &err))
        {
            throw std::runtime_error(err);
        }

        const std::vector<int> robot_dofs(static_cast<std::size_t>(robot_count), 7);
        std::vector<std::vector<double>> record_rows;
        record_rows.reserve(static_cast<std::size_t>(1 + num_tasks * 32));

        std::vector<int> use_flags(static_cast<std::size_t>(robot_count), 0);
        std::vector<lego_manipulation::math::VectorJd> robot_goal_buffer(static_cast<std::size_t>(robot_count));

        auto resetRowState = [&](const lego_manipulation::math::VectorJd &default_goal) {
            std::fill(use_flags.begin(), use_flags.end(), 0);
            for (auto &goal : robot_goal_buffer)
            {
                goal = default_goal;
            }
        };

        auto assignGoal = [&](int rid, const lego_manipulation::math::VectorJd &goal) {
            if (rid < 0 || rid >= robot_count)
            {
                return;
            }
            use_flags[static_cast<std::size_t>(rid)] = 1;
            robot_goal_buffer[static_cast<std::size_t>(rid)] = goal;
        };

        auto assignGoalFromList = [&](int rid, const vecgoal *goals, std::size_t idx) {
            if (rid < 0 || rid >= robot_count || goals == nullptr)
            {
                return;
            }
            if (idx >= goals->size())
            {
                return;
            }
            assignGoal(rid, (*goals)[idx]);
        };

        auto fetchGoalList = [&](const std::vector<vec2d<vecgoal>> &storage,
                                 int rid,
                                 int first_index,
                                 int grasp_index) -> const vecgoal * {
            if (rid < 0 || rid >= static_cast<int>(storage.size()))
            {
                return nullptr;
            }
            const auto &level1 = storage[static_cast<std::size_t>(rid)];
            if (first_index < 0 || first_index >= static_cast<int>(level1.size()))
            {
                return nullptr;
            }
            const auto &grasp_lists = level1[static_cast<std::size_t>(first_index)];
            if (grasp_index < 0 || grasp_index >= static_cast<int>(grasp_lists.size()))
            {
                return nullptr;
            }
            return &grasp_lists[static_cast<std::size_t>(grasp_index)];
        };

        auto fetchHandoverGoals = [&](int rid, int receive_rid) -> const vecgoal * {
            if (rid < 0 || rid >= static_cast<int>(robot_handover_goals.size()))
            {
                return nullptr;
            }
            if (receive_rid < 0 || receive_rid >= static_cast<int>(robot_handover_goals[rid].size()))
            {
                return nullptr;
            }
            if (robot_handover_goals[rid][receive_rid].empty())
            {
                return nullptr;
            }
            return &robot_handover_goals[rid][receive_rid];
        };

        for (int j = 0; j < num_tasks; ++j)
        {
            const auto cur_graph_node = assigned_task_json[std::to_string(j + 1)];
            const int brick_id = cur_graph_node.get("brick_id", 0).asInt();
            std::string brick_seq = cur_graph_node["brick_seq"].asString();
            std::string name = "b" + std::to_string(brick_id) + "_" + brick_seq;
            if (name.find('.') != std::string::npos)
            {
                name = name.substr(0, name.find('.'));
            }

            const int id = static_cast<int>(std::find(brick_names.begin(), brick_names.end(), name) - brick_names.begin());
            if (id < 0 || id >= num_bricks)
            {
                log("Brick " + name + " not found for task " + std::to_string(j + 1), LogLevel::ERROR);
                continue;
            }

            const int robot_id = cur_graph_node.get("robot_id", 0).asInt();
            const int press_side = cur_graph_node.get("press_side", 1).asInt();
            const int press_offset = cur_graph_node.get("press_offset", 0).asInt();
            const int g = (press_side - 1) * num_offsets + press_offset;
            const int sup_robot_id = cur_graph_node.get("sup_robot_id", 0).asInt();
            const int manip_type = cur_graph_node.get("manipulate_type", 0).asInt();

            if (j == 0)
            {
                std::fill(use_flags.begin(), use_flags.end(), 1);
                for (auto &goal : robot_goal_buffer)
                {
                    goal = default_standby_qjd;
                }
                write_record_row(record_rows, lego_ptr, robot_dofs, use_flags, robot_goal_buffer);
            }

            int primary = robot_id - 1;
            if (primary < 0 || primary >= robot_count)
            {
                log("Primary robot index out of range for task " + std::to_string(j + 1), LogLevel::ERROR);
                continue;
            }

            int support = sup_robot_id - 1;
            if (sup_robot_id <= 0 || support == primary || support >= robot_count)
            {
                support = -1;
            }

            if (manip_type == 1)
            {
                std::swap(primary, support);
                if (primary < 0 || primary >= robot_count || support < 0 || support >= robot_count || primary == support)
                {
                    log("Invalid flipped robot mapping for manip_type==1 in task " + std::to_string(j + 1), LogLevel::ERROR);
                    continue;
                }
            }
            else if (manip_type != 0)
            {
                throw std::runtime_error("Unsupported manipulate_type " + std::to_string(manip_type));
            }

            const int pick_robot = (manip_type == 0) ? primary : support;
            const vecgoal *pick_goal_list = fetchGoalList(robot_lego_goals, pick_robot, id, g);
            const vecgoal *place_goal_list = fetchGoalList(robot_task_goals, primary, j, g);
            const vecgoal *support_goal_list =
                (support >= 0) ? fetchGoalList(robot_support_goals, support, j, g) : nullptr;

            if (manip_type == 0)
            {
                for (int mode = 1; mode <= 16; ++mode)
                {
                    resetRowState(default_standby_qjd);
                    switch (mode)
                    {
                    case 1:
                        assignGoalFromList(primary, pick_goal_list, 0);
                        break;
                    case 2:
                        assignGoalFromList(primary, pick_goal_list, 1);
                        break;
                    case 3:
                        assignGoalFromList(primary, pick_goal_list, 2);
                        break;
                    case 4:
                        assignGoalFromList(primary, pick_goal_list, 3);
                        break;
                    case 5:
                        assignGoalFromList(primary, pick_goal_list, 4);
                        break;
                    case 6:
                    case 16:
                        assignGoal(primary, default_standby_qjd);
                        if (support >= 0)
                        {
                            assignGoal(support, default_standby_qjd);
                        }
                        break;
                    case 7:
                        assignGoal(primary, default_standby_qjd);
                        assignGoalFromList(support, support_goal_list, 0);
                        break;
                    case 8:
                        assignGoal(primary, default_standby_qjd);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 9:
                        assignGoalFromList(primary, place_goal_list, 0);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 10:
                        assignGoalFromList(primary, place_goal_list, 1);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 11:
                        assignGoalFromList(primary, place_goal_list, 2);
                        assignGoalFromList(support, support_goal_list, 0);
                        break;
                    case 12:
                        assignGoalFromList(primary, place_goal_list, 3);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 13:
                        assignGoalFromList(primary, place_goal_list, 4);
                        assignGoalFromList(support, support_goal_list, 0);
                        break;
                    case 14:
                        assignGoal(primary, default_standby_qjd);
                        assignGoalFromList(support, support_goal_list, 0);
                        break;
                    case 15:
                        assignGoal(primary, default_standby_qjd);
                        assignGoalFromList(support, support_goal_list, 0);
                        break;
                    default:
                        break;
                    }
                    write_record_row(record_rows, lego_ptr, robot_dofs, use_flags, robot_goal_buffer);
                }
            }
            else
            {
                const vecgoal *support_handover = fetchHandoverGoals(support, primary);
                auto activatePrimary = [&](const lego_manipulation::math::VectorJd &goal) { assignGoal(primary, goal); };

                for (int mode = 17; mode <= 43; ++mode)
                {
                    resetRowState(default_standby_qjd);
                    switch (mode)
                    {
                    case 17:
                        activatePrimary(default_standby_qjd);
                        assignGoalFromList(support, pick_goal_list, 0);
                        break;
                    case 18:
                        activatePrimary(default_standby_qjd);
                        assignGoalFromList(support, pick_goal_list, 1);
                        break;
                    case 19:
                        activatePrimary(default_standby_qjd);
                        assignGoalFromList(support, pick_goal_list, 2);
                        break;
                    case 20:
                        activatePrimary(default_standby_qjd);
                        assignGoalFromList(support, pick_goal_list, 3);
                        break;
                    case 21:
                        activatePrimary(default_standby_qjd);
                        assignGoalFromList(support, pick_goal_list, 4);
                        break;
                    case 22:
                        activatePrimary(default_standby_qjd);
                        assignGoal(support, default_standby_qjd);
                        break;
                    case 23:
                        activatePrimary(home_receive_qjd);
                        assignGoal(support, default_standby_qjd);
                        break;
                    case 24:
                        activatePrimary(home_receive_qjd);
                        assignGoal(support, home_handover_qjd);
                        break;
                    case 25:
                        activatePrimary(receive_qjd);
                        assignGoal(support, home_handover_qjd);
                        break;
                    case 26:
                        activatePrimary(receive_qjd);
                        assignGoalFromList(support, support_handover, 0);
                        break;
                    case 27:
                        activatePrimary(receive_qjd);
                        assignGoalFromList(support, support_handover, 1);
                        break;
                    case 28:
                        activatePrimary(receive_qjd);
                        assignGoalFromList(support, support_handover, 2);
                        break;
                    case 29:
                        activatePrimary(receive_qjd);
                        assignGoalFromList(support, support_handover, 3);
                        break;
                    case 30:
                        activatePrimary(receive_qjd);
                        assignGoal(support, home_handover_qjd);
                        break;
                    case 31:
                        activatePrimary(receive_qjd);
                        assignGoal(support, default_standby_qjd);
                        break;
                    case 32:
                        activatePrimary(home_receive_qjd);
                        assignGoal(support, default_standby_qjd);
                        break;
                    case 33:
                        assignGoalFromList(primary, place_goal_list, 0);
                        assignGoal(support, default_standby_qjd);
                        break;
                    case 34:
                        assignGoalFromList(primary, place_goal_list, 1);
                        assignGoal(support, default_standby_qjd);
                        break;
                    case 35:
                        assignGoalFromList(primary, place_goal_list, 1);
                        assignGoalFromList(support, support_goal_list, 0);
                        break;
                    case 36:
                        assignGoalFromList(primary, place_goal_list, 1);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 37:
                        assignGoalFromList(primary, place_goal_list, 2);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 38:
                        assignGoalFromList(primary, place_goal_list, 3);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 39:
                        assignGoalFromList(primary, place_goal_list, 4);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 40:
                        assignGoalFromList(primary, place_goal_list, 5);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 41:
                        activatePrimary(default_standby_qjd);
                        assignGoalFromList(support, support_goal_list, 1);
                        break;
                    case 42:
                        activatePrimary(default_standby_qjd);
                        assignGoalFromList(support, support_goal_list, 0);
                        break;
                    case 43:
                        activatePrimary(default_standby_qjd);
                        assignGoal(support, default_standby_qjd);
                        break;
                    default:
                        break;
                    }
                    write_record_row(record_rows, lego_ptr, robot_dofs, use_flags, robot_goal_buffer);
                }
            }
        }

        const std::filesystem::path steps_out_path = out_dir / (args.task + "_steps.csv");
        write_pose_file(steps_out_path.string(), robot_dofs, record_rows);

        log("Task Assignment is feasible", LogLevel::INFO);
        log("Wrote " + seq_out_path.string(), LogLevel::INFO);
        log("Wrote " + steps_out_path.string(), LogLevel::INFO);
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[error] " << e.what() << "\n";
        return 1;
    }
}
