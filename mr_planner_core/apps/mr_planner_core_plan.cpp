#include <mr_planner/backends/vamp_env_factory.h>
#include <mr_planner/core/logger.h>
#include <mr_planner/execution/tpg.h>
#include <mr_planner/io/graph_proto.h>
#include <mr_planner/io/skillplan.h>
#include <mr_planner/planning/composite_rrt.h>
#include <mr_planner/planning/planner.h>
#include <mr_planner/planning/roadmap.h>
#include <mr_planner/planning/shortcutter.h>
#include <mr_planner/visualization/meshcat_playback.h>

#include <filesystem>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Args
{
    std::string vamp_environment{"dual_gp4"};
    std::string planner{"composite_rrt"};
    std::string output_dir{"./outputs"};
    double dt{0.1};
    double vmax{1.0};
    double planning_time{5.0};
    double shortcut_time{0.0};
    int seed{1};
    int num_robots{2};
    int dof{7};
    double max_goal_dist{0.5};
    int sample_attempts{200};
    int roadmap_samples{500};
    double roadmap_max_dist{2.0};
    std::string start;
    std::string goal;
    bool write_tpg{true};
    bool meshcat{false};
    std::string meshcat_host{"127.0.0.1"};
    int meshcat_port{7600};
    double meshcat_rate{1.0};
};

void usage(const char *prog)
{
    std::cerr << "Usage: " << prog
              << " --vamp-environment <name> [--start <q0;...> --goal <q0;...>] [options]\n"
              << "Options:\n"
              << "  --planner <composite_rrt|cbs_prm> Planner backend (default: composite_rrt)\n"
              << "  --output-dir <path>          Output directory (default: ./outputs)\n"
              << "  --dt <sec>                   Discretization timestep (default: 0.1)\n"
              << "  --vmax <val>                 Joint-space L1 vmax (default: 1.0)\n"
              << "  --planning-time <sec>        Planner time limit (default: 5.0)\n"
              << "  --shortcut-time <sec>        Shortcut runtime (default: 0, disabled)\n"
              << "  --seed <int>                 RNG seed (default: 1)\n"
              << "  --num-robots <int>           Robot count for random problems (default: 2)\n"
              << "  --dof <int>                  Robot DOF for random problems (default: 7)\n"
              << "  --max-goal-dist <rad>        Max random goal step distance (default: 0.5)\n"
              << "  --sample-attempts <int>      Attempts to sample start/goal (default: 200)\n"
              << "  --roadmap-samples <int>      Roadmap samples per robot for cbs_prm (default: 500)\n"
              << "  --roadmap-max-dist <val>     Roadmap connection radius for cbs_prm (default: 2.0)\n"
              << "  --no-tpg                     Do not build/export TPG\n"
              << "  --meshcat                    Visualize the resulting schedule via Meshcat\n"
              << "  --meshcat-host <host>        Meshcat bridge host (default: 127.0.0.1)\n"
              << "  --meshcat-port <port>        Meshcat bridge port (default: 7600)\n"
              << "  --meshcat-rate <x>           Playback rate (default: 1.0; 0 disables sleeping)\n"
              << "\n"
              << "Format for --start/--goal:\n"
              << "  Semicolon-separated robots, comma-separated joint values per robot.\n"
              << "  Example (2 robots, 7-DOF):\n"
              << "    --start \"0,0,0,0,0,0,0;0,0,0,0,0,0,0\" --goal \"0.2,0,0,0,0,0,0;0,0.2,0,0,0,0,0\"\n"
              << "\n"
              << "If --start/--goal are omitted, a collision-free random problem is generated.\n";
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

bool parse_int(const std::string &s, int *out)
{
    if (!out)
    {
        return false;
    }
    try
    {
        *out = std::stoi(s);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delim))
    {
        if (!token.empty() && token.front() == ' ')
        {
            token.erase(0, token.find_first_not_of(' '));
        }
        if (!token.empty() && token.back() == ' ')
        {
            token.erase(token.find_last_not_of(' ') + 1);
        }
        if (!token.empty())
        {
            out.push_back(token);
        }
    }
    return out;
}

bool parse_joint_matrix(const std::string &spec, std::vector<std::vector<double>> *out)
{
    if (!out)
    {
        return false;
    }
    out->clear();

    const auto robots = split(spec, ';');
    if (robots.empty())
    {
        return false;
    }
    out->reserve(robots.size());
    for (const auto &robot : robots)
    {
        const auto vals = split(robot, ',');
        if (vals.empty())
        {
            return false;
        }
        std::vector<double> joints;
        joints.reserve(vals.size());
        for (const auto &v : vals)
        {
            double x = 0.0;
            if (!parse_double(v, &x))
            {
                return false;
            }
            joints.push_back(x);
        }
        out->push_back(std::move(joints));
    }
    return true;
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
        else if (a == "--vamp-environment")
        {
            args.vamp_environment = require_value(a);
        }
        else if (a == "--planner")
        {
            args.planner = require_value(a);
        }
        else if (a == "--output-dir")
        {
            args.output_dir = require_value(a);
        }
        else if (a == "--start")
        {
            args.start = require_value(a);
        }
        else if (a == "--goal")
        {
            args.goal = require_value(a);
        }
        else if (a == "--dt")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.dt) || args.dt <= 0.0)
            {
                throw std::runtime_error("Invalid --dt: " + v);
            }
        }
        else if (a == "--vmax")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.vmax) || args.vmax <= 0.0)
            {
                throw std::runtime_error("Invalid --vmax: " + v);
            }
        }
        else if (a == "--planning-time")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.planning_time) || args.planning_time <= 0.0)
            {
                throw std::runtime_error("Invalid --planning-time: " + v);
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
        else if (a == "--seed")
        {
            const std::string v = require_value(a);
            if (!parse_int(v, &args.seed))
            {
                throw std::runtime_error("Invalid --seed: " + v);
            }
        }
        else if (a == "--num-robots")
        {
            args.num_robots = std::stoi(require_value(a));
        }
        else if (a == "--dof")
        {
            args.dof = std::stoi(require_value(a));
        }
        else if (a == "--max-goal-dist")
        {
            args.max_goal_dist = std::stod(require_value(a));
        }
        else if (a == "--sample-attempts")
        {
            args.sample_attempts = std::stoi(require_value(a));
        }
        else if (a == "--roadmap-samples")
        {
            args.roadmap_samples = std::stoi(require_value(a));
        }
        else if (a == "--roadmap-max-dist")
        {
            args.roadmap_max_dist = std::stod(require_value(a));
        }
        else if (a == "--no-tpg")
        {
            args.write_tpg = false;
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
            if (!parse_int(v, &args.meshcat_port) || args.meshcat_port <= 0 || args.meshcat_port > 65535)
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
    return args;
}

}  // namespace

namespace
{
std::string join_matrix(const std::vector<std::vector<double>> &m)
{
    std::ostringstream oss;
    oss << std::setprecision(17);
    for (std::size_t i = 0; i < m.size(); ++i)
    {
        if (i)
        {
            oss << ';';
        }
        for (std::size_t j = 0; j < m[i].size(); ++j)
        {
            if (j)
            {
                oss << ',';
            }
            oss << m[i][j];
        }
    }
    return oss.str();
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

        // Require that the straight-line interpolation is collision-free. This keeps the
        // problem easy enough for smoke tests and avoids flaky planner failures.
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
}  // namespace

int main(int argc, char **argv)
{
    try
    {
        const Args args = parse_args(argc, argv);
        std::vector<std::vector<double>> start, goal;
        int desired_num_robots = args.num_robots;

        if (!args.start.empty() || !args.goal.empty())
        {
            if (args.start.empty() || args.goal.empty())
            {
                throw std::runtime_error("Both --start and --goal must be provided (or omit both for random)");
            }
            if (!parse_joint_matrix(args.start, &start) || !parse_joint_matrix(args.goal, &goal))
            {
                throw std::runtime_error("Failed to parse --start/--goal joint matrices");
            }
            if (start.size() != goal.size())
            {
                throw std::runtime_error("--start and --goal robot counts differ");
            }
            desired_num_robots = static_cast<int>(start.size());
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
        instance->setVmax(args.vmax);
        instance->setRandomSeed(static_cast<unsigned int>(args.seed));

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
                                     args.dof,
                                     args.sample_attempts,
                                     args.max_goal_dist,
                                     args.seed,
                                     &start,
                                     &goal,
                                     &sample_err))
            {
                throw std::runtime_error(sample_err);
            }
            std::cerr << "[info] sampled --start \"" << join_matrix(start) << "\"\n";
            std::cerr << "[info] sampled --goal  \"" << join_matrix(goal) << "\"\n";
        }

        const int num_robots = static_cast<int>(start.size());
        if (num_robots <= 0)
        {
            throw std::runtime_error("No robots specified");
        }

        for (int i = 0; i < num_robots; ++i)
        {
            if (start[static_cast<std::size_t>(i)].size() != goal[static_cast<std::size_t>(i)].size())
            {
                throw std::runtime_error("Robot " + std::to_string(i) + " DOF mismatch between start and goal");
            }
            instance->setRobotDOF(i, start[static_cast<std::size_t>(i)].size());
            instance->setStartPose(i, start[static_cast<std::size_t>(i)]);
            instance->setGoalPose(i, goal[static_cast<std::size_t>(i)]);
        }

        std::filesystem::create_directories(args.output_dir);

        PlannerOptions options;
        options.max_planning_time = args.planning_time;
        options.rrt_max_planning_time = args.planning_time;
        options.max_planning_iterations = 10000;
        options.max_dist = 2.0;
        options.num_samples = args.roadmap_samples;
        options.rrt_seed = args.seed;

        MRTrajectory solution;
        double planner_time = 0.0;
        if (args.planner == "composite_rrt")
        {
            CompositeRRTCPlanner planner(instance);
            planner.setSeed(args.seed);
            if (!planner.plan(options))
            {
                log("Planning failed", LogLevel::ERROR);
                return 3;
            }
            planner_time = planner.getPlanTime();

            if (!planner.getPlan(solution))
            {
                log("Planner returned no solution", LogLevel::ERROR);
                return 3;
            }
        }
        else if (args.planner == "cbs_prm")
        {
            options.max_dist = args.roadmap_max_dist;

            std::vector<std::shared_ptr<RoadMap>> roadmaps;
            roadmaps.reserve(static_cast<std::size_t>(num_robots));
            for (int rid = 0; rid < num_robots; ++rid)
            {
                auto rm = std::make_shared<RoadMap>(instance, rid);
                rm->setNumSamples(args.roadmap_samples);
                rm->setMaxDist(args.roadmap_max_dist);
                rm->buildRoadmap();
                roadmaps.push_back(std::move(rm));
            }

            CBSPlanner planner(instance, roadmaps);
            if (!planner.plan(options))
            {
                log("CBS PRM planning failed", LogLevel::ERROR);
                return 3;
            }
            planner_time = planner.getPlanTime();
            if (!planner.getPlan(solution))
            {
                log("CBS PRM returned no solution", LogLevel::ERROR);
                return 3;
            }
        }
        else
        {
            throw std::runtime_error("Unknown --planner: " + args.planner);
        }

        MRTrajectory discrete;
        rediscretizeSolution(instance, solution, discrete, args.dt);

        const std::string raw_csv = (std::filesystem::path(args.output_dir) / "solution_raw.csv").string();
        saveSolution(instance, discrete, raw_csv);

        MRTrajectory final_solution = discrete;
        if (args.shortcut_time > 0.0)
        {
            ShortcutOptions sc;
            sc.t_limit = args.shortcut_time;
            sc.dt = args.dt;
            sc.seed = args.seed;
            // default to thompson shortcutting.
            sc.thompson_selector = true;
            sc.progress_file = (std::filesystem::path(args.output_dir) / "shortcut_progress.csv").string();
            Shortcutter shortcutter(instance, sc);
            MRTrajectory shortcut;
            if (shortcutter.shortcutSolution(final_solution, shortcut))
            {
                final_solution = std::move(shortcut);
            }
        }

        const std::string out_csv = (std::filesystem::path(args.output_dir) / "solution.csv").string();
        saveSolution(instance, final_solution, out_csv);

        {
            std::vector<mr_planner::skillplan::RobotSpec> robots;
            robots.reserve(static_cast<std::size_t>(num_robots));
            for (int i = 0; i < num_robots; ++i)
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
            opts.plan_name = "mr_planner_core_plan";
            opts.environment_name = env_opt->environment_name;
            opts.backend_type = instance->instanceType();
            opts.l1_vmax = args.vmax;

            const auto json = mr_planner::skillplan::make_simple_plan(robots, final_solution, opts);
            std::string err;
            const std::string out_path = (std::filesystem::path(args.output_dir) / "skillplan.json").string();
            if (!mr_planner::skillplan::write_json_to_file(json, out_path, &err))
            {
                log("Failed to write skillplan.json: " + err, LogLevel::ERROR);
            }
        }

        std::unique_ptr<tpg::TPG> tpg_ptr;
        if (args.write_tpg || args.meshcat)
        {
            tpg::TPGConfig cfg;
            cfg.dt = args.dt;
            cfg.output_dir = args.output_dir;
            cfg.shortcut = false;
            cfg.random_shortcut = false;
            cfg.parallel = true;

            tpg_ptr = std::make_unique<tpg::TPG>();
            if (tpg_ptr->init(instance, final_solution, cfg))
            {
                if (args.write_tpg)
                {
                    const std::string pb_path = (std::filesystem::path(args.output_dir) / "tpg.pb").string();
                    std::string pb_err;
                    if (!mr_planner::graph_proto::write_tpg(*tpg_ptr, pb_path, &pb_err))
                    {
                        log("Failed to write tpg.pb: " + pb_err, LogLevel::WARN);
                    }
                }
            }
            else
            {
                tpg_ptr.reset();
            }
        }

        if (args.meshcat)
        {
            instance->enableMeshcat(args.meshcat_host, static_cast<std::uint16_t>(args.meshcat_port));
            if (tpg_ptr)
            {
                mr_planner::visualization::MeshcatPlaybackOptions viz;
                viz.real_time_rate = args.meshcat_rate;
                if (!mr_planner::visualization::play_schedule(*instance, *tpg_ptr, viz))
                {
                    log("Meshcat playback ended early (schedule execution failed or exceeded max ticks)", LogLevel::WARN);
                }
            }
        }

        log("Planning completed successfully (planner_time=" + std::to_string(planner_time) + "s)", LogLevel::INFO);
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[error] " << e.what() << "\n";
        return 1;
    }
}
