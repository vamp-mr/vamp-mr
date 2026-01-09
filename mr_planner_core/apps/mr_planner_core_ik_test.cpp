#include <mr_planner/backends/vamp_env_factory.h>
#include <mr_planner/core/instance.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Args
{
    std::string vamp_environment{"panda_two"};
    int dof{7};
    int seed{1};
    int samples{10};
    int ik_restarts{5};
    int ik_iters{80};
    double tol_pos{0.025};
    double tol_ang_deg{15.0};
    double seed_noise{0.2};
};

void usage(const char *prog)
{
    std::cerr << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --vamp-environment <name|path>  VAMP environment (default: panda_two)\n"
              << "  --dof <int>                     Robot DOF (default: 7)\n"
              << "  --seed <int>                    RNG seed (default: 1)\n"
              << "  --samples <int>                 IK samples per robot (default: 10)\n"
              << "  --ik-restarts <int>             IK restarts (default: 5)\n"
              << "  --ik-iters <int>                IK iterations per restart (default: 80)\n"
              << "  --tol-pos <m>                   Position tolerance (default: 0.025)\n"
              << "  --tol-ang-deg <deg>             Angle tolerance in degrees (default: 15)\n"
              << "  --seed-noise <rad>              Uniform noise applied to seed joints (default: 0.2)\n"
              << "  -h, --help                      Show this help\n";
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
        else if (a == "--vamp-environment")
        {
            args.vamp_environment = require_value(a);
        }
        else if (a == "--dof")
        {
            const std::string v = require_value(a);
            if (!parse_int(v, &args.dof) || args.dof <= 0)
            {
                throw std::runtime_error("Invalid --dof: " + v);
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
        else if (a == "--samples")
        {
            const std::string v = require_value(a);
            if (!parse_int(v, &args.samples) || args.samples <= 0)
            {
                throw std::runtime_error("Invalid --samples: " + v);
            }
        }
        else if (a == "--ik-restarts")
        {
            const std::string v = require_value(a);
            if (!parse_int(v, &args.ik_restarts) || args.ik_restarts <= 0)
            {
                throw std::runtime_error("Invalid --ik-restarts: " + v);
            }
        }
        else if (a == "--ik-iters")
        {
            const std::string v = require_value(a);
            if (!parse_int(v, &args.ik_iters) || args.ik_iters <= 0)
            {
                throw std::runtime_error("Invalid --ik-iters: " + v);
            }
        }
        else if (a == "--tol-pos")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.tol_pos) || args.tol_pos <= 0.0)
            {
                throw std::runtime_error("Invalid --tol-pos: " + v);
            }
        }
        else if (a == "--tol-ang-deg")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.tol_ang_deg) || args.tol_ang_deg <= 0.0)
            {
                throw std::runtime_error("Invalid --tol-ang-deg: " + v);
            }
        }
        else if (a == "--seed-noise")
        {
            const std::string v = require_value(a);
            if (!parse_double(v, &args.seed_noise) || args.seed_noise < 0.0)
            {
                throw std::runtime_error("Invalid --seed-noise: " + v);
            }
        }
        else
        {
            throw std::runtime_error("Unknown argument: " + a);
        }
    }
    return args;
}

Eigen::Vector3d so3_log(const Eigen::Matrix3d &R)
{
    const Eigen::AngleAxisd aa(R);
    const double angle = aa.angle();
    if (std::abs(angle) < 1e-12)
    {
        return Eigen::Vector3d::Zero();
    }
    return aa.axis() * angle;
}

bool sample_collision_free_joint_matrix(PlanInstance &instance,
                                        int num_robots,
                                        int attempts,
                                        std::vector<std::vector<double>> *q_out)
{
    if (!q_out)
    {
        return false;
    }
    q_out->clear();
    q_out->resize(static_cast<std::size_t>(num_robots));

    for (int attempt = 0; attempt < attempts; ++attempt)
    {
        std::vector<RobotPose> poses;
        poses.reserve(static_cast<std::size_t>(num_robots));
        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotPose pose = instance.initRobotPose(rid);
            if (!instance.sample(pose))
            {
                return false;
            }
            poses.push_back(std::move(pose));
        }
        if (!instance.checkCollision(poses, /*self=*/false))
        {
            for (int rid = 0; rid < num_robots; ++rid)
            {
                (*q_out)[static_cast<std::size_t>(rid)] = poses[static_cast<std::size_t>(rid)].joint_values;
            }
            return true;
        }
    }
    return false;
}
}  // namespace

int main(int argc, char **argv)
{
    try
    {
        const Args args = parse_args(argc, argv);

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

        const int num_robots = static_cast<int>(env_opt->robot_groups.size());
        instance->setNumberOfRobots(num_robots);
        instance->setRobotNames(env_opt->robot_groups);
        if (!env_opt->hand_groups.empty())
        {
            instance->setHandNames(env_opt->hand_groups);
        }
        for (int rid = 0; rid < num_robots; ++rid)
        {
            instance->setRobotDOF(rid, static_cast<std::size_t>(args.dof));
        }
        instance->setRandomSeed(static_cast<unsigned int>(args.seed));

        vamp_env::add_environment_obstacles(*env_opt, *instance);
        vamp_env::add_environment_attachments(*env_opt, *instance);

        if (auto defaults = vamp_env::default_base_transforms(*env_opt))
        {
            if (defaults->size() == static_cast<std::size_t>(num_robots))
            {
                for (int rid = 0; rid < num_robots; ++rid)
                {
                    instance->setRobotBaseTransform(rid, (*defaults)[static_cast<std::size_t>(rid)]);
                }
            }
        }
        instance->updateScene();

        std::vector<std::vector<double>> q_fixed;
        if (!sample_collision_free_joint_matrix(*instance, num_robots, /*attempts=*/5000, &q_fixed))
        {
            throw std::runtime_error("Failed to sample a collision-free joint matrix");
        }

        std::mt19937 rng(static_cast<std::uint32_t>(args.seed));
        std::uniform_real_distribution<double> noise_dist(-args.seed_noise, args.seed_noise);

        constexpr double kPi = 3.14159265358979323846;
        const double tol_ang = args.tol_ang_deg * kPi / 180.0;

        std::vector<double> ik_times_sec;
        ik_times_sec.reserve(static_cast<std::size_t>(num_robots * args.samples));

        for (int rid = 0; rid < num_robots; ++rid)
        {
            RobotPose true_pose = instance->initRobotPose(rid);
            true_pose.joint_values = q_fixed[static_cast<std::size_t>(rid)];
            const Eigen::Isometry3d target = instance->getEndEffectorTransformFromPose(true_pose);

            for (int sample_idx = 0; sample_idx < args.samples; ++sample_idx)
            {
                std::vector<double> seed = true_pose.joint_values;
                for (std::size_t j = 0; j < seed.size(); ++j)
                {
                    seed[j] += noise_dist(rng);
                }

                PlanInstance::InverseKinematicsOptions ik;
                ik.seed = std::move(seed);
                ik.fixed_joints = q_fixed;
                ik.max_restarts = args.ik_restarts;
                ik.max_iters = args.ik_iters;
                ik.tol_pos = args.tol_pos;
                ik.tol_ang = tol_ang;
                ik.self_only = false;

                const auto t0 = std::chrono::steady_clock::now();
                const auto q_sol_opt = instance->inverseKinematics(rid, target, ik);
                const auto t1 = std::chrono::steady_clock::now();
                ik_times_sec.push_back(std::chrono::duration<double>(t1 - t0).count());
                if (!q_sol_opt)
                {
                    throw std::runtime_error("IK failed for robot_id=" + std::to_string(rid));
                }

                RobotPose sol_pose = instance->initRobotPose(rid);
                sol_pose.joint_values = *q_sol_opt;
                const Eigen::Isometry3d cur = instance->getEndEffectorTransformFromPose(sol_pose);

                const Eigen::Vector3d p_err = target.translation() - cur.translation();
                const Eigen::Matrix3d R_err = target.linear() * cur.linear().transpose();
                const Eigen::Vector3d w_err = so3_log(R_err);
                const double ang_err = std::abs(Eigen::AngleAxisd(R_err).angle());

                if (p_err.norm() > args.tol_pos || ang_err > tol_ang)
                {
                    throw std::runtime_error(
                        "IK solution outside tolerance for robot_id=" + std::to_string(rid) +
                        " (pos_err=" + std::to_string(p_err.norm()) +
                        ", ang_err_deg=" + std::to_string(ang_err * 180.0 / kPi) + ")");
                }

                std::vector<RobotPose> poses;
                poses.reserve(static_cast<std::size_t>(num_robots));
                for (int other = 0; other < num_robots; ++other)
                {
                    RobotPose pose = instance->initRobotPose(other);
                    pose.joint_values = (other == rid) ? *q_sol_opt : q_fixed[static_cast<std::size_t>(other)];
                    poses.push_back(std::move(pose));
                }
                if (instance->checkCollision(poses, /*self=*/false))
                {
                    throw std::runtime_error("IK returned a colliding configuration for robot_id=" +
                                             std::to_string(rid));
                }
            }
        }

        if (!ik_times_sec.empty())
        {
            const double total_sec = std::accumulate(ik_times_sec.begin(), ik_times_sec.end(), 0.0);
            std::vector<double> sorted = ik_times_sec;
            std::sort(sorted.begin(), sorted.end());
            const auto at = [&](double p) -> double {
                if (sorted.empty())
                {
                    return 0.0;
                }
                const double clamped = std::min(1.0, std::max(0.0, p));
                const std::size_t idx = static_cast<std::size_t>(clamped * static_cast<double>(sorted.size() - 1));
                return sorted[idx];
            };
            const double mean_ms = 1e3 * total_sec / static_cast<double>(ik_times_sec.size());
            const double p50_ms = 1e3 * at(0.50);
            const double p95_ms = 1e3 * at(0.95);
            const double max_ms = 1e3 * sorted.back();
            std::cerr << "[info] IK timing: calls=" << ik_times_sec.size() << " total_sec=" << total_sec
                      << " mean_ms=" << mean_ms << " p50_ms=" << p50_ms << " p95_ms=" << p95_ms
                      << " max_ms=" << max_ms << "\n";
        }
        std::cerr << "[info] IK smoke passed for " << num_robots << " robots (" << args.samples << " samples/robot)\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[error] " << e.what() << "\n";
        return 1;
    }
}
