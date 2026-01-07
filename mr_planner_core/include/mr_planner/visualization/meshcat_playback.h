#pragma once

#include <mr_planner/core/instance.h>
#include <mr_planner/execution/tpg.h>

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

namespace mr_planner::visualization
{

struct MeshcatPlaybackOptions
{
    // Playback speed multiplier. For rate > 1, playback skips trajectory samples instead of
    // increasing the Meshcat publish frequency (which can overwhelm the Python bridge).
    // rate=1.0 plays in (approx.) real-time, rate=2.0 plays 2x faster, rate=0 disables sleeping.
    double real_time_rate{1.0};

    // Sleep duration when no robot can make progress (WAIT).
    double wait_sleep_sec{0.03};

    // Cap the number of visualization ticks to avoid infinite loops on malformed graphs.
    int max_ticks{500000};
};

inline bool play_schedule(PlanInstance &instance, tpg::TPG &graph, const MeshcatPlaybackOptions &options)
{
    const int num_robots = instance.getNumberOfRobots();
    if (num_robots <= 0)
    {
        return false;
    }

    const auto cfg = graph.getConfig();
    const double dt = (cfg.dt > 0.0) ? cfg.dt : 0.1;

    std::vector<RobotPose> last_pose(static_cast<std::size_t>(num_robots));
    for (int rid = 0; rid < num_robots; ++rid)
    {
        if (auto node = graph.getStartNode(rid))
        {
            last_pose[static_cast<std::size_t>(rid)] = node->pose;
        }
        else
        {
            last_pose[static_cast<std::size_t>(rid)] = instance.getStartPose(rid);
        }
        instance.moveRobot(rid, last_pose[static_cast<std::size_t>(rid)]);
        graph.update_joint_states(last_pose[static_cast<std::size_t>(rid)].joint_values, rid);
    }
    instance.updateScene();

    std::vector<RobotTrajectory> segments(static_cast<std::size_t>(num_robots));
    std::vector<std::size_t> seg_idx(static_cast<std::size_t>(num_robots), 0);
    std::vector<bool> done(static_cast<std::size_t>(num_robots), false);

    auto sleep_sec = [&](double sec) {
        if (sec <= 0.0)
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(sec));
    };

    const double rate = options.real_time_rate;
    double step_budget = 0.0;

    for (int tick = 0; tick < options.max_ticks; ++tick)
    {
        bool all_done = true;
        for (int rid = 0; rid < num_robots; ++rid)
        {
            if (!done[static_cast<std::size_t>(rid)])
            {
                all_done = false;
                break;
            }
        }
        if (all_done)
        {
            return true;
        }

        // For rate > 1, advance multiple trajectory samples per visualization tick so we don't
        // flood the Meshcat bridge with updates.
        int steps_this_tick = 1;
        if (rate > 1.0)
        {
            step_budget += rate;
            steps_this_tick = static_cast<int>(step_budget);
            step_budget -= static_cast<double>(steps_this_tick);
            steps_this_tick = std::max(1, steps_this_tick);
        }

        bool progressed_any = false;
        bool early_wait = false;
        for (int step = 0; step < steps_this_tick; ++step)
        {
            // Refresh segments as needed.
            for (int rid = 0; rid < num_robots; ++rid)
            {
                if (done[static_cast<std::size_t>(rid)])
                {
                    continue;
                }
                auto &seg = segments[static_cast<std::size_t>(rid)];
                auto &idx = seg_idx[static_cast<std::size_t>(rid)];
                if (idx >= seg.trajectory.size())
                {
                    seg.trajectory.clear();
                    seg.times.clear();
                    seg.act_ids.clear();
                    seg.cost = 0.0;
                    idx = 0;
                }
                if (!seg.trajectory.empty())
                {
                    continue;
                }

                RobotTrajectory next;
                const auto status = graph.schedule(rid, next);
                if (status == tpg::ScheduleStatus::DONE)
                {
                    done[static_cast<std::size_t>(rid)] = true;
                    continue;
                }
                if (status == tpg::ScheduleStatus::WAIT)
                {
                    continue;
                }
                if (status != tpg::ScheduleStatus::TRAJECTORY)
                {
                    return false;
                }
                seg = std::move(next);
                idx = 0;
            }

            bool progressed = false;
            for (int rid = 0; rid < num_robots; ++rid)
            {
                if (done[static_cast<std::size_t>(rid)])
                {
                    continue;
                }
                auto &seg = segments[static_cast<std::size_t>(rid)];
                auto &idx = seg_idx[static_cast<std::size_t>(rid)];
                if (idx >= seg.trajectory.size())
                {
                    continue;
                }
                const auto &pose = seg.trajectory[idx];
                last_pose[static_cast<std::size_t>(rid)] = pose;
                graph.update_joint_states(pose.joint_values, rid);
                ++idx;
                progressed = true;
            }

            progressed_any = progressed_any || progressed;
            if (!progressed)
            {
                early_wait = true;
                break;
            }
        }

        if (early_wait && !progressed_any)
        {
            // Don't accumulate a "catch-up" backlog when the schedule is blocked.
            step_budget = 0.0;
        }

        if (progressed_any)
        {
            for (int rid = 0; rid < num_robots; ++rid)
            {
                instance.moveRobot(rid, last_pose[static_cast<std::size_t>(rid)]);
            }
            instance.updateScene();

            if (rate <= 0.0)
            {
                // rate == 0 disables sleeping
            }
            else if (rate < 1.0)
            {
                sleep_sec(dt / rate);
            }
            else
            {
                sleep_sec(dt);
            }
        }
        else
        {
            sleep_sec(options.wait_sleep_sec);
        }
    }

    return false;
}

inline bool play_synchronized_trajectory(PlanInstance &instance,
                                         const MRTrajectory &trajectory,
                                         double dt,
                                         const MeshcatPlaybackOptions &options)
{
    const int num_robots = instance.getNumberOfRobots();
    if (num_robots <= 0)
    {
        return false;
    }
    if (trajectory.size() != static_cast<std::size_t>(num_robots))
    {
        return false;
    }

    const double resolved_dt = (dt > 0.0) ? dt : 0.1;

    std::vector<RobotPose> last_pose(static_cast<std::size_t>(num_robots));
    std::size_t max_steps = 0;
    for (int rid = 0; rid < num_robots; ++rid)
    {
        const auto &rt = trajectory[static_cast<std::size_t>(rid)];
        max_steps = std::max(max_steps, rt.trajectory.size());
        if (!rt.trajectory.empty())
        {
            last_pose[static_cast<std::size_t>(rid)] = rt.trajectory.front();
        }
        else
        {
            last_pose[static_cast<std::size_t>(rid)] = instance.getStartPose(rid);
        }
        instance.moveRobot(rid, last_pose[static_cast<std::size_t>(rid)]);
    }
    instance.updateScene();

    auto sleep_sec = [&](double sec) {
        if (sec <= 0.0)
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(sec));
    };

    const double rate = options.real_time_rate;
    double step_budget = 0.0;

    std::size_t step = 0;
    for (int tick = 0; tick < options.max_ticks && step < max_steps; ++tick)
    {
        for (int rid = 0; rid < num_robots; ++rid)
        {
            const auto &rt = trajectory[static_cast<std::size_t>(rid)];
            if (step < rt.trajectory.size())
            {
                last_pose[static_cast<std::size_t>(rid)] = rt.trajectory[step];
            }
            instance.moveRobot(rid, last_pose[static_cast<std::size_t>(rid)]);
        }
        instance.updateScene();
        if (rate <= 0.0)
        {
            // rate == 0 disables sleeping
        }
        else if (rate < 1.0)
        {
            sleep_sec(resolved_dt / rate);
        }
        else
        {
            sleep_sec(resolved_dt);
        }

        std::size_t advance = 1;
        if (rate > 1.0)
        {
            step_budget += rate;
            advance = static_cast<std::size_t>(step_budget);
            step_budget -= static_cast<double>(advance);
            advance = std::max<std::size_t>(1, advance);
        }
        step += advance;
    }
    return true;
}

}  // namespace mr_planner::visualization
