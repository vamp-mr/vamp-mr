#include "mr_planner/planning/sipp_rrt.h"

#include "mr_planner/core/logger.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <utility>

// -----------------------------------------------------------------------------
// SI-RRT (Safe-Interval RRT-Connect) for a single robot with moving obstacles.
//
// References:
//  - Paper: Kerimov et al., "Safe Interval Randomized Path Planning For
//    Manipulators" (arXiv:2412.19567).
//  - Reference implementation: PathPlanning/ManipulationPlanning-SI-RRT
//    (MSIRRT/src/PlannerConnect.cpp).
//
// High-level idea (paper Alg.1):
//  - Grow two trees in configuration space:
//      T_start: rooted at q_start, times go forward (earliest-arrival semantics).
//      T_goal : rooted at q_goal,  times go backward (latest-departure semantics).
//  - Each node is (q, si=[t_low,t_high]) where si is a SAFE INTERVAL:
//      robot fixed at configuration q is collision-free w.r.t. moving obstacles
//      for all t in [t_low,t_high].
//  - Connecting edges use "wait-and-go":
//      wait at parent until a departure time that makes the motion safe, then
//      move at constant speed (vMax_) along the straight-line interpolation.
//
// Mapping to the reference repo / paper:
//  - extendTowards()      ~= extend() in PlannerConnect.cpp
//  - findSafeIntervals()  ~= collision_manager.get_safe_intervals(...)
//  - growTree()           ~= set_parent(...) / "setParent" in the paper
//  - tryConnectOtherTree()~= connect_trees(...) / connect(T_other, q_new, ...)
//  - buildSolutionFromConnection()
//      ~= uniteTrees() + "trim wait at q_new" (paper Fig.2),
//         implemented similarly to prune_goal_tree() in PlannerConnect.cpp by
//         copying the goal-branch onto the start tree while trimming the wait.
//
// Notes on *time representation* here:
//  - We discretize time on a fixed grid with step col_dt_ (seconds).
//  - Internally we do all interval overlap arithmetic on integer timesteps
//    (like the reference implementation operates on integer frames).
// -----------------------------------------------------------------------------

namespace
{
constexpr double kTimeEps = 1e-9;

struct ParentCandidate
{
    VertexPtr v;
    int travel_steps{0};
    int sort_key{0};
    bool static_checked{false};
    bool static_ok{false};
};
} // namespace

SIPP_RRT::SIPP_RRT(std::shared_ptr<PlanInstance> instance, int robot_id)
    : SingleAgentPlanner(std::move(instance), robot_id)
{
    vMax_ = instance_->getVMax(robot_id);
}

int SIPP_RRT::timeToStep(double t) const
{
    return static_cast<int>(std::llround(t / col_dt_));
}

double SIPP_RRT::stepToTime(int step) const
{
    return static_cast<double>(step) * col_dt_;
}

int SIPP_RRT::ceilDivSteps(double t) const
{
    return static_cast<int>(std::ceil(t / col_dt_ - 1e-12));
}

double SIPP_RRT::nearestDistance(const RobotPose &sample, VertexPtr &nearest_vertex, const Tree &tree) const
{
    assert(!tree.vertices.empty());

    const bool allow_kdtree =
        start_nn_.available() && nn_dims_ > 0 && tree.vertices.size() >= mr_planner::planning::kLinearScanThreshold;
    if (allow_kdtree)
    {
        const mr_planner::planning::PoseKDTreeIndex<VertexPtr> *index = nullptr;
        if (&tree == &start_tree_)
        {
            index = &start_nn_;
        }
        else if (&tree == &goal_tree_)
        {
            index = &goal_nn_;
        }

        if (index != nullptr)
        {
            if (nn_query_.size() != nn_dims_)
            {
                nn_query_.assign(nn_dims_, 0.0F);
            }
            mr_planner::planning::pack_joints_l1(sample, nn_query_.data(), nn_dims_);
            VertexPtr out;
            float dist_f = 0.0F;
            if (index->nearest(nn_query_.data(), &out, &dist_f) && out)
            {
                nearest_vertex = out;
                return static_cast<double>(dist_f);
            }
        }
    }

    double best = std::numeric_limits<double>::infinity();
    nearest_vertex = nullptr;
    for (const auto &v : tree.vertices)
    {
        const double d = instance_->computeDistance(v->pose, sample);
        if (d < best)
        {
            best = d;
            nearest_vertex = v;
        }
    }
    return best;
}

void SIPP_RRT::addRootIndexed(Tree &tree, const VertexPtr &vertex)
{
    tree.addRoot(vertex);
    if (!start_nn_.available() || nn_dims_ == 0 || !vertex)
    {
        return;
    }
    if (nn_query_.size() != nn_dims_)
    {
        nn_query_.assign(nn_dims_, 0.0F);
    }
    mr_planner::planning::pack_joints_l1(vertex->pose, nn_query_.data(), nn_dims_);
    if (&tree == &start_tree_)
    {
        start_nn_.insert(nn_query_.data(), vertex);
    }
    else if (&tree == &goal_tree_)
    {
        goal_nn_.insert(nn_query_.data(), vertex);
    }
}

void SIPP_RRT::addVertexIndexed(Tree &tree, const VertexPtr &vertex)
{
    tree.addVertex(vertex);
    if (!start_nn_.available() || nn_dims_ == 0 || !vertex)
    {
        return;
    }
    if (nn_query_.size() != nn_dims_)
    {
        nn_query_.assign(nn_dims_, 0.0F);
    }
    mr_planner::planning::pack_joints_l1(vertex->pose, nn_query_.data(), nn_dims_);
    if (&tree == &start_tree_)
    {
        start_nn_.insert(nn_query_.data(), vertex);
    }
    else if (&tree == &goal_tree_)
    {
        goal_nn_.insert(nn_query_.data(), vertex);
    }
}

GrowState SIPP_RRT::extendTowards(const RobotPose &target, Tree &tree, RobotPose &out_pose) const
{
    // RRT-Connect "extend" (space-only): pick nearest vertex, then either:
    //  - directly connect if within max_delta_, or
    //  - steer by max_delta_ towards the target.
    // Time/safe-interval logic is handled separately in growTree().
    if (tree.vertices.empty())
    {
        return TRAPPED;
    }

    VertexPtr nearest_v;
    const double dist = nearestDistance(target, nearest_v, tree);
    if (!nearest_v)
    {
        return TRAPPED;
    }

    if (dist <= max_delta_)
    {
        if (dist < 1e-9)
        {
            out_pose = target;
            return REACHED;
        }
        if (instance_->connect(nearest_v->pose, target))
        {
            out_pose = target;
            return REACHED;
        }
        return TRAPPED;
    }

    RobotPose steered = instance_->initRobotPose(robot_id_);
    if (instance_->steer(nearest_v->pose, target, max_delta_, steered))
    {
        out_pose = std::move(steered);
        return ADVANCED;
    }
    return TRAPPED;
}

bool SIPP_RRT::sample(RobotPose &new_pose)
{
    // Configuration-space sampling. Note: we rely on PlanInstance::sample()
    // to respect joint limits (and potentially pre-check collisions).
    new_pose = instance_->initRobotPose(robot_id_);
    int tries = 0;
    constexpr int kMaxTries = 10;
    while (tries++ < kMaxTries)
    {
        if (instance_->sample(new_pose))
        {
            return true;
        }
    }
    return false;
}

void SIPP_RRT::precompute_obs_pose()
{
    // Precompute moving obstacle poses on the same discretized timestep grid.
    // For t beyond the provided trajectory, we assume the obstacle stays put
    // at its last configuration (matches the older codebase behavior).
    moving_obs_span_ = 0.0;
    for (const auto &obs : obstacles_)
    {
        if (!obs.times.empty())
        {
            moving_obs_span_ = std::max(moving_obs_span_, obs.times.back());
        }
    }

    moving_obs_steps_ = std::max(0, static_cast<int>(std::ceil(moving_obs_span_ / col_dt_ - 1e-12)) + 1);

    obs_poses_.clear();
    obs_poses_.reserve(obstacles_.size());
    for (const RobotTrajectory &obs : obstacles_)
    {
        std::vector<RobotPose> poses;
        poses.reserve(static_cast<std::size_t>(moving_obs_steps_));

        int ind = 0;
        for (int s = 0; s < moving_obs_steps_; ++s)
        {
            const double t = stepToTime(s);
            while (ind + 1 < static_cast<int>(obs.times.size()) && obs.times[static_cast<std::size_t>(ind + 1)] <= t)
            {
                ind++;
            }

            if (obs.times.empty())
            {
                poses.push_back(instance_->initRobotPose(obs.robot_id));
                continue;
            }

            if (ind + 1 >= static_cast<int>(obs.times.size()))
            {
                poses.push_back(obs.trajectory[static_cast<std::size_t>(ind)]);
                continue;
            }

            const double t0 = obs.times[static_cast<std::size_t>(ind)];
            const double t1 = obs.times[static_cast<std::size_t>(ind + 1)];
            const double alpha = (t - t0) / (t1 - t0);
            poses.push_back(instance_->interpolate(obs.trajectory[static_cast<std::size_t>(ind)],
                                                  obs.trajectory[static_cast<std::size_t>(ind + 1)],
                                                  alpha));
        }
        obs_poses_.push_back(std::move(poses));
    }
}

bool SIPP_RRT::findSafeIntervals(const RobotPose &pose, std::vector<SafeInterval> &safe_interval)
{
    // Compute safe intervals [t_low, t_high] for a fixed configuration `pose`,
    // i.e., all timesteps where pose does NOT collide with any moving obstacle.
    //
    // This corresponds to getSafeIntervals() in the paper / reference repo,
    // but implemented with explicit per-timestep collision checks.
    //
    // Important: We first check collision against the *static* environment.
    // If pose is statically invalid, it has no safe intervals at all.
    safe_interval.clear();

    if (t_max_steps_ <= 0)
    {
        log("SIPP_RRT: t_max_steps_ not initialized", LogLevel::ERROR);
        return false;
    }

    if (instance_->checkCollision({pose}, false))
    {
        return false;
    }

    int start_step = -1;
    for (int s = 0; s <= t_max_steps_; ++s)
    {
        bool coll = false;
        for (std::size_t i = 0; i < obs_poses_.size(); ++i)
        {
            const int idx = (moving_obs_steps_ > 0) ? std::min(s, moving_obs_steps_ - 1) : 0;
            if (instance_->checkCollision({pose, obs_poses_[i][static_cast<std::size_t>(idx)]}, true))
            {
                coll = true;
                break;
            }
        }

        if (coll)
        {
            if (start_step >= 0 && start_step <= s - 1)
            {
                SafeInterval si;
                si.pose = pose;
                si.t_start = stepToTime(start_step);
                si.t_end = stepToTime(s - 1);
                safe_interval.push_back(std::move(si));
            }
            start_step = -1;
        }
        else
        {
            if (start_step < 0)
            {
                start_step = s;
            }
        }
    }

    if (start_step >= 0 && start_step <= t_max_steps_)
    {
        SafeInterval si;
        si.pose = pose;
        si.t_start = stepToTime(start_step);
        si.t_end = stepToTime(t_max_steps_);
        safe_interval.push_back(std::move(si));
    }

    return !safe_interval.empty();
}

bool SIPP_RRT::validateMotion(const RobotPose &pose_1, const RobotPose &pose_2, int t1_step, int t2_step) const
{
    // Dynamic collision checking for a "wait-and-go" edge:
    // - Assume the robot is at pose_1 at t1_step and at pose_2 at t2_step,
    //   moving with constant speed along PlanInstance::interpolate().
    // - Check collision against every moving obstacle at each discretized step.
    //
    // This is the analog of is_collision_motion(...) in the reference repo.
    if (t2_step < t1_step)
    {
        return false;
    }
    const int dt_steps = t2_step - t1_step;
    if (dt_steps == 0)
    {
        for (std::size_t i = 0; i < obs_poses_.size(); ++i)
        {
            const int idx = (moving_obs_steps_ > 0) ? std::min(t1_step, moving_obs_steps_ - 1) : 0;
            if (instance_->checkCollision({pose_1, obs_poses_[i][static_cast<std::size_t>(idx)]}, true))
            {
                return false;
            }
        }
        return true;
    }

    for (int s = 0; s <= dt_steps; ++s)
    {
        const double alpha = static_cast<double>(s) / static_cast<double>(dt_steps);
        const RobotPose a_pose = instance_->interpolate(pose_1, pose_2, alpha);
        const int t_step = t1_step + s;
        const int idx = (moving_obs_steps_ > 0) ? std::min(t_step, moving_obs_steps_ - 1) : 0;
        for (std::size_t i = 0; i < obs_poses_.size(); ++i)
        {
            if (instance_->checkCollision({a_pose, obs_poses_[i][static_cast<std::size_t>(idx)]}, true))
            {
                return false;
            }
        }
    }
    return true;
}

std::vector<VertexPtr> SIPP_RRT::growTree(const RobotPose &pose,
                                          const std::vector<SafeInterval> &safe_intervals,
                                          Tree &tree,
                                          bool is_goal_tree)
{
    // Given a new configuration `pose` and its safe intervals, create one node
    // per safe interval that can be connected from the existing tree using
    // wait-and-go edges (paper Alg.1, setParent()).
    //
    // The two trees differ in semantics:
    //  - Start tree (is_goal_tree=false): pick the EARLIEST arrival time in si.
    //  - Goal  tree (is_goal_tree=true ): pick the LATEST departure time in si.
    //
    // Implementation detail:
    //  - Each Vertex stores:
    //      time       := "main" time at this node (arrival for start tree,
    //                    latest departure for goal tree).
    //      other_time := time at the parent end of the edge (departure for start
    //                    tree, parent-arrival for goal tree).
    //
    // This is crucial: edge timing is stored on the CHILD vertex, never by
    // mutating the parent. The previous implementation mutated parents, which
    // becomes incorrect when a node has multiple children.
    std::vector<VertexPtr> added;
    if (safe_intervals.empty())
    {
        return added;
    }

    // Gather parent candidates within a radius.
    std::vector<ParentCandidate> candidates;
    candidates.reserve(tree.vertices.size());
    for (const auto &v : tree.vertices)
    {
        const double dist = instance_->computeDistance(v->pose, pose);
        if (dist > parent_radius_)
        {
            continue;
        }
        const double travel_t = dist / vMax_;
        const int travel_steps = std::max(0, static_cast<int>(std::ceil(travel_t / col_dt_ - 1e-12)));
        const int v_time = timeToStep(v->time);
        ParentCandidate cand;
        cand.v = v;
        cand.travel_steps = travel_steps;
        cand.sort_key = is_goal_tree ? (v_time - travel_steps) : (v_time + travel_steps);
        candidates.push_back(std::move(cand));
    }

    if (candidates.empty())
    {
        // Always allow at least the nearest node as a parent candidate.
        VertexPtr nearest_v;
        const double dist = nearestDistance(pose, nearest_v, tree);
        if (nearest_v)
        {
            const double travel_t = dist / vMax_;
            const int travel_steps = std::max(0, static_cast<int>(std::ceil(travel_t / col_dt_ - 1e-12)));
            const int v_time = timeToStep(nearest_v->time);
            ParentCandidate cand;
            cand.v = nearest_v;
            cand.travel_steps = travel_steps;
            cand.sort_key = is_goal_tree ? (v_time - travel_steps) : (v_time + travel_steps);
            candidates.push_back(std::move(cand));
        }
    }

    if (candidates.empty())
    {
        return added;
    }

    std::sort(candidates.begin(), candidates.end(), [is_goal_tree](const ParentCandidate &a, const ParentCandidate &b) {
        if (is_goal_tree)
        {
            return a.sort_key > b.sort_key; // latest first
        }
        return a.sort_key < b.sort_key; // earliest first
    });

    const auto safeIntervalToSteps = [&](const SafeInterval &si) -> std::pair<int, int> {
        return {timeToStep(si.t_start), timeToStep(si.t_end)};
    };

    const auto alreadyHasInterval = [&](const SafeInterval &si) -> bool {
        const auto it = tree.vertex_map.find(pose);
        if (it == tree.vertex_map.end())
        {
            return false;
        }
        const auto [t0, t1] = safeIntervalToSteps(si);
        for (const auto &v : it->second)
        {
            const auto [u0, u1] = safeIntervalToSteps(v->si);
            if (t0 == u0 && t1 == u1)
            {
                return true;
            }
        }
        return false;
    };

    for (const auto &si : safe_intervals)
    {
        if (alreadyHasInterval(si))
        {
            continue;
        }

        const auto [si_start, si_end] = safeIntervalToSteps(si);

        for (auto &cand : candidates)
        {
            if (cand.travel_steps == 0)
            {
                continue;
            }

            if (!cand.static_checked)
            {
                cand.static_ok = instance_->connect(cand.v->pose, pose);
                cand.static_checked = true;
            }
            if (!cand.static_ok)
            {
                continue;
            }

            const auto [p_start, p_end] = safeIntervalToSteps(cand.v->si);
            const int p_time = timeToStep(cand.v->time);

            if (!is_goal_tree)
            {
                // Start tree: choose earliest feasible departure time.
                int dep_lb = std::max({p_time, p_start, si_start - cand.travel_steps, 0});
                int dep_ub = std::min(p_end, si_end - cand.travel_steps);
                if (dep_lb > dep_ub)
                {
                    continue;
                }
                for (int dep = dep_lb; dep <= dep_ub; ++dep)
                {
                    const int arr = dep + cand.travel_steps;
                    if (arr < si_start || arr > si_end)
                    {
                        continue;
                    }
                    if (!validateMotion(cand.v->pose, pose, dep, arr))
                    {
                        continue;
                    }

                    auto v = std::make_shared<Vertex>(pose);
                    v->setSafeInterval(si);
                    v->addParent(cand.v);
                    v->setOtherTime(stepToTime(dep));
                    v->setTime(stepToTime(arr));
                    addVertexIndexed(tree, v);
                    totalTreeSize++;
                    added.push_back(std::move(v));
                    break; // one parent per safe interval
                }
            }
            else
            {
                // Goal tree: choose latest feasible departure time from `pose` to reach the parent.
                int parent_arr_lb = std::max({p_start, si_start + cand.travel_steps});
                int parent_arr_ub = std::min({p_end, si_end + cand.travel_steps, p_time});
                if (parent_arr_lb > parent_arr_ub)
                {
                    continue;
                }
                for (int parent_arr = parent_arr_ub; parent_arr >= parent_arr_lb; --parent_arr)
                {
                    const int dep = parent_arr - cand.travel_steps;
                    if (dep < si_start || dep > si_end)
                    {
                        continue;
                    }
                    if (!validateMotion(pose, cand.v->pose, dep, parent_arr))
                    {
                        continue;
                    }

                    auto v = std::make_shared<Vertex>(pose);
                    v->setSafeInterval(si);
                    v->addParent(cand.v);
                    v->setTime(stepToTime(dep));       // time at this node (latest departure)
                    v->setOtherTime(stepToTime(parent_arr)); // time at parent along this edge
                    addVertexIndexed(tree, v);
                    totalTreeSize++;
                    added.push_back(std::move(v));
                    break; // one parent per safe interval
                }
            }
        }
    }

    return added;
}

void SIPP_RRT::swap_trees()
{
    current_tree_ = (current_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    other_tree_ = (other_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    current_is_goal_tree_ = !current_is_goal_tree_;
}

bool SIPP_RRT::tryConnectOtherTree(const RobotPose &target_pose,
                                  const PlannerOptions &options,
                                  bool current_is_goal_tree,
                                  VertexPtr &out_start_node,
                                  VertexPtr &out_goal_node)
{
    // RRT-Connect "connect" step (paper Alg.1, connect()):
    // greedily extend the OTHER tree towards target_pose until either:
    //  - trapped (static collision / steering failure), or
    //  - reached (space), then check if both trees contain a node at the SAME
    //    configuration + SAME safe interval with time consistency.
    //
    // Connection condition:
    //  - both trees have a node (q_new, same_si)
    //  - and start_arrival_time <= goal_departure_time
    //
    // If satisfied, we return the pair of junction vertices.
    auto verticesAtPose = [](const Tree &tree, const RobotPose &pose) -> const std::vector<VertexPtr> * {
        const auto it = tree.vertex_map.find(pose);
        if (it == tree.vertex_map.end())
        {
            return nullptr;
        }
        return &it->second;
    };

    while (!terminate(options))
    {
        RobotPose next_pose;
        const GrowState gs = extendTowards(target_pose, *other_tree_, next_pose);
        if (gs == TRAPPED)
        {
            return false;
        }

        std::vector<SafeInterval> sis;
        if (!findSafeIntervals(next_pose, sis) || sis.empty())
        {
            return false;
        }

        const auto new_nodes = growTree(next_pose, sis, *other_tree_, !current_is_goal_tree);
        if (new_nodes.empty())
        {
            return false;
        }

        if (gs != REACHED)
        {
            continue;
        }

        const auto *current_nodes = verticesAtPose(*current_tree_, target_pose);
        const auto *other_nodes = verticesAtPose(*other_tree_, target_pose);
        if (!current_nodes || !other_nodes)
        {
            return false;
        }

        auto sameInterval = [&](const SafeInterval &a, const SafeInterval &b) -> bool {
            return timeToStep(a.t_start) == timeToStep(b.t_start) && timeToStep(a.t_end) == timeToStep(b.t_end);
        };

        if (!current_is_goal_tree)
        {
            // Current is start tree; other is goal tree.
            for (const auto &s_node : *current_nodes)
            {
                for (const auto &g_node : *other_nodes)
                {
                    if (!sameInterval(s_node->si, g_node->si))
                    {
                        continue;
                    }
                    if (s_node->time <= g_node->time + kTimeEps)
                    {
                        out_start_node = s_node;
                        out_goal_node = g_node;
                        return true;
                    }
                }
            }
        }
        else
        {
            // Current is goal tree; other is start tree.
            for (const auto &g_node : *current_nodes)
            {
                for (const auto &s_node : *other_nodes)
                {
                    if (!sameInterval(s_node->si, g_node->si))
                    {
                        continue;
                    }
                    if (s_node->time <= g_node->time + kTimeEps)
                    {
                        out_start_node = s_node;
                        out_goal_node = g_node;
                        return true;
                    }
                }
            }
        }

        return false;
    }

    return false;
}

bool SIPP_RRT::buildSolutionFromConnection(const VertexPtr &start_node, const VertexPtr &goal_node)
{
    // Unite the two trees once they meet at q_new (paper Alg.1, uniteTrees()).
    //
    // The paper notes (Fig.2) that the naive concatenation typically produces a
    // long wait at the junction, because:
    //  - start-branch tries to reach q_new as early as possible, while
    //  - goal-branch tries to leave q_new as late as possible.
    //
    // We therefore "trim" the wait at q_new by copying the goal-branch onto the
    // start tree while choosing the earliest feasible departure times that still
    // keep the remainder of the (goal) branch feasible. This mirrors the idea of
    // prune_goal_tree() in the reference implementation.
    if (!start_node || !goal_node)
    {
        return false;
    }
    if (!(start_node->pose == goal_node->pose))
    {
        return false;
    }

    // Copy the goal-tree branch onto the start tree while trimming waiting at the junction (like the SI-RRT paper).
    VertexPtr current = start_node;
    VertexPtr goal_child = goal_node;
    VertexPtr goal_parent = goal_child->parent;

    while (goal_parent)
    {
        const double dist = instance_->computeDistance(current->pose, goal_parent->pose);
        const int travel_steps = std::max(0, static_cast<int>(std::ceil((dist / vMax_) / col_dt_ - 1e-12)));

        const int deadline = timeToStep(goal_child->time);
        const int cur_time = timeToStep(current->time);
        const int cur_end = timeToStep(current->si.t_end);
        const int goal_si_start = timeToStep(goal_parent->si.t_start);
        const int goal_si_end = timeToStep(goal_parent->si.t_end);

        int dep_lb = std::max({cur_time, goal_si_start - travel_steps, 0});
        int dep_ub = std::min({deadline, cur_end, goal_si_end - travel_steps});

        bool found = false;
        if (dep_lb <= dep_ub)
        {
            for (int dep = dep_lb; dep <= dep_ub; ++dep)
            {
                const int arr = dep + travel_steps;
                if (arr < goal_si_start || arr > goal_si_end)
                {
                    continue;
                }
                if (!validateMotion(current->pose, goal_parent->pose, dep, arr))
                {
                    continue;
                }

                auto v = std::make_shared<Vertex>(goal_parent->pose);
                v->setSafeInterval(goal_parent->si);
                v->addParent(current);
                v->setOtherTime(stepToTime(dep));
                v->setTime(stepToTime(arr));
                addVertexIndexed(start_tree_, v);
                totalTreeSize++;
                current = v;
                found = true;
                break;
            }
        }

        if (!found)
        {
            const int dep = timeToStep(goal_child->time);
            const int arr = timeToStep(goal_child->other_time);
            auto v = std::make_shared<Vertex>(goal_parent->pose);
            v->setSafeInterval(goal_parent->si);
            v->addParent(current);
            v->setOtherTime(stepToTime(dep));
            v->setTime(stepToTime(arr));
            addVertexIndexed(start_tree_, v);
            totalTreeSize++;
            current = v;
        }

        goal_child = goal_parent;
        goal_parent = goal_parent->parent;
    }

    const VertexPtr finish = current;
    if (!finish)
    {
        return false;
    }

    // Build a piecewise "wait-then-go" trajectory from the start-tree chain.
    std::vector<VertexPtr> chain;
    for (VertexPtr v = finish; v != nullptr; v = v->parent)
    {
        chain.push_back(v);
    }
    std::reverse(chain.begin(), chain.end());
    if (chain.empty())
    {
        return false;
    }

    RobotTrajectory traj;
    traj.robot_id = robot_id_;
    traj.times.clear();
    traj.trajectory.clear();

    traj.times.push_back(chain.front()->time);
    traj.trajectory.push_back(chain.front()->pose);

    for (std::size_t i = 1; i < chain.size(); ++i)
    {
        const auto &prev = chain[i - 1];
        const auto &cur = chain[i];
        const double depart_t = cur->other_time;
        if (depart_t > prev->time + 1e-9)
        {
            traj.times.push_back(depart_t);
            traj.trajectory.push_back(prev->pose);
        }
        traj.times.push_back(cur->time);
        traj.trajectory.push_back(cur->pose);
    }

    traj.cost = traj.times.back();
    traj.num_nodes_expanded = static_cast<int>(start_tree_.vertices.size() + goal_tree_.vertices.size());

    if (traj.cost + 1e-9 < best_cost_)
    {
        best_cost_ = traj.cost;
        solution_ = std::move(traj);
    }

    return true;
}

bool SIPP_RRT::init(const PlannerOptions &options)
{
    // Initialize SI-RRT:
    //  1) validate static start/goal,
    //  2) precompute moving obstacle poses on the timestep grid,
    //  3) compute safe intervals for start/goal configurations,
    //  4) create roots for the two trees.
    //
    // We plan up to a finite horizon t_max_ (paper uses t_max=20s in eval).
    // Here we set:
    //   t_max_ = moving_obs_span_ + max(2*direct_travel, 5s)
    // which is typically enough to allow "wait for others to pass" while
    // remaining finite for safe-interval computation.
    no_plan_needed_ = false;
    best_cost_ = std::numeric_limits<double>::infinity();
    solution_ = RobotTrajectory();
    numIterations_ = 0;
    numValidSamples_ = 0;
    totalTreeSize = 0;

    start_tree_ = Tree();
    goal_tree_ = Tree();
    current_tree_ = &start_tree_;
    other_tree_ = &goal_tree_;
    current_is_goal_tree_ = false;

    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);

    if (instance_->checkCollision({start_pose_}, false))
    {
        log("SIPP_RRT: Start pose is in collision (static)", LogLevel::ERROR);
        instance_->checkCollision({start_pose_}, false, true);
        return false;
    }
    if (instance_->checkCollision({goal_pose_}, false))
    {
        log("SIPP_RRT: Goal pose is in collision (static)", LogLevel::ERROR);
        instance_->checkCollision({goal_pose_}, false, true);
        return false;
    }

    nn_dims_ = start_pose_.joint_values.size();
    nn_query_.assign(nn_dims_, 0.0F);
    start_nn_.reset(nn_dims_);
    goal_nn_.reset(nn_dims_);

    obstacles_ = options.obstacles;
    precompute_obs_pose();

    const double direct_travel = instance_->computeDistance(start_pose_, goal_pose_) / vMax_;
    min_time_ = direct_travel;

    // Choose a finite planning horizon (t_max) similar to the SI-RRT paper.
    t_max_ = std::max(moving_obs_span_, 0.0) + std::max(2.0 * direct_travel, 5.0);
    t_max_ = std::max(t_max_, direct_travel);
    t_max_steps_ = std::max(1, ceilDivSteps(t_max_));
    t_max_ = stepToTime(t_max_steps_);

    std::vector<SafeInterval> start_sis;
    if (!findSafeIntervals(start_pose_, start_sis))
    {
        log("SIPP_RRT: No safe intervals at start pose", LogLevel::ERROR);
        return false;
    }

    SafeInterval start_si{};
    bool found_start = false;
    for (const auto &si : start_sis)
    {
        if (si.t_start <= 0.0 + kTimeEps && si.t_end >= 0.0 - kTimeEps)
        {
            start_si = si;
            found_start = true;
            break;
        }
    }
    if (!found_start)
    {
        log("SIPP_RRT: Start pose is not safe at t=0", LogLevel::ERROR);
        return false;
    }

    auto start_v = std::make_shared<Vertex>(start_pose_);
    start_v->setSafeInterval(start_si);
    start_v->setTime(0.0);
    start_v->setOtherTime(0.0);
    addRootIndexed(start_tree_, start_v);
    totalTreeSize++;

    std::vector<SafeInterval> goal_sis;
    if (!findSafeIntervals(goal_pose_, goal_sis))
    {
        log("SIPP_RRT: No safe intervals at goal pose", LogLevel::ERROR);
        return false;
    }

    bool found_goal_root = false;
    for (const auto &si : goal_sis)
    {
        // We must be able to stay at the goal through t_max_ (the other robots finish moving by then).
        if (si.t_start - kTimeEps <= t_max_ && si.t_end + kTimeEps >= t_max_)
        {
            auto goal_v = std::make_shared<Vertex>(goal_pose_);
            goal_v->setSafeInterval(si);
            goal_v->setTime(t_max_);
            addRootIndexed(goal_tree_, goal_v);
            totalTreeSize++;
            found_goal_root = true;
        }
    }
    if (!found_goal_root)
    {
        log("SIPP_RRT: Goal pose is not safe at t_max", LogLevel::ERROR);
        return false;
    }

    // If start==goal, consider the trivial solution.
    if (instance_->computeDistance(start_pose_, goal_pose_) < 1e-9)
    {
        no_plan_needed_ = true;
    }

    return true;
}

bool SIPP_RRT::plan(const PlannerOptions &options)
{
    // Main SI-RRT loop (paper Alg.1):
    //  - optionally try a direct wait-and-go connection start->goal,
    //  - then iteratively:
    //      sample q, extend current tree in space, compute safe intervals,
    //      attach intervals via growTree(), connect the other tree, swap trees.
    start_time_ = std::chrono::high_resolution_clock::now();

    if (!init(options))
    {
        return false;
    }

    if (no_plan_needed_)
    {
        solution_.robot_id = robot_id_;
        solution_.times = {0.0};
        solution_.trajectory = {start_pose_};
        solution_.cost = 0.0;
        best_cost_ = 0.0;
        return true;
    }

    // Fast path: try a direct wait-and-go edge from start to goal.
    if (options.interpolate_first_)
    {
        if (instance_->connect(start_pose_, goal_pose_))
        {
            const VertexPtr start_root = start_tree_.roots.empty() ? nullptr : start_tree_.roots[0];
            if (start_root && !goal_tree_.roots.empty())
            {
                const auto &goal_si = goal_tree_.roots[0]->si;
                const int travel_steps =
                    std::max(0, static_cast<int>(std::ceil((instance_->computeDistance(start_pose_, goal_pose_) / vMax_) / col_dt_ - 1e-12)));
                const int start_end = timeToStep(start_root->si.t_end);
                const int goal_start = timeToStep(goal_si.t_start);
                const int goal_end = timeToStep(goal_si.t_end);

                int dep_lb = std::max(0, goal_start - travel_steps);
                int dep_ub = std::min(start_end, goal_end - travel_steps);
                for (int dep = dep_lb; dep <= dep_ub; ++dep)
                {
                    const int arr = dep + travel_steps;
                    if (arr < goal_start || arr > goal_end)
                    {
                        continue;
                    }
                    if (!validateMotion(start_pose_, goal_pose_, dep, arr))
                    {
                        continue;
                    }

                    auto goal_v = std::make_shared<Vertex>(goal_pose_);
                    goal_v->setSafeInterval(goal_si);
                    goal_v->addParent(start_root);
                    goal_v->setOtherTime(stepToTime(dep));
                    goal_v->setTime(stepToTime(arr));
                    addVertexIndexed(start_tree_, goal_v);
                    totalTreeSize++;

                    buildSolutionFromConnection(goal_v, goal_tree_.roots[0]); // start_node==goal_v (in start tree), goal_node is goal root
                    if (options.terminate_on_first_sol && best_cost_ < std::numeric_limits<double>::infinity())
                    {
                        return true;
                    }
                    break;
                }
            }
        }
    }

    while (!terminate(options))
    {
        numIterations_++;

        RobotPose q_sample;
        if (!sample(q_sample))
        {
            swap_trees();
            continue;
        }
        numValidSamples_++;

        RobotPose q_new;
        const GrowState gs = extendTowards(q_sample, *current_tree_, q_new);
        if (gs == TRAPPED)
        {
            swap_trees();
            continue;
        }

        std::vector<SafeInterval> sis;
        if (!findSafeIntervals(q_new, sis) || sis.empty())
        {
            swap_trees();
            continue;
        }

        const auto new_nodes = growTree(q_new, sis, *current_tree_, current_is_goal_tree_);
        if (new_nodes.empty())
        {
            swap_trees();
            continue;
        }

        VertexPtr start_node;
        VertexPtr goal_node;
        if (tryConnectOtherTree(q_new, options, current_is_goal_tree_, start_node, goal_node))
        {
            (void)buildSolutionFromConnection(start_node, goal_node);
            if (options.terminate_on_first_sol && best_cost_ < std::numeric_limits<double>::infinity())
            {
                return true;
            }
        }

        swap_trees();
    }

    return best_cost_ < std::numeric_limits<double>::infinity();
}

bool SIPP_RRT::terminate(const PlannerOptions &options)
{
    if (numIterations_ > options.max_planning_iterations)
    {
        return true;
    }
    const auto elapsed = std::chrono::high_resolution_clock::now() - start_time_;
    const double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
    if (elapsed_seconds > options.max_planning_time)
    {
        return true;
    }
    if (options.terminate_on_first_sol && best_cost_ < std::numeric_limits<double>::infinity())
    {
        return true;
    }
    return false;
}

bool SIPP_RRT::getPlan(RobotTrajectory &solution) const
{
    if (best_cost_ < std::numeric_limits<double>::infinity())
    {
        solution = solution_;
        return true;
    }
    return false;
}

double SIPP_RRT::getPlanCost() const
{
    return best_cost_;
}
