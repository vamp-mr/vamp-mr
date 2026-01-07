#include <mr_planner/planning/composite_rrt.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

CompositeRRTCPlanner::CompositeRRTCPlanner(std::shared_ptr<PlanInstance> instance)
    : AbstractPlanner(std::move(instance)) {}

bool CompositeRRTCPlanner::plan(const PlannerOptions &options) {
    planning_time_ = 0.0;
    solved = false;
    solution_.clear();

    if (!initialize(options)) {
        return false;
    }

    start_time_ = std::chrono::steady_clock::now();

    bool extend_start_tree = true;
    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        if (timeExceeded()) {
            break;
        }

        std::vector<RobotPose> sample;
        if (!randomCompositeSample(sample)) {
            continue;
        }

        auto &tree_a = extend_start_tree ? start_tree_ : goal_tree_;
        auto &tree_b = extend_start_tree ? goal_tree_ : start_tree_;

        auto nearest_vertex = nearest(tree_a, sample);
        if (!nearest_vertex) {
            extend_start_tree = !extend_start_tree;
            continue;
        }

        auto new_vertex = steer(nearest_vertex, sample);
        if (!new_vertex) {
            extend_start_tree = !extend_start_tree;
            continue;
        }

        if (!validateMotion(nearest_vertex, new_vertex, options)) {
            extend_start_tree = !extend_start_tree;
            continue;
        }

        tree_a.push_back(new_vertex);

        CompositeVertexPtr connection_vertex;
        const auto state = connectTree(tree_b, new_vertex, options, connection_vertex);

        if (state == GrowState::REACHED && connection_vertex) {
            CompositeVertexPtr start_meeting = extend_start_tree ? new_vertex : connection_vertex;
            CompositeVertexPtr goal_meeting = extend_start_tree ? connection_vertex : new_vertex;

            if (buildSolution(start_meeting, goal_meeting, options)) {
                planning_time_ = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - start_time_)
                                     .count();
                return true;
            }
        }

        extend_start_tree = !extend_start_tree;
    }

    planning_time_ = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_time_)
                         .count();
    return false;
}

bool CompositeRRTCPlanner::getPlan(MRTrajectory &solution) const {
    if (!solved) {
        return false;
    }
    solution = solution_;
    return true;
}

bool CompositeRRTCPlanner::initialize(const PlannerOptions &options) {
    if (!instance_) {
        return false;
    }

    step_size_ = std::max(options.max_dist, 1e-4);
    max_planning_time_ = options.rrt_max_planning_time;
    max_iterations_ = std::max(options.max_planning_iterations, 1);
    distance_tolerance_ = std::max(options.pruning_radius, 1e-4);

    start_config_ = instance_->getStartPoses();
    goal_config_ = instance_->getGoalPoses();

    if (start_config_.empty() || goal_config_.empty() ||
        start_config_.size() != goal_config_.size()) {
        return false;
    }

    if (!validateConfiguration(start_config_) || !validateConfiguration(goal_config_)) {
        return false;
    }

    start_tree_.clear();
    goal_tree_.clear();

    auto start_vertex = std::make_shared<CompositeVertex>(start_config_);
    auto goal_vertex = std::make_shared<CompositeVertex>(goal_config_);

    start_vertex->cost = 0.0;
    goal_vertex->cost = 0.0;

    start_tree_.push_back(std::move(start_vertex));
    goal_tree_.push_back(std::move(goal_vertex));

    return true;
}

bool CompositeRRTCPlanner::randomCompositeSample(std::vector<RobotPose> &sample) const {
    if (num_robots_ <= 0) {
        return false;
    }

    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    sample.clear();

    if (distribution(rng_) < goal_bias_) {
        sample = goal_config_;
        return true;
    }

    // For tightly constrained multi-robot environments (e.g., panda_four_bins), sampling all robots
    // independently and then checking the full composite collision can have a very low acceptance rate.
    // Instead, sample robots sequentially and reject early, while always validating a *full* robot set
    // (some VAMP instances do not support collision checking on subsets).
    constexpr int kCompositeAttempts = 64;
    constexpr int kRobotAttempts = 64;
    for (int attempt = 0; attempt < kCompositeAttempts; ++attempt)
    {
        sample = start_config_;

        bool sampled_all = true;
        for (int robot = 0; robot < num_robots_; ++robot)
        {
            bool found = false;
            for (int rtry = 0; rtry < kRobotAttempts; ++rtry)
            {
                RobotPose pose = instance_->initRobotPose(robot);
                if (!instance_->sample(pose))
                {
                    continue;
                }

                auto probe = sample;
                probe[robot] = std::move(pose);
                if (instance_->checkCollision(probe, false))
                {
                    continue;
                }

                sample = std::move(probe);
                found = true;
                break;
            }
            if (!found)
            {
                sampled_all = false;
                break;
            }
        }

        if (!sampled_all)
        {
            continue;
        }

        if (!validateConfiguration(sample))
        {
            continue;
        }

        return true;
    }

    return false;
}

CompositeRRTCPlanner::CompositeVertexPtr CompositeRRTCPlanner::nearest(
    const std::vector<CompositeVertexPtr> &tree,
    const std::vector<RobotPose> &sample) const {
    if (tree.empty() || sample.size() != static_cast<std::size_t>(num_robots_)) {
        return nullptr;
    }

    double best_distance = std::numeric_limits<double>::max();
    CompositeVertexPtr best_vertex;

    for (const auto &vertex : tree) {
        if (!vertex) {
            continue;
        }
        const double dist = compositeDistance(vertex->poses, sample);
        if (dist < best_distance) {
            best_distance = dist;
            best_vertex = vertex;
        }
    }

    return best_vertex;
}

CompositeRRTCPlanner::CompositeVertexPtr CompositeRRTCPlanner::steer(
    const CompositeVertexPtr &from,
    const std::vector<RobotPose> &target) const {
    if (!from || target.size() != static_cast<std::size_t>(num_robots_)) {
        return nullptr;
    }

    const double distance = compositeDistance(from->poses, target);
    if (distance <= distance_tolerance_) {
        return nullptr;
    }

    const double ratio = std::min(1.0, step_size_ / distance);
    std::vector<RobotPose> new_config;
    new_config.reserve(static_cast<std::size_t>(num_robots_));

    for (int robot = 0; robot < num_robots_; ++robot) {
        const auto &from_pose = from->poses[robot];
        const auto &target_pose = target[robot];
        if (ratio >= 1.0 - 1e-9) {
            new_config.push_back(target_pose);
        } else {
            new_config.push_back(instance_->interpolate(from_pose, target_pose, ratio));
        }
    }

    auto vertex = std::make_shared<CompositeVertex>(std::move(new_config));
    vertex->parent = from;
    vertex->cost = from->cost + compositeDistance(from->poses, vertex->poses);
    return vertex;
}

bool CompositeRRTCPlanner::validateConfiguration(const std::vector<RobotPose> &config) const {
    if (config.size() != static_cast<std::size_t>(num_robots_)) {
        return false;
    }
    return !instance_->checkCollision(config, false);
}

bool CompositeRRTCPlanner::validateMotion(const CompositeVertexPtr &from,
                                          const CompositeVertexPtr &to,
                                          const PlannerOptions &options) const {
    if (!from || !to) {
        return false;
    }

    // const double step = instance_->computeMotionStepSize(from->poses, to->poses,
    //                                                      options.num_interpolations);
    if (instance_->checkMultiRobotMotion(from->poses, to->poses, 0.1, false)) {
        return false;
    }

    return true;
}

GrowState CompositeRRTCPlanner::connectTree(std::vector<CompositeVertexPtr> &tree,
                                            const CompositeVertexPtr &target,
                                            const PlannerOptions &options,
                                            CompositeVertexPtr &last_added) {
    last_added.reset();

    if (tree.empty() || !target) {
        return GrowState::TRAPPED;
    }

    auto nearest_vertex = nearest(tree, target->poses);
    if (!nearest_vertex) {
        return GrowState::TRAPPED;
    }

    if (posesEqual(nearest_vertex->poses, target->poses)) {
        last_added = nearest_vertex;
        return GrowState::REACHED;
    }

    auto new_vertex = steer(nearest_vertex, target->poses);
    if (!new_vertex || !validateMotion(nearest_vertex, new_vertex, options)) {
        return GrowState::TRAPPED;
    }

    tree.push_back(new_vertex);
    last_added = new_vertex;

    if (posesEqual(new_vertex->poses, target->poses)) {
        return GrowState::REACHED;
    }

    auto current = new_vertex;
    while (true) {
        auto next = steer(current, target->poses);
        if (!next) {
            break;
        }
        if (!validateMotion(current, next, options)) {
            break;
        }
        tree.push_back(next);
        current = next;
        last_added = current;

        if (posesEqual(current->poses, target->poses)) {
            return GrowState::REACHED;
        }
    }

    return GrowState::ADVANCED;
}

bool CompositeRRTCPlanner::posesEqual(const std::vector<RobotPose> &a,
                                      const std::vector<RobotPose> &b) const {
    if (a.size() != b.size()) {
        return false;
    }
    return compositeDistance(a, b) <= distance_tolerance_;
}

double CompositeRRTCPlanner::compositeDistance(const std::vector<RobotPose> &a,
                                               const std::vector<RobotPose> &b) const {
    if (a.size() != b.size()) {
        return std::numeric_limits<double>::max();
    }

    double distance = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        distance = std::max(distance, instance_->computeDistance(a[i], b[i]));
    }
    return distance;
}

std::vector<std::vector<RobotPose>> CompositeRRTCPlanner::traceToRoot(
    const CompositeVertexPtr &leaf) const {
    std::vector<std::vector<RobotPose>> branch;
    for (auto current = leaf; current != nullptr; current = current->parent) {
        branch.push_back(current->poses);
    }
    return branch;
}

bool CompositeRRTCPlanner::buildSolution(const CompositeVertexPtr &start_meeting,
                                         const CompositeVertexPtr &goal_meeting,
                                         const PlannerOptions &) {
    if (!start_meeting || !goal_meeting) {
        return false;
    }

    auto start_branch = traceToRoot(start_meeting);
    if (start_branch.empty()) {
        return false;
    }
    std::reverse(start_branch.begin(), start_branch.end());

    auto goal_branch = traceToRoot(goal_meeting);
    if (goal_branch.empty()) {
        return false;
    }

    if (!posesEqual(start_branch.back(), goal_branch.front())) {
        return false;
    }

    goal_branch.erase(goal_branch.begin());

    std::vector<std::vector<RobotPose>> composite_path;
    composite_path.reserve(start_branch.size() + goal_branch.size());
    composite_path.insert(composite_path.end(), start_branch.begin(), start_branch.end());
    composite_path.insert(composite_path.end(), goal_branch.begin(), goal_branch.end());

    if (composite_path.empty()) {
        return false;
    }

    solution_.clear();
    solution_.resize(num_robots_);
    for (int robot = 0; robot < num_robots_; ++robot) {
        auto &traj = solution_[robot];
        traj.robot_id = robot;
        traj.trajectory.reserve(composite_path.size());
        traj.times.reserve(composite_path.size());
    }

    double time_accum = 0.0;
    for (std::size_t idx = 0; idx < composite_path.size(); ++idx) {
        if (composite_path[idx].size() != static_cast<std::size_t>(num_robots_)) {
            return false;
        }

        if (idx > 0) {
            double segment_time = 0.0;
            const auto &prev = composite_path[idx - 1];
            const auto &curr = composite_path[idx];
            for (int robot = 0; robot < num_robots_; ++robot) {
                segment_time = std::max(
                    segment_time,
                    instance_->computeDistance(prev[robot], curr[robot])) / instance_->getVMax(robot);
            }
            time_accum += segment_time;
        }

        for (int robot = 0; robot < num_robots_; ++robot) {
            auto &traj = solution_[robot];
            traj.trajectory.push_back(composite_path[idx][robot]);
            traj.times.push_back(time_accum);
        }
    }

    for (int robot = 0; robot < num_robots_; ++robot) {
        auto &traj = solution_[robot];
        if (!traj.times.empty()) {
            traj.times.front() = 0.0;
            traj.cost = traj.times.back();
        } else {
            traj.cost = 0.0;
        }
    }

    solved = true;
    return true;
}

bool CompositeRRTCPlanner::timeExceeded() const {
    if (max_planning_time_ <= 0.0) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start_time_).count();
    return elapsed > max_planning_time_;
}

void CompositeRRTCPlanner::setSeed(int seed) {
    rng_.seed(static_cast<std::mt19937::result_type>(seed));
    if (instance_) {
        instance_->setRandomSeed(seed);
    }
}
