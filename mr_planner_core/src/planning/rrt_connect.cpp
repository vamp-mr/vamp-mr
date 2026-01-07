#include "mr_planner/planning/rrt_connect.h"
#include "mr_planner/core/logger.h"
#include <sstream>

RRTConnect::RRTConnect(std::shared_ptr<PlanInstance> instance, int robot_id)
    : SingleAgentPlanner(instance, robot_id) {
}

void RRTConnect::setStaticObstacles(const std::vector<RobotPose> &obstacles) {
    static_obstacles_.clear();
    static_obstacles_.reserve(obstacles.size());
    for (const auto &obs : obstacles) {
        if (obs.robot_id != robot_id_) {
            static_obstacles_.push_back(obs);
        }
    }
}

void RRTConnect::setRandomSeed(unsigned int seed) {
    rng_seed_ = seed;
    seed_set_ = true;
    if (instance_) {
        instance_->setRandomSeed(rng_seed_);
    }
}

bool RRTConnect::init(const PlannerOptions &options) {
    max_dist_ = options.max_dist;
    max_planning_time_ = options.rrt_max_planning_time;

    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);

    start_tree_.clear();
    goal_tree_.clear();

    auto start_vertex = std::make_shared<Vertex>(start_pose_);
    auto goal_vertex = std::make_shared<Vertex>(goal_pose_);

    auto format_shape = [](Object::Shape shape) {
        switch (shape) {
            case Object::Box: return std::string("box");
            case Object::Sphere: return std::string("sphere");
            case Object::Cylinder: return std::string("cylinder");
            case Object::Mesh: return std::string("mesh");
        }
        return std::string("unknown");
    };

    auto format_object = [&](const Object &obj) {
        std::ostringstream oss;
        oss << obj.name << " [" << format_shape(obj.shape) << "] ";
        oss << "pos(" << obj.x << ", " << obj.y << ", " << obj.z << ") ";
        oss << "quat(" << obj.qx << ", " << obj.qy << ", " << obj.qz << ", " << obj.qw << ") ";
        oss << "size(l=" << obj.length << ", w=" << obj.width << ", h=" << obj.height << ", r=" << obj.radius << ")";
        return oss.str();
    };

    // Check if start/goal are valid against the static environment and other robots.
    if (instance_->checkCollision(withStaticObstacles(start_pose_), false)) {
        log("RRTConnect: Start pose is in collision!", LogLevel::ERROR);
        return false;
    }

    // check if goal is in collision
    if (instance_->checkCollision(withStaticObstacles(goal_pose_), false)) {
        log("RRTConnect: Goal pose is in collision!", LogLevel::ERROR);
        const auto debug_links = instance_->debugCollidingLinks(withStaticObstacles(goal_pose_), false);
        int cnt = debug_links.size();
        log("Number of colliding links: " + std::to_string(cnt), LogLevel::WARN);
        for (const auto &contact : debug_links) {
            log("Colliding link: Robot " + std::to_string(contact.robot_a) + " " + format_object(contact.link_a) +
                " with Robot " + std::to_string(contact.robot_b) + " " + format_object(contact.link_b), LogLevel::WARN);
        }
        return false;
    }

    start_tree_.push_back(start_vertex);
    goal_tree_.push_back(goal_vertex);

    return true;
}

bool RRTConnect::plan(const PlannerOptions &options) {
    if (options.rrt_seed >= 0) {
        rng_seed_ = static_cast<unsigned int>(options.rrt_seed);
        seed_set_ = true;
    }
    if (seed_set_) {
        instance_->setRandomSeed(rng_seed_);
    }

    if (!init(options)) {
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.times.push_back(0.0);
        solution_.trajectory.push_back(goal_pose_);
        solution_.cost = 0.0;
        return false;
    }
    start_time_ = std::chrono::system_clock::now();
    bool success = false;

    if (options.interpolate_first_) {
        // try to connect start and goal directly first
        auto start_vertex = start_tree_.front();
        auto goal_vertex = goal_tree_.front();

        // steer towards goal in a loop
        while(true) {
            std::shared_ptr<Vertex> new_vertex;
            steer(start_vertex, goal_vertex, new_vertex);
            if(validateMotion(start_vertex, new_vertex, options, true)) {
                start_tree_.push_back(new_vertex);
                start_vertex = new_vertex;
                if(new_vertex->pose == goal_vertex->pose) {
                    success = extractSolution(new_vertex, goal_vertex);
                    return success;
                }
            } else {
                break;
            }
        }
    }
    // Implement the RRT-Connect algorithm here
    bool extending_start_tree = true;
    for(int i = 0; i < options.max_planning_iterations; i++) {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = current_time - start_time_;
        if (elapsed_time.count() > max_planning_time_) {
            log("RRTConnect: Time limit exceeded!", LogLevel::WARN);
            break;
        }

        auto &tree_a = extending_start_tree ? start_tree_ : goal_tree_;
        auto &tree_b = extending_start_tree ? goal_tree_ : start_tree_;

        RobotPose random_sample;
        if (!randomSample(random_sample)) {
            log("RRTConnect: Failed to sample a random pose", LogLevel::WARN);
            continue;
        }
        auto new_sample = std::make_shared<Vertex>(random_sample);
        std::shared_ptr<Vertex> nearest_vertex = nearest(new_sample, tree_a);
        std::shared_ptr<Vertex> new_tree_a_vertex;
        steer(nearest_vertex, new_sample, new_tree_a_vertex);

        if(validateMotion(nearest_vertex, new_tree_a_vertex, options, extending_start_tree)) {
            tree_a.push_back(new_tree_a_vertex);
            std::shared_ptr<Vertex> nearest_tree_b_vertex = nearest(new_tree_a_vertex, tree_b);
            std::shared_ptr<Vertex> new_tree_b_vertex;
            steer(nearest_tree_b_vertex, new_tree_a_vertex, new_tree_b_vertex);

            if(validateMotion(nearest_tree_b_vertex, new_tree_b_vertex, options, extending_start_tree)) {
                tree_b.push_back(new_tree_b_vertex);
                while(true) {
                    std::shared_ptr<Vertex> continued_tree_b_vertex;
                    steer(new_tree_b_vertex, new_tree_a_vertex, continued_tree_b_vertex);
                    if(validateMotion(new_tree_b_vertex, continued_tree_b_vertex, options, extending_start_tree)) {
                        tree_b.push_back(continued_tree_b_vertex);
                        new_tree_b_vertex = continued_tree_b_vertex;
                    } else {
                        break;
                    }
                    if(new_tree_b_vertex->pose == new_tree_a_vertex->pose) {
                        break;
                    }
                }
            }
            if(new_tree_b_vertex->pose == new_tree_a_vertex->pose) {
                log("RRTConnect found a solution!", LogLevel::INFO);
                if(extending_start_tree) {
                    success = extractSolution(new_tree_a_vertex, new_tree_b_vertex);
                } else {
                    success = extractSolution(new_tree_b_vertex, new_tree_a_vertex);
                }
                break;
            }
        }
        extending_start_tree = !extending_start_tree; // Alternate between extending start and goal trees
    }
    if (!success) {
        log("RRTConnect failed to find a solution, exceeding time limit or maximum iterations.", LogLevel::WARN);
    }

    return success;
}

bool RRTConnect::plan(const PlannerOptions &options, const MRTrajectory &other_solutions) {
    if (options.rrt_seed >= 0) {
        rng_seed_ = static_cast<unsigned int>(options.rrt_seed);
        seed_set_ = true;
    }
    if (seed_set_) {
        instance_->setRandomSeed(rng_seed_);
    }

    if (!init(options)) {
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.times.push_back(0.0);
        solution_.trajectory.push_back(goal_pose_);
        solution_.cost = 0.0;
        log("Failed to initialize RRTConnect", LogLevel::ERROR);
        return false;
    }
    start_time_ = std::chrono::system_clock::now();
    bool success = false;

    // Implement the RRT-Connect algorithm here

    return success;
}

bool RRTConnect::plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound) {
    if (options.rrt_seed >= 0) {
        rng_seed_ = static_cast<unsigned int>(options.rrt_seed);
        seed_set_ = true;
    }
    if (seed_set_) {
        instance_->setRandomSeed(rng_seed_);
    }
    return false; // Not implemented
}

bool RRTConnect::plan(const PlannerOptions &options, double &lower_bound) {
    if (options.rrt_seed >= 0) {
        rng_seed_ = static_cast<unsigned int>(options.rrt_seed);
        seed_set_ = true;
    }
    if (seed_set_) {
        instance_->setRandomSeed(rng_seed_);
    }
    return false; // Not implemented
}

bool RRTConnect::getPlan(RobotTrajectory &solution) const {
    solution = solution_;
    return true;
}

double RRTConnect::getPlanCost() const {
    return solution_.cost;
}

bool RRTConnect::extractSolution(std::shared_ptr<Vertex> start_tree_current, std::shared_ptr<Vertex> goal_tree_current) {
    std::vector<RobotPose> start_tree_traj;
    std::vector<RobotPose> goal_tree_traj;
    auto current = start_tree_current;
    // Bug inside the while loop
    while(current) {
        start_tree_traj.push_back(current->pose);
        current = current->parent;
    }
    std::reverse(start_tree_traj.begin(), start_tree_traj.end()); // Reverse to get the trajectory from start to current
    current = goal_tree_current;
    while(current) {
        goal_tree_traj.push_back(current->pose);
        current = current->parent;
    }
    start_tree_traj.pop_back(); // Remove the last pose since it is the same as the first pose of goal tree trajectory

    solution_.robot_id = robot_id_;
    solution_.times.clear();
    solution_.trajectory.clear();
    solution_.cost = 0.0;
    solution_.trajectory.insert(solution_.trajectory.end(), start_tree_traj.begin(), start_tree_traj.end());
    solution_.trajectory.insert(solution_.trajectory.end(), goal_tree_traj.begin(), goal_tree_traj.end());
    solution_.times.resize(solution_.trajectory.size());
    double time = 0.0;
    for (size_t i = 0; i < solution_.times.size(); ++i) {
        if (i > 0) {
            time += max_dist_; // Assuming uniform time step for simplicity
        }
        solution_.times[i] = time;
    }
    solution_.cost = time;

    return true;
}

bool RRTConnect::randomSample(RobotPose &new_sample) {
    RobotPose newpose = instance_->initRobotPose(robot_id_);
    bool found_sample = false;
    int tries = 0;
    int max_tries = 10;

    /* Goal biasing
    if (std::rand() % 100 < 10) {
        new_sample = goal_pose_;
        return true;
    }
    */
    while (!found_sample && tries < max_tries) {
        tries++;
        found_sample = instance_->sample(newpose);
        if (!found_sample) {
            continue;
        }
        if (instance_->checkCollision(withStaticObstacles(newpose), false)) {
            found_sample = false;
            continue;
        }
        new_sample = newpose;
    }
    return found_sample;
}

std::shared_ptr<Vertex> RRTConnect::nearest(const std::shared_ptr<Vertex> &sample, const std::vector<std::shared_ptr<Vertex>> &tree_ref) {
    double min_dist = std::numeric_limits<double>::max();
    std::shared_ptr<Vertex> nearest_vertex = nullptr;

    for (const auto &vertex : tree_ref) {
        double dist = instance_->computeDistance(vertex->pose, sample->pose);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_vertex = vertex;
        }
    }
    return nearest_vertex;
}

void RRTConnect::steer(const std::shared_ptr<Vertex> &nearest, const std::shared_ptr<Vertex> &sample, std::shared_ptr<Vertex> &new_vertex) {
    double dist = instance_->computeDistance(nearest->pose, sample->pose);
    if (dist < step_size_) {
        new_vertex = std::make_shared<Vertex>(sample->pose);
        new_vertex->time = nearest->time + step_size_;
        new_vertex->parent = nearest;
        return;
    }
    double ratio = step_size_ / dist;
    RobotPose new_pose = instance_->interpolate(nearest->pose, sample->pose, ratio);
    new_vertex = std::make_shared<Vertex>(new_pose);
    new_vertex->time = nearest->time + step_size_;
    new_vertex->parent = nearest;
}

bool RRTConnect::checkStepCollision(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options) {
    if (a->pose == b->pose) {
        return false;
    }

    if (static_obstacles_.empty()) {
        return !instance_->connect(a->pose, b->pose);
    }

    std::vector<RobotPose> motion_start = withStaticObstacles(a->pose);
    std::vector<RobotPose> motion_goal = withStaticObstacles(b->pose);
    double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, options.num_interpolations);
    return instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, false);
}

bool RRTConnect::validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options, bool extending_start_tree) {
    if(extending_start_tree)
        return !(a->pose == b->pose) 
            && (std::find(start_tree_.begin(), start_tree_.end(), b) == start_tree_.end() 
            && std::find(goal_tree_.begin(), goal_tree_.end(), b) == goal_tree_.end())
            && !checkStepCollision(a, b, options)
            && checkConstraint(a, b, options);
    else
        return !(a->pose == b->pose) 
            && (std::find(goal_tree_.begin(), goal_tree_.end(), b) == goal_tree_.end() 
            && std::find(start_tree_.begin(), start_tree_.end(), b) == start_tree_.end())
            && !checkStepCollision(b, a, options)
            && checkConstraint(b, a, options);
}

// NOTE: For goal tree, the vertex a should be the new vertex and b should be the nearest vertex in the goal tree
bool RRTConnect::checkConstraint(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options) {
    bool valid = true;
    for(auto constraint : options.constraints) {
        if(constraint.robot_id != robot_id_) {
            continue; // Skip constraints for other robots
        }
        // Do not need to check the lookup table since all the pose are newly sampled
        if(constraint.isAvoidance) {
            if(constraint.type == ConstraintType::VERTEX) {
                if(instance_->checkCollision({constraint.pose, b->pose}, true)) {
                    valid = false;
                    break;
                }
            } else if(constraint.type == ConstraintType::EDGE) {
                const std::vector<RobotPose> motion_start = {a->pose, constraint.pose};
                const std::vector<RobotPose> motion_goal = {b->pose, constraint.to_pose};
                const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, options.num_interpolations);
                if (instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true)) {
                    valid = false;
                    break;
                }
            }
        } else {
            if(constraint.type == ConstraintType::VERTEX) {
                if(b->pose == constraint.pose) {
                    valid = false;
                    break;
                }
            } else if(constraint.type == ConstraintType::EDGE) {
                if(constraint.pose == a->pose && constraint.to_pose == b->pose) {
                    valid = false;
                    break;
                }
            }
        }
        if(!valid) {
            break;
        }
    }
    return valid;
}

std::vector<RobotPose> RRTConnect::withStaticObstacles(const RobotPose &pose) const {
    std::vector<RobotPose> poses;
    poses.reserve(static_obstacles_.size() + 1);
    poses.push_back(pose);
    appendStaticObstacles(poses);
    return poses;
}

void RRTConnect::appendStaticObstacles(std::vector<RobotPose> &poses) const {
    for (const auto &obs : static_obstacles_) {
        if (obs.robot_id != robot_id_) {
            poses.push_back(obs);
        }
    }
}
