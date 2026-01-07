#include "mr_planner/planning/rrt.h"
#include "mr_planner/core/logger.h"

// RRTPlanner class
RRTPlanner::RRTPlanner(std::shared_ptr<PlanInstance> instance, int robot_id)
    : SingleAgentPlanner(instance, robot_id) {
}

bool RRTPlanner::init(const PlannerOptions &options) {
    max_dist_ = options.max_dist;
    max_planning_time_ = options.rrt_max_planning_time;

    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);
    tree_.clear();

    auto start_vertex = std::make_shared<Vertex>(start_pose_);
    tree_.push_back(start_vertex);

    return true;
}

bool RRTPlanner::plan(const PlannerOptions &options) {
    if(!init(options)) {
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.times.push_back(0.0);
        solution_.trajectory.push_back(goal_pose_);
        solution_.cost = 0.0;
        std::cout << "Failed to initialize RRT" << std::endl;
        return false;
    }
    start_time_ = std::chrono::system_clock::now();
    bool success = false;
    while (true) {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = current_time - start_time_;
        if (elapsed_time.count() > max_planning_time_) {
            std::cout << "RRT time limit exceeded!\n";
            break;
        }

        RobotPose random_sample;
        if (!randomSample(random_sample)) {
            std::cout << "Failed to sample a random pose" << std::endl;
            break;
        }
        auto new_sample = std::make_shared<Vertex>(random_sample);
        auto nearest_vertex = nearest(new_sample);
        std::shared_ptr<Vertex> new_vertex;
        steer(nearest_vertex, new_sample, new_vertex);

        if (!validateMotion(nearest_vertex, new_vertex, options)) {
            continue;
        }

        tree_.push_back(new_vertex);

        if (reachedGoal(new_vertex, options)) {
            auto goal_vertex = std::make_shared<Vertex>(goal_pose_);
            goal_vertex->time = new_vertex->time + instance_->computeDistance(new_vertex->pose, goal_pose_);
            goal_vertex->parent = new_vertex;
            tree_.push_back(goal_vertex);
            
            solution_.robot_id = robot_id_;
            solution_.times.clear();
            solution_.trajectory.clear();
            solution_.cost = 0.0;
            std::shared_ptr<Vertex> current = goal_vertex;
            while (current != nullptr) {
                solution_.times.push_back(current->time);
                solution_.trajectory.push_back(current->pose);
                current = current->parent;
            }
            std::reverse(solution_.times.begin(), solution_.times.end());
            std::reverse(solution_.trajectory.begin(), solution_.trajectory.end());
            solution_.cost = new_vertex->time;
            std::cout << "RRT found a solution!" << std::endl;
            success = true;
            break;
        }
    }
    return success;
}

bool RRTPlanner::plan(const PlannerOptions &options, const MRTrajectory &other_solutions) {
    if(!init(options)) {
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.times.push_back(0.0);
        solution_.trajectory.push_back(goal_pose_);
        solution_.cost = 0.0;
        std::cout << "Failed to initialize RRT" << std::endl;
        return false;
    }
    other_solutions_ = other_solutions;
    bool success = true;
    return success;
}

bool RRTPlanner::plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound) {
    return false;
}

bool RRTPlanner::plan(const PlannerOptions &options, double &lower_bound) {
    return false;
}

bool RRTPlanner::getPlan(RobotTrajectory &solution) const {
    solution = solution_;
    return true;
}

double RRTPlanner::getPlanCost() const {
    return solution_.cost;
}

bool RRTPlanner::randomSample(RobotPose &new_sample) {
    RobotPose newpose = instance_->initRobotPose(robot_id_);
    bool found_sample = false;
    int tries = 0;
    int max_tries = 10;

    // Goal biasing
    if (std::rand() % 100 < 10) {
        new_sample = goal_pose_;
        return true;
    }

    while (!found_sample && tries < max_tries) {
        tries++;
        found_sample = instance_->sample(newpose);
        if (!found_sample) {
            continue;
        }
        new_sample = newpose;
    }
    return found_sample;
}

std::shared_ptr<Vertex> RRTPlanner::nearest(const std::shared_ptr<Vertex> &sample) {
    double min_dist = std::numeric_limits<double>::max();
    std::shared_ptr<Vertex> nearest_vertex = nullptr;
    for (const auto &vertex : tree_) {
        double dist = instance_->computeDistance(vertex->pose, sample->pose);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_vertex = vertex;
        }
    }
    return nearest_vertex;
}

void RRTPlanner::steer(const std::shared_ptr<Vertex> &nearest, const std::shared_ptr<Vertex> &sample, std::shared_ptr<Vertex> &new_vertex) {
    double dist = instance_->computeDistance(nearest->pose, sample->pose);
    if(dist < step_size_) {
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

bool RRTPlanner::reachedGoal(const std::shared_ptr<Vertex> &vertex, const PlannerOptions &options) {
    return instance_->computeDistance(vertex->pose, goal_pose_) < step_size_
        && !checkStepCollision(vertex, std::make_shared<Vertex>(goal_pose_))
        && checkConstraint(vertex, std::make_shared<Vertex>(goal_pose_), options);
}

bool RRTPlanner::validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options) {
    return !(a->pose == b->pose) 
            && (std::find(tree_.begin(), tree_.end(), b) == tree_.end()) 
            && !checkStepCollision(a, b)
            && checkConstraint(a, b, options);
}

bool RRTPlanner::checkStepCollision(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) {
    return !(a->pose == b->pose) && !instance_->connect(a->pose, b->pose);
}

bool RRTPlanner::checkConstraint(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options) {
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