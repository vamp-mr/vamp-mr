#include "mr_planner/applications/lego/lego_plan.h"
#include "mr_planner/core/logger.h"
#include "mr_planner/core/task.h"
#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/planning/rrt_connect.h"
#include <cmath>

LegoPlan::LegoPlan(double planning_time_limit) : planning_time_limit_(planning_time_limit) {}

bool LegoPlan::planSegmentRRT(int robot_id,
                             const std::vector<double>& start_joints,
                             const std::vector<double>& goal_joints,
                             RobotTrajectory& solution,
                             std::shared_ptr<PlanInstance> instance,
                             int activity_type,
                             const std::vector<std::vector<double>>& other_robot_joints)
{
    // Validate inputs
    if (!validateInputs(robot_id, start_joints, goal_joints, instance)) {
        log("LegoPlan::planSegmentRRT - Invalid inputs", LogLevel::ERROR);
        return false;
    }
    (void)activity_type;

    // Clear previous solution
    solution.trajectory.clear();
    solution.times.clear();
    solution.cost = 0.0;
    solution.robot_id = robot_id;

    // Use custom RRTConnect planner
    RRTConnect rrt_connect_planner(instance, robot_id);
    PlannerOptions options;
    options.rrt_max_planning_time = planning_time_limit_;
    options.max_dist = 2.0;
    options.rrt_seed = (rrt_seed_ >= 0) ? (rrt_seed_ + robot_id) : rrt_seed_;

    instance->setStartPose(robot_id, start_joints);
    instance->setGoalPose(robot_id, goal_joints);

    // Set up static obstacles (other robots) whenever provided.
    if (!other_robot_joints.empty()) {
        std::vector<RobotPose> static_obstacles;
        for (int other_id = 0; other_id < static_cast<int>(other_robot_joints.size()); ++other_id) {
            if (other_id == robot_id) {
                continue;
            }
            if (other_robot_joints[other_id].empty()) {
                continue;
            }
            RobotPose other_pose = instance->initRobotPose(other_id);
            other_pose.joint_values = other_robot_joints[other_id];
            static_obstacles.push_back(other_pose);
        }
        if (!static_obstacles.empty()) {
            rrt_connect_planner.setStaticObstacles(static_obstacles);
        }
    }

    bool success = rrt_connect_planner.plan(options);
    if (success) {
        log("RRT-Connect segment planning succeeded for robot " + std::to_string(robot_id), LogLevel::INFO);
        success &= rrt_connect_planner.getPlan(solution);
    } else {
        log("RRT-Connect segment planning failed for robot " + std::to_string(robot_id), LogLevel::ERROR);
    }
    return success;
}

void LegoPlan::setPlanningTimeLimit(double time_limit)
{
    if (time_limit > 0.0) {
        planning_time_limit_ = time_limit;
    } else {
        log("LegoPlan::setPlanningTimeLimit - Invalid time limit: " + std::to_string(time_limit) + ". Using default value.", LogLevel::WARN);
    }
}

double LegoPlan::getPlanningTimeLimit() const
{
    return planning_time_limit_;
}

void LegoPlan::setRRTSeed(int seed)
{
    rrt_seed_ = seed;
    log("LegoPlan::setRRTSeed - RRT seed set to: " + std::to_string(seed), LogLevel::INFO);
}

int LegoPlan::getRRTSeed() const
{
    return rrt_seed_;
}

bool LegoPlan::validateInputs(int robot_id, 
                             const std::vector<double>& start_joints,
                             const std::vector<double>& goal_joints,
                             const std::shared_ptr<PlanInstance> &instance) const
{
    if (!instance) {
        log("LegoPlan::validateInputs - instance is null", LogLevel::ERROR);
        return false;
    }

    // Check robot_id bounds
    if (robot_id < 0 || robot_id >= instance->getNumberOfRobots()) {
        log("LegoPlan::validateInputs - Invalid robot_id: " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }

    // Check joint configuration sizes
    const size_t expected_joints = instance->getRobotDOF(robot_id);
    
    if (start_joints.size() != expected_joints) {
        log("LegoPlan::validateInputs - Start joints size mismatch. Expected: " + std::to_string(expected_joints) + 
            ", Got: " + std::to_string(start_joints.size()), LogLevel::ERROR);
        return false;
    }

    if (goal_joints.size() != expected_joints) {
        log("LegoPlan::validateInputs - Goal joints size mismatch. Expected: " + std::to_string(expected_joints) + 
            ", Got: " + std::to_string(goal_joints.size()), LogLevel::ERROR);
        return false;
    }

    // Check for NaN or infinite values
    for (size_t i = 0; i < start_joints.size(); ++i) {
        if (!std::isfinite(start_joints[i])) {
            log("LegoPlan::validateInputs - Invalid start joint value at index " + std::to_string(i) + 
                ": " + std::to_string(start_joints[i]), LogLevel::ERROR);
            return false;
        }
    }

    for (size_t i = 0; i < goal_joints.size(); ++i) {
        if (!std::isfinite(goal_joints[i])) {
            log("LegoPlan::validateInputs - Invalid goal joint value at index " + std::to_string(i) + 
                ": " + std::to_string(goal_joints[i]), LogLevel::ERROR);
            return false;
        }
    }

    return true;
}
