#ifndef LEGO_PLAN_H
#define LEGO_PLAN_H

#include <memory>
#include <vector>
#include <string>

#include "mr_planner/core/instance.h"

/**
 * @brief LegoPlan provides segment planning for LEGO policies/tasks.
 *
 * This class currently uses the internal RRTConnect planner (VAMP- or MoveIt-backed
 * PlanInstance) and is intended to stay ROS/MoveIt-free so it can live in a portable
 * `mr_planner_lego` layer.
 */
class LegoPlan {
public:
    /**
     * @brief Constructor for LegoPlan
     * @param planning_time_limit Maximum time allowed for planning (default: 5.0 seconds)
     */
    explicit LegoPlan(double planning_time_limit = 5.0);

    /**
     * @brief Destructor
     */
    ~LegoPlan() = default;

    /**
     * @brief Plan a trajectory for a single robot using RRTConnect
     * @param robot_id ID of the robot to plan for (0 or 1)
     * @param start_joints Starting joint configuration
     * @param goal_joints Goal joint configuration
     * @param solution Output trajectory solution
     * @param instance Planning instance for conversion utilities
     * @param activity_type Activity type to determine if policy-based planning
     * @param other_robot_joints Joint values of other robots for static obstacles
     * @return true if planning succeeded, false otherwise
     */
    bool planSegmentRRT(int robot_id,
                       const std::vector<double>& start_joints,
                       const std::vector<double>& goal_joints,
                       RobotTrajectory& solution,
                       std::shared_ptr<PlanInstance> instance,
                       int activity_type = -1,
                       const std::vector<std::vector<double>>& other_robot_joints = {});

    /**
     * @brief Set the planning time limit
     * @param time_limit Maximum planning time in seconds
     */
    void setPlanningTimeLimit(double time_limit);

    /**
     * @brief Get the current planning time limit
     * @return Current planning time limit in seconds
     */
    double getPlanningTimeLimit() const;

    /**
     * @brief Set the RRT seed for reproducible planning
     * @param seed Seed value (-1 for random)
     */
    void setRRTSeed(int seed);

    /**
     * @brief Get the current RRT seed
     * @return Current RRT seed
     */
    int getRRTSeed() const;

private:
    double planning_time_limit_;
    int rrt_seed_ = -1;

    /**
     * @brief Validate inputs for planning
     * @param robot_id Robot ID to validate
     * @param start_joints Starting joint configuration
     * @param goal_joints Goal joint configuration
     * @param instance Planning instance (used to validate robot DOF)
     * @return true if inputs are valid, false otherwise
     */
    bool validateInputs(int robot_id, 
                       const std::vector<double>& start_joints,
                       const std::vector<double>& goal_joints,
                       const std::shared_ptr<PlanInstance> &instance) const;
};

#endif // LEGO_PLAN_H
