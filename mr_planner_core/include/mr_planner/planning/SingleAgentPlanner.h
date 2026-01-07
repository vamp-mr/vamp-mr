#ifndef SINGLE_AGENT_PLANNER_H
#define SINGLE_AGENT_PLANNER_H

#include "mr_planner/core/instance.h" // Include the abstract problem instance definition
#include "mr_planner/core/graph.h"
#include <memory>
#include <vector>
#include <chrono>
#include <unordered_map>

enum ConstraintType {
    VERTEX,
    EDGE,
    LEQLENGTH,
    GLENGTH
};

struct Conflict {
    std::vector<int> robots;
    std::vector<int> time_idx;
    ConstraintType type;
    bool isTarget = false;
};

struct Constraint {
    int robot_id;
    int other_robot_id;
    double time;
    RobotPose pose;
    RobotPose to_pose;
    ConstraintType type;
    bool isAvoidance = false;
    RobotPose self_vertex;
    RobotPose self_edge_start;
    RobotPose self_edge_end;
};

struct PlannerOptions {
    double max_planning_time = 2.0;
    double rrt_max_planning_time = 2.0;
    int max_planning_iterations = 10000;
    int num_samples = 1000;
    double max_dist = 4.0;
    int num_interpolations = 24;
    double pruning_radius = 1e-6;
    int rrt_seed = -1;
    bool isRoot = false;
    bool terminate_on_first_sol = true;
    bool pp_random_order = false;
    bool interpolate_first_ = true;
    std::string single_agent_planner = "STRRT";
    std::string log_fname = "";
    MRTrajectory obstacles;
    std::vector<struct Constraint> constraints;

    // constructor
    PlannerOptions() = default;
    PlannerOptions(double max_planning_time, int max_planning_iterations)
    {
        this->max_planning_time = max_planning_time;
        this->max_planning_iterations = max_planning_iterations;
    }
};


// Abstract planner class
class SingleAgentPlanner {
public:
    // Initialize the planner with a specific planning problem instance
    SingleAgentPlanner(std::shared_ptr<PlanInstance> instance,
                      int robot_id) : instance_(instance), robot_id_(robot_id)
                     {}

    virtual bool init(const PlannerOptions &options);
    
    // Perform the planning process
    virtual bool plan(const PlannerOptions &options) = 0;

    virtual bool plan(const PlannerOptions &options, double &lower_bound)
        {throw std::runtime_error("Not implemented"); return false;};

    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions)
        {throw std::runtime_error("Not implemented"); return false;};

    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound)
        {throw std::runtime_error("Not implemented"); return false;};

    // Retrieve the plan (if needed, depending on your design, this could return a path, a series of actions, etc.)
    // For simplicity, this could return a boolean indicating success for now,
    // but you might want to define a more complex structure for the plan itself.
    virtual bool getPlan(RobotTrajectory &solution) const = 0;

    virtual double getPlanCost() const = 0;

    virtual void reSampling(const PlannerOptions &options) {}

    virtual void applyDenseMap() {}

    virtual void swapStartGoal() {}

    virtual ~SingleAgentPlanner() = default;

    virtual bool terminate(const PlannerOptions &options) {
        return false;
    }

protected:
    int robot_id_;
    RobotPose start_pose_;
    RobotPose goal_pose_;
    std::shared_ptr<PlanInstance> instance_;
};


typedef std::shared_ptr<SingleAgentPlanner> SingleAgentPlannerPtr;

#endif // SINGLE_AGENT_PLANNER_H
