// Multi-robot planner interfaces and implementations: AbstractPlanner base class,
// PriorityPlanner (sequential prioritized planning), and CBSPlanner (Conflict-Based
// Search over per-robot PRM roadmaps), plus free-function utilities for saving,
// loading, validating, and retiming multi-robot trajectories.
#ifndef MR_PLANNER_PLANNER_H
#define MR_PLANNER_PLANNER_H

#include "mr_planner/planning/voxel_grid.h"
#include "mr_planner/core/instance.h" // Include the abstract problem instance definition
#include "mr_planner/core/metrics.h"
#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/planning/prm.h"
#include <memory>
#include <vector>
#include <random>
#include <cstdint>

#if MR_PLANNER_WITH_ROS
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#endif

// Abstract planner class
/// Base interface for multi-robot planners: plan() computes trajectories for
/// all robots in the instance, getPlan() retrieves the resulting solution.
class AbstractPlanner {
public:
    // Initialize the planner with a specific planning problem instance
    AbstractPlanner(std::shared_ptr<PlanInstance> instance) : instance_(instance) {
        num_robots_ = instance->getNumberOfRobots();
    }
    
    // Perform the planning process
    virtual bool plan(const PlannerOptions &options) = 0;

    // Retrieve the resulting plan; returns true on success.
    virtual bool getPlan(MRTrajectory &solution) const = 0;

    virtual ~AbstractPlanner() = default;

    double getPlanTime() const {
        return planning_time_;
    }

protected:
    int num_robots_;
    std::shared_ptr<PlanInstance> instance_;
    double planning_time_ = 0;
};



// Example of a concrete planner class that implements the AbstractPlanner interface
// This is where you would implement specific planning algorithms
/// Prioritized planner: plans robots one at a time in (optionally random)
/// priority order, treating earlier robots' trajectories as moving obstacles
/// for the later ones.
class PriorityPlanner : public AbstractPlanner {
public:
    PriorityPlanner(std::shared_ptr<PlanInstance> instance);

    virtual bool plan(const PlannerOptions &options) override;

    virtual bool getPlan(MRTrajectory &solution) const override;

protected:
    std::vector<SingleAgentPlannerPtr> agent_planners_;
    MRTrajectory solution_;
    bool solved = false;
};

/// Node in the CBS constraint tree: the accumulated constraints, the per-robot
/// solution planned under them, and cost / conflict statistics for ordering.
struct CBSNode {
    std::vector<int> robots;
    std::vector<Conflict> conflicts;
    std::vector<Constraint> constraints;
    double cost;
    double makespan;
    int num_pairs;
    int num_conflicts;
    int num_col_checks = 0;
    double lower_bound;
    MRTrajectory solution;
    MRTrajectory speedup_solution;
    std::uint64_t salt = 0;
};

/// Open-list ordering for CBS: lower makespan first, then fewer conflicts,
/// then fewer conflicting pairs; a random salt breaks remaining ties.
class CompareNode {
public:
    bool operator()(const CBSNode* a, const CBSNode* b) const {
        if (a->makespan != b->makespan) return a->makespan > b->makespan;
        if (a->num_conflicts != b->num_conflicts) return a->num_conflicts > b->num_conflicts;
        if (a->num_pairs != b->num_pairs) return a->num_pairs > b->num_pairs;
        if (a->salt != b->salt) return a->salt > b->salt;
        return false; // full tie
    }
};

/// Focal-list ordering for CBS: fewer conflicts first, then lower makespan.
class CompareFocal {
public:
    bool operator()(const CBSNode* a, const CBSNode* b) const {
        return a->num_conflicts > b->num_conflicts || (a->num_conflicts == b->num_conflicts && a->makespan > b->makespan);
    }
};

/// Conflict-Based Search planner. Each robot plans on its own PRM roadmap;
/// robot-robot vertex/edge conflicts are resolved by branching on constraints
/// in a best-first constraint-tree search (optionally with a focal list).
class CBSPlanner : public AbstractPlanner {
public:
    CBSPlanner(std::shared_ptr<PlanInstance> instance);

    CBSPlanner(std::shared_ptr<PlanInstance> instance, std::vector<std::shared_ptr<RoadMap>> roadmaps);

    CBSPlanner(std::shared_ptr<PlanInstance> instance, std::vector<std::shared_ptr<RoadMap>> roadmaps, std::shared_ptr<VoxelGrid> voxel_grid);

    /// Run CBS until a conflict-free solution is found or the time limit expires.
    virtual bool plan(const struct PlannerOptions &options) override;

    /// Copy the best solution found; returns false if unsolved.
    virtual bool getPlan(MRTrajectory &solution) const override;

    double getPlanTime() const;

    /// Request termination of the underlying single-agent planners.
    void stop();

    /// Find the first same-timestep robot-robot (or robot-target) collision in the node's solution.
    bool findVertexConflict(const CBSNode *node, Conflict &conflict);

    /// Find the first robot-robot collision along interpolated edge motions in the node's solution.
    bool findEdgeConflict(const CBSNode *node, Conflict &conflict, const PlannerOptions &options);

    /// Count conflicting robot pairs and total conflicts in the node's solution; returns {num_pairs, num_conflicts}.
    std::pair<int, int> countNumConflicts(const CBSNode *node, const PlannerOptions &options);

    /// Swap every robot's start and goal (for planning the reverse query).
    void swapStartGoal();

    /// Reverse the stored solution back to the original start-to-goal direction after swapStartGoal().
    void revertSolution();

    /// Retime a synchronized solution so each timestep takes only the slowest robot's execution time.
    MRTrajectory accelerateSolution(const MRTrajectory &solution);

    /// Branch on a conflict: create both child nodes, each constraining one of the two robots.
    std::pair<CBSNode*, CBSNode*> generateChildNodes(CBSNode *current, const Conflict &conflict, const PlannerOptions &options);

    /// Create one child node that constrains robot i of the conflict and replans that robot.
    CBSNode *generateChildNode(CBSNode *current, const Conflict &conflict, const PlannerOptions &options, int i);

    /// Append the constraint derived from the conflict (for robot i) to the child node.
    void addConstraint(CBSNode *current, CBSNode *newNode, const Conflict &conflict, int i);

protected:
    std::shared_ptr<VoxelGrid> voxel_grid_;
    std::vector<SingleAgentPlannerPtr> agent_planners_;
    std::vector<std::shared_ptr<RoadMap>> roadmaps_;
    MRTrajectory solution_;
    bool solved = false;
    const double epsilon = 0.0;
    double time_limit_ = 120.0;

    int col_count = 0;

    std::mt19937_64 node_salt_rng_;
    bool node_salt_rng_seeded_ = false;

    
    void seedNodeSaltRng(const PlannerOptions &options);
    std::uint64_t nextNodeSalt();
};

// utils
RobotPose myInterpolate(const RobotPose &a, const RobotPose &b, double t);

#if MR_PLANNER_WITH_ROS
bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    MRTrajectory &solution,
                    bool reset_speed = true);


bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    int robot_id,
                    RobotTrajectory &solution);

bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const moveit_msgs::RobotTrajectory &plan_traj,
                  const std::string &file_name);
#endif
                  
bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const MRTrajectory &synced_traj,
                  const std::string &file_name);

#if MR_PLANNER_WITH_ROS
/* time is assumed to be uniform as dt */
bool loadSolution(std::shared_ptr<PlanInstance> instance,
                  const std::string &file_name,
                  double dt,
                  moveit_msgs::RobotTrajectory &plan_traj);

/* time is supplied in the first column*/
bool loadSolution(std::shared_ptr<PlanInstance> instance,
                  const std::string &file_name,
                  moveit_msgs::RobotTrajectory &plan_traj);
#endif

bool validateSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    double col_dt);

/* assuming uniform discretiziation, check for collisions*/
bool validateSolution(std::shared_ptr<PlanInstance> instance,
                       const MRTrajectory &solution);

void retimeSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    MRTrajectory &retime_solution,
                    double dt);

void retimeSolution(std::shared_ptr<PlanInstance> instance,
                    const RobotTrajectory &solution,
                    RobotTrajectory &retime_solution,
                    int robot_id);

#if MR_PLANNER_WITH_ROS
void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    moveit_msgs::RobotTrajectory &retime_traj,
                    double new_dt);
#endif

void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                        const MRTrajectory &solution,
                        MRTrajectory &retime_solution,
                        double new_dt);
void removeWait(std::shared_ptr<PlanInstance> instance,
                        MRTrajectory &solution);
#if MR_PLANNER_WITH_ROS
bool validateSolution(std::shared_ptr<PlanInstance> instance,
                     const moveit_msgs::RobotTrajectory &plan_traj);

bool optimizeTrajectory(std::shared_ptr<PlanInstance> instance,
                        const moveit_msgs::RobotTrajectory& input_trajectory,
                        const std::string& group_name,
                        robot_model::RobotModelConstPtr robot_model,
                        const ros::NodeHandle& node_handle,
                        moveit_msgs::RobotTrajectory& smoothed_traj
                        );
#endif

#endif // MR_PLANNER_PLANNER_H
