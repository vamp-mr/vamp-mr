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
class AbstractPlanner {
public:
    // Initialize the planner with a specific planning problem instance
    AbstractPlanner(std::shared_ptr<PlanInstance> instance) : instance_(instance) {
        num_robots_ = instance->getNumberOfRobots();
    }
    
    // Perform the planning process
    virtual bool plan(const PlannerOptions &options) = 0;

    // Retrieve the plan (if needed, depending on your design, this could return a path, a series of actions, etc.)
    // For simplicity, this could return a boolean indicating success for now,
    // but you might want to define a more complex structure for the plan itself.
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

class CompareFocal {
public:
    bool operator()(const CBSNode* a, const CBSNode* b) const {
        return a->num_conflicts > b->num_conflicts || (a->num_conflicts == b->num_conflicts && a->makespan > b->makespan);
    }
};

class CBSPlanner : public AbstractPlanner {
public:
    CBSPlanner(std::shared_ptr<PlanInstance> instance);

    CBSPlanner(std::shared_ptr<PlanInstance> instance, std::vector<std::shared_ptr<RoadMap>> roadmaps);

    CBSPlanner(std::shared_ptr<PlanInstance> instance, std::vector<std::shared_ptr<RoadMap>> roadmaps, std::shared_ptr<VoxelGrid> voxel_grid);

    virtual bool plan(const struct PlannerOptions &options) override;

    virtual bool getPlan(MRTrajectory &solution) const override;

    double getPlanTime() const;

    void stop();

    bool findVertexConflict(const CBSNode *node, Conflict &conflict);

    bool findEdgeConflict(const CBSNode *node, Conflict &conflict, const PlannerOptions &options);

    std::pair<int, int> countNumConflicts(const CBSNode *node, const PlannerOptions &options);

    void swapStartGoal();

    void revertSolution();

    MRTrajectory accelerateSolution(const MRTrajectory &solution);

    std::pair<CBSNode*, CBSNode*> generateChildNodes(CBSNode *current, const Conflict &conflict, const PlannerOptions &options);

    CBSNode *generateChildNode(CBSNode *current, const Conflict &conflict, const PlannerOptions &options, int i);

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
