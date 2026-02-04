#ifndef SIPP_RRT_H
#define SIPP_RRT_H

#include "mr_planner/planning/SingleAgentPlanner.h" 
#include "mr_planner/core/graph.h"
#include "mr_planner/planning/nn_kdtree.h"

// Example of a concrete planner class that implements the AbstractPlanner interface
// This is where you would implement specific planning algorithms
class SIPP_RRT : public SingleAgentPlanner {
public:
    SIPP_RRT(std::shared_ptr<PlanInstance> instance,
                      int robot_id);

    virtual bool init(const PlannerOptions &options) override;

    virtual bool plan(const PlannerOptions &options) override;

    virtual bool getPlan(RobotTrajectory &solution) const override;

    virtual double getPlanCost() const override;

    virtual bool terminate(const PlannerOptions &options) override;

private:
    bool findSafeIntervals(const RobotPose &pose, std::vector<SafeInterval> &safe_interval);
    bool sample(RobotPose &new_pose);
    GrowState extendTowards(const RobotPose &target, Tree &tree, RobotPose &out_pose) const;
    std::vector<VertexPtr> growTree(const RobotPose &pose,
                                    const std::vector<SafeInterval> &safe_intervals,
                                    Tree &tree,
                                    bool is_goal_tree);
    bool tryConnectOtherTree(const RobotPose &target_pose,
                             const PlannerOptions &options,
                             bool current_is_goal_tree,
                             VertexPtr &out_start_node,
                             VertexPtr &out_goal_node);
    bool buildSolutionFromConnection(const VertexPtr &start_node, const VertexPtr &goal_node);
    bool validateMotion(const RobotPose &pose_1, const RobotPose &pose_2, int t1_step, int t2_step) const;
    void precompute_obs_pose();
    void swap_trees();
    double nearestDistance(const RobotPose &sample, VertexPtr &nearest_vertex, const Tree &tree) const;
    void addRootIndexed(Tree &tree, const VertexPtr &vertex);
    void addVertexIndexed(Tree &tree, const VertexPtr &vertex);
    int timeToStep(double t) const;
    double stepToTime(int step) const;
    int ceilDivSteps(double t) const;

    Tree start_tree_;
    Tree goal_tree_;
    Tree* current_tree_ = &start_tree_;
    Tree* other_tree_ = &goal_tree_;
    bool current_is_goal_tree_ = false;

    double min_time_ = 0.0;
    double max_delta_ = 2.0;
    double parent_radius_ = 6.0;
    double vMax_ = 1.0;
    double col_dt_ = 0.05;
    int numIterations_ = 0;
    int totalTreeSize = 0;
    int numValidSamples_ = 0;
    double t_max_ = 0.0;
    int t_max_steps_ = 0;
    std::chrono::time_point<std::chrono::system_clock> start_time_;

    RobotTrajectory solution_;
    double best_cost_ = std::numeric_limits<double>::infinity();
    bool no_plan_needed_ = false;
   
    // obstacles;
    double moving_obs_span_ = 0.0;
    int moving_obs_steps_ = 0;
    MRTrajectory obstacles_;
    std::vector<std::vector<RobotPose>> obs_poses_; // robot X span_steps

    // random number generator
    std::mt19937 rng_;

    mr_planner::planning::PoseKDTreeIndex<VertexPtr> start_nn_;
    mr_planner::planning::PoseKDTreeIndex<VertexPtr> goal_nn_;
    mutable std::vector<float> nn_query_;
    std::size_t nn_dims_{0};
};

#endif // SINGLE_AGENT_PLANNER_H
