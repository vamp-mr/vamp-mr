#ifndef SIPP_RRT_H
#define SIPP_RRT_H

#include "mr_planner/planning/SingleAgentPlanner.h" 
#include "mr_planner/core/graph.h"

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
/*  */
    bool findSafeIntervals(const RobotPose &pose, std::vector<SafeInterval> &safe_interval);

    bool sample(RobotPose &new_pose);

    GrowState extend(const RobotPose &new_pose, Tree &tree, RobotPose &added_pose, RobotPose &nearest_pose, bool goalTree);

    GrowState connect(const RobotPose &delta_pose, std::vector<VertexPtr> &connected_vertex, Tree &tree, bool goalTree);

    GrowState setSIParent(VertexPtr &new_sample, const RobotPose &nearest_pose, Tree &tree, bool goalTree, bool otherParent);

    GrowState step(const RobotPose &new_pose, std::vector<VertexPtr> &added_vertex, Tree &tree, bool goalTree, bool connect);

    bool interpolate(const RobotPose &pose_1, const RobotPose &pose_2, RobotTrajectory &solution);

    void shortenSITime(VertexPtr connected_vertex, bool goal_tree);

    void update_solution(std::vector<VertexPtr> &new_sample, bool goalTree);

    void swap_trees();

    double nearest(const RobotPose &sample, VertexPtr &nearest_vertex, const Tree &tree, bool goalTree);

    bool validateMotion(const RobotPose &pose_1, const RobotPose &pose_2, double t1, double t2);
    
    void precompute_obs_pose();

private:
    Tree start_tree_;
    Tree goal_tree_;
    Tree* current_tree_ = &start_tree_;
    Tree* other_tree_ = &goal_tree_;
    bool goal_tree = false;

    double min_time_ = 0.0;
    double max_delta_ = 1.0;
    double vMax_ = 1.0;
    double col_dt_ = 0.05;
    int numIterations_ = 0;
    int totalTreeSize = 0;
    int numValidSamples_ = 0;
    std::chrono::time_point<std::chrono::system_clock> start_time_;

    RobotTrajectory solution_;
    double best_cost_ = std::numeric_limits<double>::infinity();
   
    // obstacles;
    double moving_obs_span_ = 0.0;
    int moving_obs_steps_ = 0;
    MRTrajectory obstacles_;
    std::vector<std::vector<RobotPose>> obs_poses_; // robot X span_steps

    // random number generator
    std::mt19937 rng_;
};

#endif // SINGLE_AGENT_PLANNER_H
