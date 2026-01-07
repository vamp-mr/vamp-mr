#ifndef SPACE_TIME_RRT_H
#define SPACE_TIME_RRT_H

#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/core/graph.h"

// Example of a concrete planner class that implements the AbstractPlanner interface
// This is where you would implement specific planning algorithms
class STRRT : public SingleAgentPlanner {
public:
    STRRT(std::shared_ptr<PlanInstance> instance,
                      int robot_id);

    virtual bool init(const PlannerOptions &options) override;

    virtual bool plan(const PlannerOptions &options) override;

    virtual bool plan(const PlannerOptions &options, double &lower_bound) override;

    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions) override;

    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound) override;

    virtual bool getPlan(RobotTrajectory &solution) const override;

    virtual double getPlanCost() const override;

    virtual void reSampling(const PlannerOptions &options) override;

    virtual void applyDenseMap() override {}

    virtual bool terminate(const PlannerOptions &options) override;

    virtual void swapStartGoal() override;

    bool shouldExpandTime();

    bool sampleConditionally(std::shared_ptr<Vertex> &new_sample);

    void sampleGoal(std::shared_ptr<Vertex> &new_goal);

    void expandTime();

    GrowState extend(const std::shared_ptr<Vertex> &new_sample, Tree &tree, std::shared_ptr<Vertex> &added_sample, bool goalTree);

    GrowState connect(const std::shared_ptr<Vertex> &new_sample, Tree &tree, bool goalTree);

    void update_solution(std::shared_ptr<Vertex> &new_sample, bool goalTree);

    void prune_trees();

    void swap_trees();

    double distanceSpaceTime(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b);

    void nearest(const std::shared_ptr<Vertex> &sample, std::shared_ptr<Vertex> &nearest_vertex, const Tree &tree, bool goalTree);

    bool validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b);

private:
    Tree start_tree_;
    Tree goal_tree_;
    Tree* current_tree_ = &start_tree_;
    Tree* other_tree_ = &goal_tree_;
    bool goal_tree = false;

    double min_time_ = 0.0;
    double max_time_ = 0.0;
    double max_deltat_ = 0.5;
    bool time_bounded_ = false;
    double vMax_ = 1.0;
    double col_dt_ = 0.05;
    int numIterations_ = 0;
    int totalTreeSize = 0;
    int numValidSamples_ = 0;
    int batch_size = 500;
    int cur_batch_size = 0;
    std::chrono::time_point<std::chrono::system_clock> start_time_;

    RobotTrajectory solution_;
    double best_cost_ = std::numeric_limits<double>::infinity();
    MRTrajectory obstacles_;

    // random number generator
    std::mt19937 rng_;
};

#endif // SPACE_TIME_RRT_H