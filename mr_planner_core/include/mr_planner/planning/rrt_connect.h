#ifndef MR_PLANNER_RRT_CONNECT_H
#define MR_PLANNER_RRT_CONNECT_H

#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/core/graph.h"
#include "mr_planner/planning/roadmap.h"
#include "mr_planner/planning/nn_kdtree.h"
#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <random>
#include <algorithm>
#include <chrono>

class RRTConnect : public SingleAgentPlanner {
public:
    RRTConnect(std::shared_ptr<PlanInstance> instance, int robot_id);
    RRTConnect(std::shared_ptr<PlanInstance> instance, int robot_id, const PlannerOptions &options);

    void setRandomSeed(unsigned int seed);
    void setStaticObstacles(const std::vector<RobotPose> &obstacles);

    virtual bool init(const PlannerOptions &options) override;
    virtual bool plan(const PlannerOptions &options) override;
    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions) override;
    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound) override;
    virtual bool plan(const PlannerOptions &options, double &lower_bound) override;
    virtual bool getPlan(RobotTrajectory &solution) const override;
    virtual double getPlanCost() const override;

    bool randomSample(RobotPose &new_sample);
    std::shared_ptr<Vertex> nearest(const std::shared_ptr<Vertex> &sample, const std::vector<std::shared_ptr<Vertex>> &tree_ref);
    void steer(const std::shared_ptr<Vertex> &nearest, const std::shared_ptr<Vertex> &sample, std::shared_ptr<Vertex> &new_vertex);
    bool checkStepCollision(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options);
    bool validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options, bool extending_start_tree = true);
    bool reachedGoal(const std::shared_ptr<Vertex> &vertex, const PlannerOptions &options);
    bool checkConstraint(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options);
    bool extractSolution(std::shared_ptr<Vertex> start_tree_current, std::shared_ptr<Vertex> goal_tree_current);
    std::vector<RobotPose> withStaticObstacles(const RobotPose &pose) const;
    void appendStaticObstacles(std::vector<RobotPose> &poses) const;

protected:
    double step_size_ = 2.0; // Currently step size is equal to max_dist_ for simplicity in constraint checkings
    std::vector<std::shared_ptr<Vertex>> start_tree_;
    std::vector<std::shared_ptr<Vertex>> goal_tree_;

    double max_planning_time_ = 2.0;
    double max_dist_ = 4.0;
    RobotTrajectory solution_;
    MRTrajectory other_solutions_;

    std::chrono::time_point<std::chrono::system_clock> start_time_;
    unsigned int rng_seed_{0};
    bool seed_set_{false};
    std::vector<RobotPose> static_obstacles_;

    mr_planner::planning::PoseKDTreeIndex<std::shared_ptr<Vertex>> start_nn_;
    mr_planner::planning::PoseKDTreeIndex<std::shared_ptr<Vertex>> goal_nn_;
    std::vector<float> nn_query_;
    std::size_t nn_dims_{0};
};

#endif // MR_PLANNER_RRT_CONNECT_H
