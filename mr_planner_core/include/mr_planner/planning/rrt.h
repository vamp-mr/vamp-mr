#ifndef MR_PLANNER_RRT_H
#define MR_PLANNER_RRT_H

#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/core/graph.h"
#include "mr_planner/planning/roadmap.h"
#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <random>
#include <algorithm>
#include <chrono>

class RRTPlanner : public SingleAgentPlanner {
public:
    RRTPlanner(std::shared_ptr<PlanInstance> instance, int robot_id);
    RRTPlanner(std::shared_ptr<PlanInstance> instance, int robot_id, const PlannerOptions &options);

    virtual bool init(const PlannerOptions &options) override;
    virtual bool plan(const PlannerOptions &options) override;
    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions) override;
    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound) override;
    virtual bool plan(const PlannerOptions &options, double &lower_bound) override;
    virtual bool getPlan(RobotTrajectory &solution) const override;
    virtual double getPlanCost() const override;

    bool randomSample(RobotPose &new_sample);
    std::shared_ptr<Vertex> nearest(const std::shared_ptr<Vertex> &sample);
    void steer(const std::shared_ptr<Vertex> &nearest, const std::shared_ptr<Vertex> &sample, std::shared_ptr<Vertex> &new_vertex);
    bool checkStepCollision(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b);
    bool validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options);
    bool reachedGoal(const std::shared_ptr<Vertex> &vertex, const PlannerOptions &options);
    bool checkConstraint(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b, const PlannerOptions &options);

protected:
    double step_size_ = 4.0; // Currently step size is equal to max_dist_ for simplicity in constraint checkings
    std::vector<std::shared_ptr<Vertex>> tree_;

    double max_planning_time_ = 2.0;
    double max_dist_ = 4.0;
    RobotTrajectory solution_;
    MRTrajectory other_solutions_;

    std::chrono::time_point<std::chrono::system_clock> start_time_;
};

#endif // MR_PLANNER_RRT_H