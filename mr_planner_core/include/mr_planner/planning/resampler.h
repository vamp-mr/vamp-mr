#ifndef MR_PLANNER_RESAMPLER_H
#define MR_PLANNER_RESAMPLER_H

#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/core/graph.h"
#include "mr_planner/planning/roadmap.h"
#include "mr_planner/planning/rrt.h"
#include "mr_planner/planning/rrt_connect.h"
#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <random>
#include <algorithm>
#include <chrono>

class Resampler {
public:
    Resampler(std::shared_ptr<PlanInstance> instance, int robot_id, std::shared_ptr<Graph> roadmap);

    bool resample(const PlannerOptions &options);
    bool resample(const PlannerOptions &options, const MRTrajectory &other_solutions);

    void addTrajToRoadmap(const RobotTrajectory &solution, const PlannerOptions &options);
    bool validateMotion(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v, const PlannerOptions &options);

protected:
    std::shared_ptr<PlanInstance> instance_;
    std::shared_ptr<Graph> roadmap_;
    std::shared_ptr<RRTConnect> planner_;
    int robot_id_;
    int num_threads_ = 10; // Number of threads to use for parallel planning
};

#endif // MR_PLANNER_RESAMPLER_H