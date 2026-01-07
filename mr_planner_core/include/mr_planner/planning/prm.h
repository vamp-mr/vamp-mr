#ifndef MR_PLANNER_PRM_H
#define MR_PLANNER_PRM_H

#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/core/graph.h"
#include "mr_planner/planning/voxel_grid.h"
#include "mr_planner/planning/roadmap.h"
#include "mr_planner/planning/resampler.h"
#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <random>
#include <algorithm>
#include <chrono>

struct AStarNode {
    std::shared_ptr<Vertex> vertex;
    double f;
    double g; // roadmap step size sum
    double h;
    int timestep;
    int num_conflicts = 0;
    std::shared_ptr<AStarNode> parent;
    bool in_open = false;
    bool in_focal = false;
    bool closed = false;
    std::uint64_t salt = 0; // random bits for tie-breaking

    AStarNode()  : vertex(nullptr), f(0), g(0), h(0), num_conflicts(0), parent(nullptr) {};

    AStarNode(std::shared_ptr<Vertex> vertex_, double f_, double g_, double h_) : vertex(vertex_), f(f_), g(g_), h(h_) {};

    AStarNode(const AStarNode& other) = default;
    AStarNode& operator=(const AStarNode& other) = default;

    bool operator==(const AStarNode &other) const {
        return vertex->id == other.vertex->id && g == other.g;
    }

};


class PRM : public SingleAgentPlanner {
public:
    PRM(std::shared_ptr<PlanInstance> instance, int robot_id);

    PRM(std::shared_ptr<PlanInstance> instance, int robot_id, std::shared_ptr<RoadMap> roadmap);

    PRM(std::shared_ptr<PlanInstance> instance, int robot_id, std::shared_ptr<RoadMap> roadmap, std::shared_ptr<VoxelGrid> voxel_grid);

    virtual bool init(const PlannerOptions &options) override;

    virtual bool plan(const PlannerOptions &options) override;

    virtual bool plan(const PlannerOptions &options, double &lower_bound) override;

    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions) override;

    virtual bool plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound) override;

    virtual bool getPlan(RobotTrajectory &solution) const override;

    virtual double getPlanCost() const override;

    virtual bool terminate(const PlannerOptions &options) override;

    virtual void reSampling(const PlannerOptions &options) override;

    virtual void applyDenseMap() override;

    virtual void swapStartGoal() override;

    void sampleStart(std::shared_ptr<Vertex> &new_start);

    void sampleGoal(std::shared_ptr<Vertex> &new_goal);

    bool sampleConditionally(std::shared_ptr<Vertex> &new_sample);

    void buildRoadmap(const PlannerOptions &options);

    bool updateRoadmap(const PlannerOptions &options);

    void computeHeuristic(const PlannerOptions &options);

    bool searchPath(const PlannerOptions &options);

    bool validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b);

    bool startAndGoalConnected();

    std::shared_ptr<Graph> queryRoadmap();

    bool checkConstraints(const AStarNode &current, const PlannerOptions &options, bool isMakeSpan);

    bool checkExactConstraints(const AStarNode &current, const AStarNode &neighbor, const PlannerOptions &options, bool isMakeSpan);

    bool checkIsMakeSpan(const PlannerOptions &options);

    int checkNumConflicts(const AStarNode &current, const AStarNode &neighbor, double tentative_g, const PlannerOptions &options);

    void addConstraints(PlannerOptions &options, const Conflict &conflict);  

    void findCostRange(const PlannerOptions &options, double &minCost, double &maxCost, double &staticCostThresh);

    double computeEdgeWorkspaceProximityHeuristic(const AStarNode &current,
                                               const AStarNode &neighbor,
                                               const PlannerOptions &options);

    double computeEdgeJointSpaceHeuristic(const AStarNode &current,
                                       const AStarNode &neighbor,
                                       const PlannerOptions &options);

    Eigen::Vector3d getEndEffectorPositionFromPose(const RobotPose &pose) const;
    void logSearchStats() const;

private:
    int num_samples_;
    double max_dist_;
    std::shared_ptr<VoxelGrid> voxel_grid_;
    std::shared_ptr<Graph> roadmap_;
    int start_id_;
    int goal_id_;
    RobotTrajectory solution_;
    std::unordered_map<int, double> heuristic_;
    MRTrajectory other_solutions_;
    double max_planning_time_ = 10.0;
    double epsilon_ = 0.0;
    double w_ = 3.0;
    double lower_bound_ = 0.0;
    std::shared_ptr<RoadMap> roadmap_obj_;
    bool swapped_ = false;
    bool terminated_ = false;
    int num_col_checks_ = 0;
    bool dense_roadmap_ = false;
    int low_level_expansions_ = 0;
    std::chrono::duration<double> focal_build_time_{0};
    std::chrono::duration<double> collision_check_time_{0};
    std::chrono::duration<double> low_level_time_{0};
    std::chrono::duration<double> rrt_sample_time_{0};

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    std::mt19937_64 salt_rng_;
    bool salt_rng_seeded_ = false;


    void seedSaltRng(const PlannerOptions &options);
    std::uint64_t nextSalt();
};


#endif // PRM_H
