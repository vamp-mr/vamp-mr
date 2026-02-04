#ifndef MR_PLANNER_COMPOSITE_RRT_H
#define MR_PLANNER_COMPOSITE_RRT_H

#include "mr_planner/planning/planner.h"
#include "mr_planner/planning/nn_kdtree.h"
#include <chrono>
#include <memory>
#include <random>
#include <utility>
#include <vector>

class CompositeRRTCPlanner : public AbstractPlanner {
public:
    CompositeRRTCPlanner(std::shared_ptr<PlanInstance> instance);

    bool plan(const PlannerOptions &options) override;
    bool getPlan(MRTrajectory &solution) const override;
    void setSeed(int seed);

private:
    struct CompositeVertex {
        explicit CompositeVertex(std::vector<RobotPose> poses_in)
            : poses(std::move(poses_in)) {}

        std::vector<RobotPose> poses;
        double cost{0.0};
        std::shared_ptr<CompositeVertex> parent;
    };
    using CompositeVertexPtr = std::shared_ptr<CompositeVertex>;

    bool initialize(const PlannerOptions &options);
    bool randomCompositeSample(std::vector<RobotPose> &sample) const;
    CompositeVertexPtr nearest(const std::vector<CompositeVertexPtr> &tree,
                               const std::vector<RobotPose> &sample) const;
    CompositeVertexPtr steer(const CompositeVertexPtr &from,
                             const std::vector<RobotPose> &target) const;
    bool validateConfiguration(const std::vector<RobotPose> &config) const;
    bool validateMotion(const CompositeVertexPtr &from,
                        const CompositeVertexPtr &to,
                        const PlannerOptions &options) const;
    GrowState connectTree(std::vector<CompositeVertexPtr> &tree,
                          const CompositeVertexPtr &target,
                          const PlannerOptions &options,
                          CompositeVertexPtr &last_added);
    bool posesEqual(const std::vector<RobotPose> &a,
                    const std::vector<RobotPose> &b) const;
    double compositeDistance(const std::vector<RobotPose> &a,
                             const std::vector<RobotPose> &b) const;
    std::vector<std::vector<RobotPose>> traceToRoot(const CompositeVertexPtr &leaf) const;
    bool buildSolution(const CompositeVertexPtr &start_meeting,
                       const CompositeVertexPtr &goal_meeting,
                       const PlannerOptions &options);
    bool timeExceeded() const;

    MRTrajectory solution_;
    bool solved{false};

    double step_size_{2.0};
    double max_planning_time_{2.0};
    int max_iterations_{10000};
    double distance_tolerance_{1e-4};
    double goal_bias_{0.05};

    std::vector<RobotPose> start_config_;
    std::vector<RobotPose> goal_config_;
    std::vector<CompositeVertexPtr> start_tree_;
    std::vector<CompositeVertexPtr> goal_tree_;

    mr_planner::planning::PoseKDTreeIndex<CompositeVertexPtr> start_nn_;
    mr_planner::planning::PoseKDTreeIndex<CompositeVertexPtr> goal_nn_;
    std::vector<std::size_t> composite_dofs_;
    std::vector<std::size_t> composite_offsets_;
    std::size_t composite_dims_{0};
    mutable std::vector<float> composite_query_;
    std::vector<float> composite_insert_;

    mutable std::mt19937 rng_{std::random_device{}()};
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

#endif // MR_PLANNER_COMPOSITE_RRT_H
