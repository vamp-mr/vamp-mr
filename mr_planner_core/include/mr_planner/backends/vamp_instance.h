#ifndef VAMP_INSTANCE_H
#define VAMP_INSTANCE_H

// Opt-in macro that keeps only declarations when including this header. This reduces
// compile times for translation units that rely on the explicit instantiations defined
// in src/vamp_instance.cpp. Define MR_PLANNER_VAMP_INSTANCE_FORCE_HEADER_ONLY to
// restore the older header-only behavior when a TU needs to instantiate new variants.
#if defined(MR_PLANNER_VAMP_INSTANCE_FORCE_HEADER_ONLY)
#define MR_PLANNER_VAMP_INSTANCE_INCLUDE_IMPL 1
#elif defined(MR_PLANNER_VAMP_INSTANCE_DECL_ONLY)
#define MR_PLANNER_VAMP_INSTANCE_INCLUDE_IMPL 0
#else
#define MR_PLANNER_VAMP_INSTANCE_INCLUDE_IMPL 1
#endif

#if !MR_PLANNER_WITH_VAMP
#error "vamp_instance.h included without VAMP support"
#endif

#include <mr_planner/core/instance.h>

#include <vamp/collision/environment.hh>
#include <vamp/collision/attachments.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/multi_robot.hh>
#include <vamp/robots/link_mapping.hh>
#include <vamp/vector.hh>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <new>
#include <random>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

struct MeshcatVisualizerOptions
{
    std::string host{"127.0.0.1"};
    std::uint16_t port{7600};
    bool auto_flush{false};
};

template <typename... RobotTs>
class VampInstance : public PlanInstance {
    static_assert(sizeof...(RobotTs) > 0, "VampInstance requires at least one robot type");
    static_assert(sizeof...(RobotTs) < (sizeof(std::size_t) * 8),
                  "VampInstance requires support for mask storage in std::size_t");

public:
    using RobotTuple = std::tuple<RobotTs...>;
    static constexpr std::size_t kRobotCount = sizeof...(RobotTs);
    static constexpr std::size_t kRake = vamp::FloatVectorWidth;

    VampInstance();
    ~VampInstance();

    void setNumberOfRobots(int num_robots) override;
    void setRobotNames(const std::vector<std::string> &robot_names) override;
    void setRobotDOF(int robot_id, size_t dof) override;
    void setRobotBaseTransform(int robot_id, const Eigen::Isometry3d &transform) override;
    void setVisualizationInstance(const std::shared_ptr<PlanInstance> &instance) override;
    void enableMeshcat(const std::string &host = "127.0.0.1", std::uint16_t port = 7600);
    void disableMeshcat();
    void setMeshcatAutoFlush(bool enabled) { meshcat_options_.auto_flush = enabled; }

    template <typename DataT>
    void setEnvironment(const vamp::collision::Environment<DataT> &environment);
    void setEnvironment(const vamp::collision::Environment<float> &environment);

    const vamp::collision::Environment<vamp::FloatVector<kRake>> &environment() const;

    void clearRobotAttachments();
    void setRobotAttachment(int robot_id, const vamp::collision::Attachment<float> &attachment);
    void attachObjectToRobot(const std::string &name,
                             int robot_id,
                             const std::string &link_name,
                             const RobotPose &pose) override;
    void detachObjectFromRobot(const std::string &name, const RobotPose &pose) override;
    void setObjectColor(const std::string &name, double r, double g, double b, double a) override;

    void moveRobot(int robot_id, const RobotPose &pose) override;
    void pushScene() override {}
    void popScene(bool /*apply_to_sim*/ = true) override {}
    void plotEE(const RobotPose &/*pose*/, int /*marker_id*/) override {}
    void setPadding(double /*padding*/) override {}
    void setRandomSeed(unsigned int seed) override;
    std::vector<LinkCollision> debugCollidingLinks(const std::vector<RobotPose> &poses, bool self) override;

    void updateScene() override;
    void resetScene(bool reset_sim) override;

    // Returns true when any robot in the provided configuration collides; false when collision free.
    bool checkCollision(const std::vector<RobotPose> &poses, bool self, bool debug=false) override;

    // Returns true as soon as any waypoint sampled along the motion is invalid (in collision);
    // false indicates the entire sampled motion is collision free.
    bool checkMultiRobotMotion(const std::vector<RobotPose> &start,
                               const std::vector<RobotPose> &goal,
                               double step_size,
                               bool self=false) override;
    bool checkMultiRobotTrajectory(const MRTrajectory &trajectory,
                                   bool self=false) override;
    bool checkMultiRobotSweep(const MRTrajectory &trajectory,
                              bool self=true) override;
    bool setCollision(const std::string &obj_name, const std::string &link_name, bool allow) override;

    double computeDistance(const RobotPose &a, const RobotPose &b) const override;
    double computeDistance(const RobotPose &a, const RobotPose &b, int dim) const override;
    bool connect(const RobotPose &a, const RobotPose &b, double col_step_size = 0.1, bool debug=false) override;
    bool steer(const RobotPose &a,
               const RobotPose &b,
               double max_dist,
               RobotPose &result,
               double col_step_size = 0.1) override;
    bool sample(RobotPose &pose) override;
    RobotPose interpolate(const RobotPose &a, const RobotPose &b, double t) const override;
    double interpolate(const RobotPose &a, const RobotPose &b, double t, int dim) const override;

    void addMoveableObject(const Object &obj) override;
    void moveObject(const Object &obj) override;
    void removeObject(const std::string &name) override;
    Eigen::Vector3d getEndEffectorPositionFromPose(const RobotPose &pose) const override;
    void printKnownObjects() const override;


private:
    static constexpr std::size_t kMaxSubsetSize = (kRobotCount < 2U) ? kRobotCount : static_cast<std::size_t>(2);

    template <std::size_t... Indices, typename Func>
    static constexpr void for_each_index(std::index_sequence<Indices...>, Func &&func)
    {
        (func(std::integral_constant<std::size_t, Indices>{}), ...);
    }

    template <std::size_t Index>
    using RobotAt = std::tuple_element_t<Index, RobotTuple>;

    using PoseArray = std::array<const RobotPose *, kRobotCount>;

    using SubsetCollisionFn = bool (*)(const VampInstance &, const PoseArray &, bool);
    using SubsetMotionFn = bool (*)(VampInstance &, const PoseArray &, const PoseArray &, double, bool);
    using SubsetTrajectoryFn = bool (*)(VampInstance &,
                                        const std::array<const RobotTrajectory *, kRobotCount> &,
                                        std::size_t,
                                        std::size_t,
                                        bool,
                                        bool);

    using ComputeDistanceFn = double (*)(const RobotPose &, const RobotPose &);
    using ComputeDistanceDimFn = double (*)(const RobotPose &, const RobotPose &, int);
    using SampleFn = bool (*)(VampInstance &, RobotPose &);
    using InterpolateFn = RobotPose (*)(const RobotPose &, const RobotPose &, double);
    using InterpolateDimFn = double (*)(const RobotPose &, const RobotPose &, double, int);
    using EndEffectorFn = Eigen::Vector3d (*)(const VampInstance &, const RobotPose &);
    using EndEffectorTfFn = Eigen::Isometry3f (*)(const VampInstance &, const RobotPose &);

    template <typename Robot>
    static void ensurePoseDimension(const RobotPose &pose);

    template <typename Robot>
    static auto configurationBlockFromPose(const RobotPose &pose)
        -> typename Robot::template ConfigurationBlock<kRake>;

    PoseArray gatherPoses(const std::vector<RobotPose> &poses, bool allow_partial) const;

    template <std::size_t Index>
    const vamp::collision::Attachment<float> *attachmentForRobot() const;

    template <std::size_t Index>
    auto makeState(const RobotPose &pose) const
        -> vamp::collision::MultiRobotState<RobotAt<Index>, kRake>;

    template <std::size_t Index>
    Eigen::Vector3d endEffectorPositionForRobot(const RobotPose &pose) const;
    template <std::size_t Index>
    Eigen::Isometry3f endEffectorTransformForRobot(const RobotPose &pose) const;
    Eigen::Isometry3f endEffectorTransform(std::size_t robot_index, const RobotPose &pose) const;

    static auto makeAttachmentFromObject(const Object &obj)
        -> std::optional<vamp::collision::Attachment<float>>;
    static Eigen::Isometry3f attachmentTransformFromObject(const Object &obj);
    static Eigen::Isometry3f objectPoseToIsometry(const Object &obj);

    // Helper that evaluates the selected robots in SIMD and returns true if any collision is found.
    template <std::size_t... I>
    bool checkCollisionPack(const PoseArray &poses, bool self, std::index_sequence<I...>);

    std::string robotNameForIndex(std::size_t robot_index) const;

    bool attachmentAllowsObject(const std::unordered_set<std::string> &allowed,
                                const std::string &other_robot_name,
                                const std::string &other_link_name) const;

    template <std::size_t AttachmentRobotIndex, std::size_t OtherRobotIndex, typename OtherSpheres>
    bool attachmentRobotCollidesWithRobotSpheres(
        const std::vector<vamp::collision::Sphere<vamp::FloatVector<kRake>>> &attachment_spheres,
        const OtherSpheres &other_spheres) const;

    template <std::size_t... I>
    bool checkCollisionPackWithAttachmentAllowances(
        const PoseArray &poses,
        const vamp::collision::MultiRobotCollisionFilter *filter,
        std::index_sequence<I...>) const;

    template <std::size_t... I>
    bool fkccMultiAllWithAttachmentAllowances(
        const vamp::collision::MultiRobotCollisionFilter *filter,
        std::index_sequence<I...>,
        const vamp::collision::MultiRobotState<RobotAt<I>, kRake> &... states) const;

    template <std::size_t... I>
    bool subsetCollisionImpl(const PoseArray &poses, bool self) const;

    template <std::size_t... I>
    void initializeDofs(std::index_sequence<I...>);

    template <std::size_t RobotIndex>
    struct MotionCacheEntry
    {
        using Robot = RobotAt<RobotIndex>;
        // SIMD-packed configuration samples evaluated in fkcc_multi_*.
        typename Robot::template ConfigurationBlock<kRake> block{};
        // Per-joint scalar step used to rewind the block between rake iterations.
        std::vector<float> backstep{};
    };

    template <std::size_t RobotIndex>
    struct TrajectoryCacheEntry
    {
        using Robot = RobotAt<RobotIndex>;
        typename Robot::template ConfigurationBlock<kRake> block{};
    };

    // Returns true if any lane across the SIMD “rake” detects a collision along the motion.
    template <std::size_t... I>
    bool checkMotionPack(const PoseArray &start,
                         const PoseArray &goal,
                         double step_size,
                         bool self,
                         std::index_sequence<I...>);

    template <std::size_t... I>
    bool checkTrajectoryPack(const std::array<const RobotTrajectory *, kRobotCount> &trajectories,
                             std::size_t offset,
                             std::size_t lane_count,
                             bool self,
                             bool cross,
                             std::index_sequence<I...>);

    bool subsetTrajectorySwitch(const std::array<const RobotTrajectory *, kRobotCount> &trajectories,
                                const std::vector<std::size_t> &active,
                                std::size_t offset,
                                std::size_t lane_count,
                                bool self,
                                bool cross);

    template <std::size_t... I>
    bool setCollisionDispatch(const std::string &object, const std::string &link, bool allow, std::index_sequence<I...>);

    template <std::size_t Index>
    bool setCollisionForRobot(const std::string &object, const std::string &link, bool allow);

    template <typename Robot>
    static bool matchLinkName(const std::string &input, const std::string &robot_name, std::string &output);

    template <std::size_t Index>
    static double computeDistanceThunk(const RobotPose &a, const RobotPose &b)
    {
        return computeDistanceImpl<RobotAt<Index>>(a, b);
    }

    template <std::size_t... I>
    static auto makeComputeDistanceDispatchImpl(std::index_sequence<I...>)
        -> std::array<ComputeDistanceFn, sizeof...(I)>
    {
        return std::array<ComputeDistanceFn, sizeof...(I)>{{&computeDistanceThunk<I>...}};
    }

    template <std::size_t Index>
    static double computeDistanceDimThunk(const RobotPose &a, const RobotPose &b, int dim)
    {
        return computeDistanceDimImpl<RobotAt<Index>>(a, b, dim);
    }

    template <std::size_t... I>
    static auto makeComputeDistanceDimDispatchImpl(std::index_sequence<I...>)
        -> std::array<ComputeDistanceDimFn, sizeof...(I)>
    {
        return std::array<ComputeDistanceDimFn, sizeof...(I)>{{&computeDistanceDimThunk<I>...}};
    }

    template <std::size_t Index>
    static bool sampleThunk(VampInstance &instance, RobotPose &pose)
    {
        return instance.template sampleImpl<RobotAt<Index>>(pose);
    }

    template <std::size_t... I>
    static auto makeSampleDispatchImpl(std::index_sequence<I...>)
        -> std::array<SampleFn, sizeof...(I)>
    {
        return std::array<SampleFn, sizeof...(I)>{{&sampleThunk<I>...}};
    }

    template <std::size_t Index>
    static RobotPose interpolateThunk(const RobotPose &a, const RobotPose &b, double t)
    {
        return interpolateImpl<RobotAt<Index>>(a, b, t);
    }

    template <std::size_t... I>
    static auto makeInterpolateDispatchImpl(std::index_sequence<I...>)
        -> std::array<InterpolateFn, sizeof...(I)>
    {
        return std::array<InterpolateFn, sizeof...(I)>{{&interpolateThunk<I>...}};
    }

    template <std::size_t Index>
    static double interpolateDimThunk(const RobotPose &a, const RobotPose &b, double t, int dim)
    {
        return interpolateDimImpl<RobotAt<Index>>(a, b, t, dim);
    }

    template <std::size_t... I>
    static auto makeInterpolateDimDispatchImpl(std::index_sequence<I...>)
        -> std::array<InterpolateDimFn, sizeof...(I)>
    {
        return std::array<InterpolateDimFn, sizeof...(I)>{{&interpolateDimThunk<I>...}};
    }

    template <std::size_t Index>
    static Eigen::Vector3d endEffectorThunk(const VampInstance &instance, const RobotPose &pose)
    {
        return instance.template endEffectorPositionForRobot<Index>(pose);
    }

    template <std::size_t... I>
    static auto makeEndEffectorDispatchImpl(std::index_sequence<I...>)
        -> std::array<EndEffectorFn, sizeof...(I)>
    {
        return std::array<EndEffectorFn, sizeof...(I)>{{&endEffectorThunk<I>...}};
    }

    template <std::size_t Index>
    static Eigen::Isometry3f endEffectorTransformThunk(const VampInstance &instance, const RobotPose &pose)
    {
        return instance.template endEffectorTransformForRobot<Index>(pose);
    }

    template <std::size_t... I>
    static auto makeEndEffectorTransformDispatchImpl(std::index_sequence<I...>)
        -> std::array<EndEffectorTfFn, sizeof...(I)>
    {
        return std::array<EndEffectorTfFn, sizeof...(I)>{{&endEffectorTransformThunk<I>...}};
    }

    template <std::size_t Index>
    static bool subsetCollisionSingleCaller(const VampInstance &instance,
                                            const PoseArray &poses,
                                            bool self)
    {
        return instance.subsetCollisionImpl<Index>(poses, self);
    }

    template <std::size_t A, std::size_t B>
    static bool subsetCollisionPairCaller(const VampInstance &instance,
                                          const PoseArray &poses,
                                          bool self)
    {
        return instance.subsetCollisionImpl<A, B>(poses, self);
    }

    template <std::size_t Index>
    static bool subsetMotionSingleCaller(VampInstance &instance,
                                         const PoseArray &start,
                                         const PoseArray &goal,
                                         double step_size,
                                         bool self)
    {
        return instance.checkMotionPack(start, goal, step_size, self, std::index_sequence<Index>{});
    }

    template <std::size_t A, std::size_t B>
    static bool subsetMotionPairCaller(VampInstance &instance,
                                       const PoseArray &start,
                                       const PoseArray &goal,
                                       double step_size,
                                       bool self)
    {
        return instance.checkMotionPack(start, goal, step_size, self, std::index_sequence<A, B>{});
    }

    template <std::size_t Index>
    static bool subsetTrajectorySingleCaller(VampInstance &instance,
                                             const std::array<const RobotTrajectory *, kRobotCount> &trajectories,
                                             std::size_t offset,
                                             std::size_t lane_count,
                                             bool self,
                                             bool cross)
    {
        return instance.checkTrajectoryPack(trajectories, offset, lane_count, self, cross, std::index_sequence<Index>{});
    }

    template <std::size_t A, std::size_t B>
    static bool subsetTrajectoryPairCaller(VampInstance &instance,
                                           const std::array<const RobotTrajectory *, kRobotCount> &trajectories,
                                           std::size_t offset,
                                           std::size_t lane_count,
                                           bool self,
                                           bool cross)
    {
        return instance.checkTrajectoryPack(trajectories, offset, lane_count, self, cross, std::index_sequence<A, B>{});
    }

    struct CollisionSingleFunctor
    {
        template <std::size_t Index>
        constexpr SubsetCollisionFn operator()() const
        {
            if constexpr (Index < kRobotCount)
            {
                return &subsetCollisionSingleCaller<Index>;
            }
            else
            {
                return nullptr;
            }
        }
    };

    struct CollisionPairFunctor
    {
        template <std::size_t A, std::size_t B>
        constexpr SubsetCollisionFn operator()() const
        {
            if constexpr (A < B && B < kRobotCount)
            {
                return &subsetCollisionPairCaller<A, B>;
            }
            else
            {
                return nullptr;
            }
        }
    };

    struct MotionSingleFunctor
    {
        template <std::size_t Index>
        constexpr SubsetMotionFn operator()() const
        {
            if constexpr (Index < kRobotCount)
            {
                return &subsetMotionSingleCaller<Index>;
            }
            else
            {
                return nullptr;
            }
        }
    };

    struct MotionPairFunctor
    {
        template <std::size_t A, std::size_t B>
        constexpr SubsetMotionFn operator()() const
        {
            if constexpr (A < B && B < kRobotCount)
            {
                return &subsetMotionPairCaller<A, B>;
            }
            else
            {
                return nullptr;
            }
        }
    };

    struct TrajectorySingleFunctor
    {
        template <std::size_t Index>
        constexpr SubsetTrajectoryFn operator()() const
        {
            if constexpr (Index < kRobotCount)
            {
                return &subsetTrajectorySingleCaller<Index>;
            }
            else
            {
                return nullptr;
            }
        }
    };

    struct TrajectoryPairFunctor
    {
        template <std::size_t A, std::size_t B>
        constexpr SubsetTrajectoryFn operator()() const
        {
            if constexpr (A < B && B < kRobotCount)
            {
                return &subsetTrajectoryPairCaller<A, B>;
            }
            else
            {
                return nullptr;
            }
        }
    };

    template <typename Fn, typename IndexFunctor>
    static auto makeSingleSubsetDispatch(IndexFunctor index_fn)
        -> std::array<Fn, kRobotCount>
    {
        std::array<Fn, kRobotCount> dispatch{};
        // Compile-time loop that instantiates one function pointer per robot index.
        // We avoid a runtime switch and still generate the right templated caller.
        for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto index_c) {
            constexpr std::size_t idx = index_c;
            dispatch[idx] = index_fn.template operator()<idx>();
        });
        return dispatch;
    }

    template <typename Fn, typename IndexFunctor>
    static auto makePairSubsetDispatch(IndexFunctor index_fn)
        -> std::array<std::array<Fn, kRobotCount>, kRobotCount>
    {
        std::array<std::array<Fn, kRobotCount>, kRobotCount> dispatch{};
        // Compile-time double loop that fills the symmetric pair-dispatch table.
        // Generates the correct templated function for each ordered pair without
        // a runtime switch, then mirrors it across [I][J] and [J][I].
        for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto i_c) {
            constexpr std::size_t I = i_c;
            for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto j_c) {
                constexpr std::size_t J = j_c;
                if constexpr (I < J && kMaxSubsetSize >= 2)
                {
                    const auto fn = index_fn.template operator()<I, J>();
                    dispatch[I][J] = fn;
                    dispatch[J][I] = fn;
                }
                else
                {
                    dispatch[I][J] = nullptr;
                }
            });
        });
        return dispatch;
    }

    template <std::size_t... I>
    static vamp::collision::MultiRobotCollisionFilter makeSubsetFilter(
        const vamp::collision::MultiRobotCollisionFilter &src)
    {
        vamp::collision::MultiRobotCollisionFilter dst;
        dst.per_robot.reserve(sizeof...(I));
        (dst.per_robot.push_back(
             (I < src.per_robot.size()) ?
                 src.per_robot[I] :
                 vamp::collision::MultiRobotCollisionFilter::RobotFilter{}),
         ...);
        return dst;
    }

    static inline const auto subset_collision_single_dispatch_ =
        makeSingleSubsetDispatch<SubsetCollisionFn>(CollisionSingleFunctor{});
    static inline const auto subset_collision_pair_dispatch_ =
        makePairSubsetDispatch<SubsetCollisionFn>(CollisionPairFunctor{});

    static inline const auto subset_motion_single_dispatch_ =
        makeSingleSubsetDispatch<SubsetMotionFn>(MotionSingleFunctor{});
    static inline const auto subset_motion_pair_dispatch_ =
        makePairSubsetDispatch<SubsetMotionFn>(MotionPairFunctor{});

    static inline const auto subset_trajectory_single_dispatch_ =
        makeSingleSubsetDispatch<SubsetTrajectoryFn>(TrajectorySingleFunctor{});
    static inline const auto subset_trajectory_pair_dispatch_ =
        makePairSubsetDispatch<SubsetTrajectoryFn>(TrajectoryPairFunctor{});

    static inline const auto compute_distance_dispatch_ =
        makeComputeDistanceDispatchImpl(std::make_index_sequence<kRobotCount>{});
    static inline const auto compute_distance_dim_dispatch_ =
        makeComputeDistanceDimDispatchImpl(std::make_index_sequence<kRobotCount>{});
    static inline const auto sample_dispatch_ =
        makeSampleDispatchImpl(std::make_index_sequence<kRobotCount>{});
    static inline const auto interpolate_dispatch_ =
        makeInterpolateDispatchImpl(std::make_index_sequence<kRobotCount>{});
    static inline const auto interpolate_dim_dispatch_ =
        makeInterpolateDimDispatchImpl(std::make_index_sequence<kRobotCount>{});
    static inline const auto end_effector_dispatch_ =
        makeEndEffectorDispatchImpl(std::make_index_sequence<kRobotCount>{});
    static inline const auto end_effector_tf_dispatch_ =
        makeEndEffectorTransformDispatchImpl(std::make_index_sequence<kRobotCount>{});

    void rebuildCollisionFilter();

    template <std::size_t... I>
    void rebuildCollisionFilterImpl(std::index_sequence<I...>);

    template <std::size_t Index>
    void rebuildCollisionFilterForRobot();

    bool subsetCollisionSwitch(const PoseArray &poses, const std::vector<std::size_t> &active, bool self) const;
    // Dispatches to the appropriate SIMD path and returns true if any waypoint collides.
    bool subsetMotionSwitch(const PoseArray &start,
                            const PoseArray &goal,
                            const std::vector<std::size_t> &active,
                            double step_size,
                            bool self);

    template <typename Robot>
    static double computeDistanceImpl(const RobotPose &a, const RobotPose &b);

    template <typename Robot>
    static double computeDistanceDimImpl(const RobotPose &a, const RobotPose &b, int dim);

    template <typename Robot>
    bool sampleImpl(RobotPose &pose);

    template <typename Robot>
    static RobotPose interpolateImpl(const RobotPose &a, const RobotPose &b, double t);

    template <typename Robot>
    static double interpolateDimImpl(const RobotPose &a, const RobotPose &b, double t, int dim);

    static Eigen::Quaternionf quaternionFromObject(const Object &obj);
    static vamp::collision::Cuboid<float> makeCuboidShape(const Object &obj);
    static vamp::collision::Cylinder<float> makeCylinderShape(const Object &obj);
    void rebuildEnvironment();

    // Meshcat helpers
    template <std::size_t Index>
    void appendRobotSpheres(Json::Value &out) const;
    Eigen::Isometry3f poseForObject(const Object &obj) const;
    void publishMeshcatScene();
    void resetMeshcatScene(bool reset_sim);
    void sendMeshcatJson(const Json::Value &msg);
    void enqueueMeshcatPayload(const std::string &payload, bool is_reset);
    void startMeshcatWorker();
    void stopMeshcatWorker();
    void runMeshcatWorker();
    void connectMeshcat();
    bool ensureMeshcatConnected();
    template <typename Robot>
    static std::string linkNameForSphere(std::size_t sphere_index);
    static Json::Value toJsonArray(const Eigen::Vector3f &vec);
    static Json::Value toJsonArray(const std::array<double, 4> &rgba);
    static std::array<double, 4> colorFromPalette(std::size_t index);

    std::array<Eigen::Isometry3f, kRobotCount> base_transforms_{};
    std::unique_ptr<vamp::collision::Environment<vamp::FloatVector<kRake>>> environment_static_;
    vamp::collision::Environment<vamp::FloatVector<kRake>> environment_{};
    vamp::collision::Environment<float> environment_input_{};
    std::array<std::optional<vamp::collision::Attachment<float>>, kRobotCount> attachments_{};
    std::shared_ptr<PlanInstance> visualization_instance_;
    bool meshcat_enabled_{false};
    MeshcatVisualizerOptions meshcat_options_{};
    bool meshcat_dirty_{false};
    bool meshcat_connected_{false};
    bool meshcat_debug_{false};
    boost::asio::io_context meshcat_io_ctx_{};
    boost::asio::ip::tcp::socket meshcat_socket_{meshcat_io_ctx_};
    Json::StreamWriterBuilder meshcat_writer_builder_;
    std::vector<std::array<double, 4>> meshcat_robot_colors_;
    std::unordered_map<std::string, std::array<double, 4>> meshcat_object_colors_;
    bool meshcat_needs_full_scene_{true};
    std::unordered_set<std::string> meshcat_dirty_objects_;
    std::unordered_set<std::string> meshcat_deleted_objects_;
    std::thread meshcat_thread_;
    std::mutex meshcat_mutex_;
    std::condition_variable meshcat_cv_;
    std::deque<std::string> meshcat_queue_;
    bool meshcat_queue_has_reset_{false};
    bool meshcat_worker_running_{false};
    std::string meshcat_last_payload_;
    std::unordered_set<std::string> movable_objects_;
    std::vector<RobotPose> robot_shadow_;
    mutable std::mutex objects_mutex_;
    std::mt19937 rng_{};

    using LinkAllowances = std::unordered_map<std::string, std::unordered_set<std::string>>;
    std::array<LinkAllowances, kRobotCount> link_collision_overrides_{};
    std::array<std::unordered_set<std::string>, kRobotCount> attachment_collision_overrides_{};
    vamp::collision::MultiRobotCollisionFilter collision_filter_{};
};

// ---- Implementation ----

#if MR_PLANNER_VAMP_INSTANCE_INCLUDE_IMPL

template <typename... RobotTs>
VampInstance<RobotTs...>::VampInstance()
{
    std::random_device rd;
    rng_ = std::mt19937(rd());
    for (auto &tf : base_transforms_)
    {
        tf = Eigen::Isometry3f::Identity();
    }
    meshcat_writer_builder_["indentation"] = "";
    meshcat_robot_colors_.assign(kRobotCount, {});
    for (std::size_t i = 0; i < kRobotCount; ++i)
    {
        meshcat_robot_colors_[i] = colorFromPalette(i);
    }
    const char *debug_env = std::getenv("MR_PLANNER_MESHCAT_DEBUG");
    meshcat_debug_ = (debug_env != nullptr && std::string(debug_env) == "1");
    meshcat_dirty_ = true;
    instance_type_ = "VampInstance";
}

template <typename... RobotTs>
VampInstance<RobotTs...>::~VampInstance()
{
    stopMeshcatWorker();
    if (meshcat_socket_.is_open())
    {
        boost::system::error_code ec;
        meshcat_socket_.close(ec);
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setRandomSeed(unsigned int seed)
{
    rng_.seed(static_cast<std::mt19937::result_type>(seed));
}

template <typename... RobotTs>
std::vector<LinkCollision> VampInstance<RobotTs...>::debugCollidingLinks(const std::vector<RobotPose> &poses,
                                                                         bool self)
{
    std::vector<LinkCollision> result;
    if (poses.empty())
    {
        return result;
    }

    const auto gathered = gatherPoses(poses, true);

    std::vector<std::size_t> active;
    active.reserve(kRobotCount);
    for (std::size_t idx = 0; idx < kRobotCount; ++idx)
    {
        if (gathered[idx] != nullptr)
        {
            active.push_back(idx);
        }
    }

    if (active.empty())
    {
        return result;
    }

    // Map local indices (order of states passed to VAMP) back to global robot ids.
    std::vector<int> local_to_global;
    local_to_global.reserve(active.size());
    for (auto idx : active)
    {
        local_to_global.push_back(static_cast<int>(idx));
    }

    auto remap_index = [&](int local_idx) -> int {
        return local_to_global[static_cast<std::size_t>(local_idx)];
    };

    // Build a remapped collision filter when operating on subsets so per-robot
    // allowances stay aligned with the local ordering of states handed to VAMP.
    std::optional<vamp::collision::MultiRobotCollisionFilter> local_filter;
    const vamp::collision::MultiRobotCollisionFilter *filter_ptr = nullptr;
    if (!collision_filter_.empty())
    {
        if (active.size() == kRobotCount)
        {
            filter_ptr = &collision_filter_;
        }
        else
        {
            local_filter.emplace();
            local_filter->per_robot.resize(active.size());
            for (std::size_t local_idx = 0; local_idx < active.size(); ++local_idx)
            {
                const auto global_idx = active[local_idx];
                if (global_idx < collision_filter_.per_robot.size())
                {
                    local_filter->per_robot[local_idx] = collision_filter_.per_robot[global_idx];
                }
            }
            filter_ptr = &(*local_filter);
        }
    }

    auto process_debug_result = [&](const auto &debug_result) {
        auto to_double = [](const std::vector<float> &v, double fallback = 0.0) {
            return v.empty() ? fallback : static_cast<double>(v.front());
        };

        auto shape_from_tag = [](const std::string &tag) -> Object::Shape {
            if (tag == "sphere")
            {
                return Object::Sphere;
            }
            if (tag == "cylinder" || tag == "capsule")
            {
                return Object::Cylinder;
            }
            if (tag == "box" || tag == "cuboid")
            {
                return Object::Box;
            }
            return Object::Mesh;
        };

        auto make_object = [&](const std::string &name, int robot_idx, const auto &pose) {
            Object obj;
            obj.name = name;
            obj.parent_link = name;
            obj.robot_id = robot_idx;
            obj.state = Object::Static;
            obj.shape = shape_from_tag(pose.shape);

            obj.x = to_double(pose.x);
            obj.y = to_double(pose.y);
            obj.z = to_double(pose.z);
            obj.qx = to_double(pose.qx);
            obj.qy = to_double(pose.qy);
            obj.qz = to_double(pose.qz);
            obj.qw = to_double(pose.qw, 1.0);

            obj.length = to_double(pose.l);
            obj.width = to_double(pose.w);
            obj.height = to_double(pose.h);

            switch (obj.shape)
            {
            case Object::Sphere:
                obj.radius = obj.length * 0.5;
                obj.length = obj.width = obj.height = 2.0 * obj.radius;
                break;
            case Object::Cylinder:
                obj.radius = obj.width * 0.5;
                break;
            default:
                obj.radius = 0.0;
                break;
            }

            return obj;
        };

        std::unordered_set<std::string> seen_pairs;
        for (const auto &ll : debug_result.link_link)
        {
            const auto ra = remap_index(static_cast<int>(ll.first.robot_index));
            const auto rb = remap_index(static_cast<int>(ll.second.robot_index));
            const std::string name_a = (ra >= 0 && ra < static_cast<int>(robot_names_.size())) ?
                                           robot_names_[ra] :
                                           std::string("robot_") + std::to_string(ra);
            const std::string name_b = (rb >= 0 && rb < static_cast<int>(robot_names_.size())) ?
                                           robot_names_[rb] :
                                           std::string("robot_") + std::to_string(rb);

            const std::string link_a = name_a + "::" + ll.first.name;
            const std::string link_b = name_b + "::" + ll.second.name;
            const std::string key = std::to_string(ra) + "|" + link_a + "|" + std::to_string(rb) + "|" + link_b;
            if (seen_pairs.insert(key).second)
            {
                LinkCollision lc;
                lc.robot_a = ra;
                lc.robot_b = rb;
                lc.link_a = make_object(link_a, ra, ll.first_pose);
                lc.link_b = make_object(link_b, rb, ll.second_pose);
                result.push_back(std::move(lc));
            }
        }

        if (!self) {
            for (const auto &lo : debug_result.link_object)
            {
                const auto ra = remap_index(static_cast<int>(lo.link.robot_index));
                const std::string name_a = (ra >= 0 && ra < static_cast<int>(robot_names_.size())) ?
                                            robot_names_[ra] :
                                            std::string("robot_") + std::to_string(ra);
                const std::string link_a = name_a + "::" + lo.link.name;
                const std::string link_b = std::string("object::") + lo.object;
                const std::string key = std::to_string(ra) + "|" + link_a + "|-1|" + link_b;
                if (seen_pairs.insert(key).second)
                {
                    LinkCollision lc;
                    lc.robot_a = ra;
                    lc.robot_b = -1;
                    lc.link_a = make_object(link_a, ra, lo.link_pose);
                    if (lo.object_pose)
                    {
                        lc.link_b = make_object(link_b, -1, *lo.object_pose);
                    }
                    else
                    {
                        lc.link_b.name = link_b;
                        lc.link_b.parent_link = link_b;
                        lc.link_b.robot_id = -1;
                        lc.link_b.state = Object::Static;
                        lc.link_b.shape = Object::Mesh;
                        lc.link_b.x = lc.link_b.y = lc.link_b.z = 0.0;
                        lc.link_b.qx = lc.link_b.qy = lc.link_b.qz = 0.0;
                        lc.link_b.qw = 1.0;
                        lc.link_b.length = lc.link_b.width = lc.link_b.height = lc.link_b.radius = 0.0;
                    }
                    result.push_back(std::move(lc));
                }
            }
        }
    };

    if (active.size() == kRobotCount)
    {
        auto make_state = [&](auto index_c) {
            constexpr std::size_t Index = decltype(index_c)::value;
            using Robot = RobotAt<Index>;
            const auto *pose = gathered[Index];
            if (!pose)
            {
                throw std::invalid_argument("VampInstance: missing pose for robot in debugCollidingLinks");
            }
            return vamp::collision::make_multi_robot_state<Robot, kRake>(
                configurationBlockFromPose<Robot>(*pose),
                base_transforms_[Index],
                attachmentForRobot<Index>());
        };

        auto states = [&]<std::size_t... I>(std::index_sequence<I...>) {
            return std::tuple{make_state(std::integral_constant<std::size_t, I>{})...};
        }(std::make_index_sequence<kRobotCount>{});

        const auto debug_result = std::apply(
            [&](auto &&...st) {
                if (filter_ptr)
                {
                    return vamp::collision::debug_fkcc_multi_all<kRake>(environment_, *filter_ptr, st...);
                }
                return vamp::collision::debug_fkcc_multi_all<kRake>(environment_, st...);
            },
            states);
        process_debug_result(debug_result);
        return result;
    }

    if (active.size() == 1)
    {
        const std::size_t idx = active.front();
        bool handled = false;
        for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto idx_c) {
            // Compile-time loop to pick the correct robot type; only the matching
            // index executes the body while the rest return immediately.
            if (handled || idx_c.value != idx)
            {
                return;
            }
            const auto *pose = gathered[idx];
            if (!pose)
            {
                throw std::invalid_argument("VampInstance: missing pose for robot in debugCollidingLinks");
            }
            const auto state = vamp::collision::make_multi_robot_state<RobotAt<idx_c.value>, kRake>(
                configurationBlockFromPose<RobotAt<idx_c.value>>(*pose),
                base_transforms_[idx],
                attachmentForRobot<idx_c.value>());
            const auto debug_result = filter_ptr ?
                                          vamp::collision::debug_fkcc_multi_all<kRake>(environment_, *filter_ptr, state) :
                                          vamp::collision::debug_fkcc_multi_all<kRake>(environment_, state);
            process_debug_result(debug_result);
            handled = true;
        });
        if (!handled)
        {
            throw std::out_of_range("VampInstance: robot index out of range in debugCollidingLinks");
        }
        return result;
    }

    if (active.size() == 2)
    {
        const std::size_t a = active[0];
        const std::size_t b = active[1];
        bool handled = false;
        for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto a_c) {
            for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto b_c) {
                // Compile-time double loop over robot indices to instantiate the
                // correct state types; only the matching pair executes the body.
                if (handled || a_c.value != a || b_c.value != b)
                {
                    return;
                }
                const auto *pose_a = gathered[a];
                const auto *pose_b = gathered[b];
                if (!pose_a || !pose_b)
                {
                    throw std::invalid_argument("VampInstance: missing pose for robot in debugCollidingLinks");
                }
                const auto state_a = vamp::collision::make_multi_robot_state<RobotAt<a_c.value>, kRake>(
                    configurationBlockFromPose<RobotAt<a_c.value>>(*pose_a),
                    base_transforms_[a],
                    attachmentForRobot<a_c.value>());
                const auto state_b = vamp::collision::make_multi_robot_state<RobotAt<b_c.value>, kRake>(
                    configurationBlockFromPose<RobotAt<b_c.value>>(*pose_b),
                    base_transforms_[b],
                    attachmentForRobot<b_c.value>());
                const auto debug_result = filter_ptr ?
                                              vamp::collision::debug_fkcc_multi_all<kRake>(environment_, *filter_ptr, state_a, state_b) :
                                              vamp::collision::debug_fkcc_multi_all<kRake>(environment_, state_a, state_b);
                process_debug_result(debug_result);
                handled = true;
            });
        });
        if (!handled)
        {
            throw std::out_of_range("VampInstance: robot index out of range in debugCollidingLinks");
        }
        return result;
    }

    throw std::invalid_argument("VampInstance: unsupported robot subset in debugCollidingLinks");
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setNumberOfRobots(int num_robots)
{
    if (num_robots != static_cast<int>(kRobotCount))
    {
        throw std::invalid_argument("VampInstance: unexpected robot count");
    }

    PlanInstance::setNumberOfRobots(num_robots);
    initializeDofs(std::make_index_sequence<kRobotCount>{});
    clearRobotAttachments();
    robot_shadow_.assign(static_cast<std::size_t>(num_robots), RobotPose{});
    if (visualization_instance_)
    {
        visualization_instance_->setNumberOfRobots(num_robots);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_robot_colors_.size() != kRobotCount)
        {
            meshcat_robot_colors_.assign(kRobotCount, {});
            for (std::size_t i = 0; i < kRobotCount; ++i)
            {
                meshcat_robot_colors_[i] = colorFromPalette(i);
            }
        }
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setVisualizationInstance(const std::shared_ptr<PlanInstance> &instance)
{
    visualization_instance_ = instance;
    if (visualization_instance_)
    {
        // Prefer external viz when provided; disable internal meshcat unless re-enabled explicitly.
        disableMeshcat();
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setRobotNames(const std::vector<std::string> &robot_names)
{
    if (robot_names.size() != kRobotCount)
    {
        throw std::invalid_argument("VampInstance: robot name count mismatch");
    }

    PlanInstance::setRobotNames(robot_names);
    if (visualization_instance_)
    {
        visualization_instance_->setRobotNames(robot_names);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setRobotDOF(int robot_id, size_t dof)
{
    if (robot_id < 0 || robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }

    constexpr std::array<std::size_t, kRobotCount> kDims = {RobotTs::dimension...};
    const std::size_t required = kDims[static_cast<std::size_t>(robot_id)];
    std::size_t clamped = required;

    if (dof == required + 1)
    {
        clamped = required + 1;
    }

    PlanInstance::setRobotDOF(robot_id, clamped);
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setRobotBaseTransform(int robot_id, const Eigen::Isometry3d &transform)
{
    if (robot_id < 0 || robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }

    auto &dst = base_transforms_[static_cast<std::size_t>(robot_id)].matrix();
    const double *src = transform.matrix().data();
    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 4; ++row)
        {
            // Column-major storage
            dst(row, col) = static_cast<float>(src[col * 4 + row]);
        }
    }

    if (visualization_instance_)
    {
        visualization_instance_->setRobotBaseTransform(robot_id, transform);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
}

template <typename... RobotTs>
template <typename DataT>
void VampInstance<RobotTs...>::setEnvironment(const vamp::collision::Environment<DataT> &environment)
{
    environment_static_ = std::make_unique<vamp::collision::Environment<vamp::FloatVector<kRake>>>(environment);
    rebuildEnvironment();
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setEnvironment(const vamp::collision::Environment<float> &environment)
{
    environment_static_ = std::make_unique<vamp::collision::Environment<vamp::FloatVector<kRake>>>(environment);
    rebuildEnvironment();
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::moveRobot(int robot_id, const RobotPose &pose)
{
    if (robot_id < 0)
    {
        return;
    }
    const auto idx = static_cast<std::size_t>(robot_id);
    if (idx >= robot_shadow_.size())
    {
        return;
    }
    robot_shadow_[idx] = pose;
    if (visualization_instance_)
    {
        visualization_instance_->moveRobot(robot_id, pose);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::clearRobotAttachments()
{
    for (auto &attachment : attachments_)
    {
        attachment.reset();
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setRobotAttachment(int robot_id, const vamp::collision::Attachment<float> &attachment)
{
    if (robot_id < 0 || robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot_id out of range");
    }
    attachments_[static_cast<std::size_t>(robot_id)] = attachment;
}

template <typename... RobotTs>
const vamp::collision::Environment<vamp::FloatVector<VampInstance<RobotTs...>::kRake>> &
VampInstance<RobotTs...>::environment() const
{
    return environment_;
}


template <typename... RobotTs>
template <typename Robot>
void VampInstance<RobotTs...>::ensurePoseDimension(const RobotPose &pose)
{
    if (pose.joint_values.size() != Robot::dimension)
    {
        if (pose.joint_values.size() < Robot::dimension)
        {
            throw std::invalid_argument("VampInstance: pose dimension mismatch");
        }

        // Allow a single trailing configuration entry (e.g., fixed flange joint) and ignore it.
        if (pose.joint_values.size() > Robot::dimension + 1)
        {
            throw std::invalid_argument("VampInstance: pose dimension mismatch");
        }
    }
}

template <typename... RobotTs>
template <typename Robot>
auto VampInstance<RobotTs...>::configurationBlockFromPose(const RobotPose &pose)
    -> typename Robot::template ConfigurationBlock<kRake>
{
    ensurePoseDimension<Robot>(pose);

    using Block = typename Robot::template ConfigurationBlock<kRake>;
    using Row = typename Block::RowT;

    Block block = Block::zero_vector();
    for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
    {
        const float value = static_cast<float>(pose.joint_values[joint]);
        block[joint] = Row::fill(value);
    }

    return block;
}

template <typename... RobotTs>
typename VampInstance<RobotTs...>::PoseArray
VampInstance<RobotTs...>::gatherPoses(const std::vector<RobotPose> &poses, bool allow_partial) const
{
    PoseArray result{};
    result.fill(nullptr);

    for (const auto &pose : poses)
    {
        if (pose.robot_id < 0 || pose.robot_id >= static_cast<int>(kRobotCount))
        {
            throw std::out_of_range("VampInstance: robot id out of range");
        }

        auto &slot = result[static_cast<std::size_t>(pose.robot_id)];
        if (slot != nullptr)
        {
            throw std::invalid_argument("VampInstance: duplicate pose for robot id");
        }
        slot = &pose;
    }

    if (!allow_partial)
    {
        for (const auto *ptr : result)
        {
            if (ptr == nullptr)
            {
                throw std::invalid_argument("VampInstance: missing pose for robot");
            }
        }
    }

    return result;
}

template <typename... RobotTs>
template <std::size_t Index>
const vamp::collision::Attachment<float> *
VampInstance<RobotTs...>::attachmentForRobot() const
{
    if constexpr (Index < kRobotCount)
    {
        const auto &opt = attachments_[Index];
        return opt ? &opt.value() : nullptr;
    }
    else
    {
        return nullptr;
    }
}

template <typename... RobotTs>
template <std::size_t Index>
auto VampInstance<RobotTs...>::makeState(const RobotPose &pose) const
    -> vamp::collision::MultiRobotState<RobotAt<Index>, kRake>
{
    return vamp::collision::make_multi_robot_state<RobotAt<Index>, kRake>(
        configurationBlockFromPose<RobotAt<Index>>(pose),
        base_transforms_[Index],
        attachmentForRobot<Index>());
}

template <typename... RobotTs>
template <std::size_t Index>
Eigen::Vector3d VampInstance<RobotTs...>::endEffectorPositionForRobot(const RobotPose &pose) const
{
    const Eigen::Isometry3f ee = endEffectorTransformForRobot<Index>(pose);
    return ee.translation().cast<double>();
}

template <typename... RobotTs>
template <std::size_t Index>
Eigen::Isometry3f VampInstance<RobotTs...>::endEffectorTransformForRobot(const RobotPose &pose) const
{
    using Robot = RobotAt<Index>;
    ensurePoseDimension<Robot>(pose);

    typename Robot::ConfigurationArray joints{};
    for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
    {
        joints[joint] = static_cast<float>(pose.joint_values[joint]);
    }

    const Eigen::Isometry3f ee = Robot::eefk(joints);
    return base_transforms_[Index] * ee;
}

template <typename... RobotTs>
Eigen::Isometry3f VampInstance<RobotTs...>::endEffectorTransform(std::size_t robot_index,
                                                                 const RobotPose &pose) const
{
    if (robot_index >= kRobotCount)
    {
        throw std::out_of_range("VampInstance: robot index out of range for end effector transform");
    }
    return end_effector_tf_dispatch_[robot_index](*this, pose);
}

template <typename... RobotTs>
Eigen::Isometry3f VampInstance<RobotTs...>::attachmentTransformFromObject(const Object &obj)
{
    Eigen::Isometry3f relative = Eigen::Isometry3f::Identity();
    relative.translation() = Eigen::Vector3f(
        static_cast<float>(obj.x_attach),
        static_cast<float>(obj.y_attach),
        static_cast<float>(obj.z_attach));

    Eigen::Quaternionf q_attach(
        static_cast<float>(obj.qw_attach),
        static_cast<float>(obj.qx_attach),
        static_cast<float>(obj.qy_attach),
        static_cast<float>(obj.qz_attach));
    if (q_attach.norm() > 1e-6F)
    {
        q_attach.normalize();
        relative.linear() = q_attach.toRotationMatrix();
    }

    return relative;
}

template <typename... RobotTs>
Eigen::Isometry3f VampInstance<RobotTs...>::objectPoseToIsometry(const Object &obj)
{
    Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
    tf.translation() = Eigen::Vector3f(
        static_cast<float>(obj.x),
        static_cast<float>(obj.y),
        static_cast<float>(obj.z));
    tf.linear() = quaternionFromObject(obj).toRotationMatrix();
    return tf;
}

template <typename... RobotTs>
auto VampInstance<RobotTs...>::makeAttachmentFromObject(const Object &obj)
    -> std::optional<vamp::collision::Attachment<float>>
{
    const Eigen::Isometry3f relative = attachmentTransformFromObject(obj);
    vamp::collision::Attachment<float> attachment(relative);

    switch (obj.shape)
    {
    case Object::Shape::Cylinder:
    {
        float inflaction_factor = 1.01F; 
        // add a small margin to inflate the cylinder length so that the sphere approximation is better
        float length = static_cast<float>(obj.length * inflaction_factor);
        if (length <= 0.0F)
        {
            length = static_cast<float>(obj.height);
        }
        if (length <= 0.0F)
        {
            return std::nullopt;
        }

        const float radius = static_cast<float>(obj.radius);
        if (radius <= 0.0F)
        {
            return std::nullopt;
        }

        auto cylinder = vamp::collision::factory::cylinder::center::flat(
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            radius,
            length);
        attachment.add_cylinder(cylinder, -1.0F, 0);
        break;
    }
    case Object::Shape::Box:
    {
        const float half_x = static_cast<float>(obj.length) * 0.5F;
        const float half_y = static_cast<float>(obj.width) * 0.5F;
        const float half_z = static_cast<float>(obj.height) * 0.5F;
        if (half_x <= 0.0F || half_y <= 0.0F || half_z <= 0.0F)
        {
            return std::nullopt;
        }

        auto cuboid = vamp::collision::factory::cuboid::flat(
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            half_x,
            half_y,
            half_z);
        attachment.add_cuboid(cuboid, -1.0F, 0);
        break;
    }
    default:
        return std::nullopt;
    }

    return attachment;
}


template <typename... RobotTs>
template <typename Robot>
bool VampInstance<RobotTs...>::matchLinkName(
    const std::string &input,
    const std::string &robot_name,
    std::string &output)
{
    if constexpr (!vamp::robots::LinkMapping<Robot>::available)
    {
        (void)input;
        (void)robot_name;
        (void)output;
        return false;
    }

    const auto &link_names = vamp::robots::LinkMapping<Robot>::link_names;
    for (const auto link_sv : link_names)
    {
        const std::string link(link_sv);
        const auto underscore_pos = link.find('_');
        if (input == link ||
            (!robot_name.empty() && input == robot_name + "::" + link) ||
            (!robot_name.empty() && input == robot_name + "/" + link) ||
            (!robot_name.empty() && input == robot_name + "_" + link) ||
            (!robot_name.empty() && underscore_pos != std::string::npos &&
             input == robot_name + link.substr(underscore_pos)) ||
            (!robot_name.empty() && underscore_pos != std::string::npos &&
             input == robot_name + "/" + link.substr(underscore_pos + 1)))
        {
            output = link;
            return true;
        }
    }

    return false;
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::rebuildCollisionFilter()
{
    collision_filter_ = {};
    rebuildCollisionFilterImpl(std::make_index_sequence<kRobotCount>{});
}

template <typename... RobotTs>
template <std::size_t... I>
void VampInstance<RobotTs...>::rebuildCollisionFilterImpl(std::index_sequence<I...>)
{
    (rebuildCollisionFilterForRobot<I>(), ...);
}

template <typename... RobotTs>
template <std::size_t Index>
void VampInstance<RobotTs...>::rebuildCollisionFilterForRobot()
{
    using Robot = RobotAt<Index>;

    const auto &overrides = link_collision_overrides_[Index];
    const auto &attachment_overrides = attachment_collision_overrides_[Index];
    if (overrides.empty() && attachment_overrides.empty())
    {
        return;
    }

    auto &robot_filter = collision_filter_.ensure_robot(Index, Robot::n_spheres);
    std::vector<std::size_t> spheres;
    for (const auto &[object, links] : overrides)
    {
        for (const auto &link : links)
        {
            spheres.clear();
            if (!vamp::robots::collect_link_spheres<Robot>(link, spheres) || spheres.empty())
            {
                continue;
            }

            for (const auto sphere_index : spheres)
            {
                robot_filter.allow_sphere(sphere_index, object);
            }
        }
    }

    for (const auto &object : attachment_overrides)
    {
        robot_filter.allow_attachment(object);
    }
}

template <typename... RobotTs>
template <std::size_t... I>
bool VampInstance<RobotTs...>::setCollisionDispatch(
    const std::string &object,
    const std::string &link,
    bool allow,
    std::index_sequence<I...>)
{
    bool matched = false;
    ((matched = setCollisionForRobot<I>(object, link, allow) || matched), ...);
    return matched;
}

template <typename... RobotTs>
template <std::size_t Index>
bool VampInstance<RobotTs...>::setCollisionForRobot(
    const std::string &object,
    const std::string &link,
    bool allow)
{
    using Robot = RobotAt<Index>;
    std::lock_guard<std::mutex> lock(objects_mutex_);

    const std::string robot_name = (Index < robot_names_.size()) ? robot_names_[Index] : std::string{};

    auto normalize_to_obj = [&](const std::string &candidate) -> std::string {
        if (robot_name.empty())
        {
            return candidate;
        }
        const std::string dbl_colon = robot_name + "::";
        if (candidate.rfind(dbl_colon, 0) == 0)
        {
            return candidate.substr(dbl_colon.size());
        }
        const std::string slash = robot_name + "/";
        if (candidate.rfind(slash, 0) == 0)
        {
            return candidate.substr(slash.size());
        }
        return candidate;
    };

    const std::string normalized_obj_name = normalize_to_obj(link);

    auto attached_or_pending = [&](const std::string &name) {
        auto it = objects_.find(name);
        if (it == objects_.end())
        {
            return false;
        }
        const Object &obj = it->second;
        if (obj.state == Object::State::Attached)
        {
            return obj.robot_id == static_cast<int>(Index);
        }
        // Object exists but is not attached yet; allow pre-emptive whitelist for this robot.
        return true;
    };

    if (attached_or_pending(normalized_obj_name))
    {
        auto &by_object = attachment_collision_overrides_[Index];
        if (allow)
        {
            by_object.insert(object);
        }
        else
        {
            by_object.erase(object);
        }
        return true;
    }

    std::string normalized_link;
    if (!matchLinkName<Robot>(link, robot_name, normalized_link))
    {
        // neither a link nor an object
        return false;
    }

    auto &by_object = link_collision_overrides_[Index];
    if (allow)
    {
        by_object[object].insert(normalized_link);
    }
    else
    {
        auto it = by_object.find(object);
        if (it != by_object.end())
        {
            it->second.erase(normalized_link);
            if (it->second.empty())
            {
                by_object.erase(it);
            }
        }
    }

    return true;
}


template <typename... RobotTs>
template <std::size_t... I>
void VampInstance<RobotTs...>::initializeDofs(std::index_sequence<I...>)
{
    ((PlanInstance::setRobotDOF(static_cast<int>(I), RobotAt<I>::dimension),
      PlanInstance::setHandDof(static_cast<int>(I), 0)), ...);
}


template <typename... RobotTs>
template <std::size_t... I>
bool VampInstance<RobotTs...>::subsetCollisionImpl(const PoseArray &poses, bool self) const
{
    static_assert(sizeof...(I) > 0, "subsetCollisionImpl requires at least one index");

    if (self)
    {
        return vamp::collision::fkcc_multi_self<kRake>(
            makeState<I>(*poses[I])...);
    }

    if (collision_filter_.empty())
    {
        return vamp::collision::fkcc_multi_all<kRake>(
            environment_,
            makeState<I>(*poses[I])...);
    }

    const auto local_filter = makeSubsetFilter<I...>(collision_filter_);
    return vamp::collision::fkcc_multi_all<kRake>(
        environment_,
        local_filter,
        makeState<I>(*poses[I])...);
}

template <typename... RobotTs>
std::string VampInstance<RobotTs...>::robotNameForIndex(std::size_t robot_index) const
{
    if (robot_index < robot_names_.size() && !robot_names_[robot_index].empty())
    {
        return robot_names_[robot_index];
    }
    return std::string("robot_") + std::to_string(robot_index);
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::attachmentAllowsObject(const std::unordered_set<std::string> &allowed,
                                                      const std::string &other_robot_name,
                                                      const std::string &other_link_name) const
{
    if (allowed.empty())
    {
        return false;
    }
    if (allowed.find(std::string("*")) != allowed.end())
    {
        return true;
    }

    auto contains = [&](const std::string &key) -> bool {
        return allowed.find(key) != allowed.end();
    };

    if (contains(other_robot_name + "::" + other_link_name) ||
        contains(other_robot_name + "/" + other_link_name) ||
        contains(other_robot_name + "_" + other_link_name) ||
        contains(other_link_name))
    {
        return true;
    }

    const auto underscore_pos = other_link_name.find('_');
    if (underscore_pos != std::string::npos && underscore_pos + 1 < other_link_name.size())
    {
        const std::string suffix_with_underscore = other_link_name.substr(underscore_pos);
        const std::string suffix = other_link_name.substr(underscore_pos + 1);
        if (contains(other_robot_name + suffix_with_underscore) ||
            contains(other_robot_name + "/" + suffix) ||
            contains(other_robot_name + "::" + suffix))
        {
            return true;
        }
    }

    return false;
}

template <typename... RobotTs>
template <std::size_t AttachmentRobotIndex, std::size_t OtherRobotIndex, typename OtherSpheres>
bool VampInstance<RobotTs...>::attachmentRobotCollidesWithRobotSpheres(
    const std::vector<vamp::collision::Sphere<vamp::FloatVector<kRake>>> &attachment_spheres,
    const OtherSpheres &other_spheres) const
{
    if (attachment_spheres.empty())
    {
        return false;
    }

    const auto &allowed = attachment_collision_overrides_[AttachmentRobotIndex];
    const std::string other_robot_name = robotNameForIndex(OtherRobotIndex);

    constexpr std::size_t count_b = std::remove_reference_t<decltype(other_spheres.x)>::num_rows;
    for (const auto &sphere : attachment_spheres)
    {
        for (std::size_t b = 0; b < count_b; ++b)
        {
            if (vamp::collision::sphere_sphere_sql2(
                    sphere,
                    other_spheres.x[b],
                    other_spheres.y[b],
                    other_spheres.z[b],
                    other_spheres.r[b])
                    .test_zero())
            {
                continue;
            }

            const std::string other_link_name = linkNameForSphere<RobotAt<OtherRobotIndex>>(b);
            if (attachmentAllowsObject(allowed, other_robot_name, other_link_name))
            {
                continue;
            }
            if (const char *dbg = std::getenv("MR_PLANNER_VAMP_DEBUG_ATTACHMENT_ALLOW"))
            {
                if (std::string(dbg) == "1")
                {
                    std::cerr << "[VampInstance] attachment collision blocked between robot "
                              << AttachmentRobotIndex << " attachment and " << other_robot_name << "::"
                              << other_link_name << " (allowed entries=" << allowed.size() << ")\n";
                }
            }
            return true;
        }
    }

    return false;
}

template <typename... RobotTs>
template <std::size_t... I>
bool VampInstance<RobotTs...>::checkCollisionPackWithAttachmentAllowances(
    const PoseArray &poses,
    const vamp::collision::MultiRobotCollisionFilter *filter,
    std::index_sequence<I...>) const
{
    return fkccMultiAllWithAttachmentAllowances(
        filter,
        std::index_sequence<I...>{},
        makeState<I>(*poses[I])...);
}

template <typename... RobotTs>
template <std::size_t... I>
bool VampInstance<RobotTs...>::fkccMultiAllWithAttachmentAllowances(
    const vamp::collision::MultiRobotCollisionFilter *filter,
    std::index_sequence<I...>,
    const vamp::collision::MultiRobotState<RobotAt<I>, kRake> &... states) const
{
    static_assert(sizeof...(I) == kRobotCount, "fkccMultiAllWithAttachmentAllowances expects a full robot pack");

    auto spheres = std::tuple<typename RobotAt<I>::template Spheres<kRake>...>{};
    auto attachments = std::tuple<
        std::conditional_t<true, std::vector<vamp::collision::Sphere<vamp::FloatVector<kRake>>>, RobotAt<I>>...>{};

    bool valid = true;
    (..., ([&](const auto &state)
           {
               using StateT = vamp::collision::MultiRobotState<RobotAt<I>, kRake>;
               const auto *robot_filter = (filter != nullptr) ? filter->robot(I) : nullptr;
               if (!vamp::collision::detail::process_robot_state<StateT, kRake>(
                       environment_,
                       state,
                       std::get<I>(spheres),
                       std::get<I>(attachments),
                       robot_filter))
               {
                   valid = false;
               }
           }(states)));

    if (!valid)
    {
        return false;
    }

    if (vamp::collision::detail::spheres_cross_variant<kRake, false, 0>(spheres))
    {
        return false;
    }

    if (vamp::collision::detail::attachments_present(attachments))
    {
        bool has_disallowed = false;
        for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto a_c) {
            constexpr std::size_t A = a_c.value;
            if (has_disallowed)
            {
                return;
            }
            const auto &attachment_vec = std::get<A>(attachments);
            if (attachment_vec.empty())
            {
                return;
            }
            for_each_index(std::make_index_sequence<kRobotCount>{}, [&](auto b_c) {
                constexpr std::size_t B = b_c.value;
                if (has_disallowed || A == B)
                {
                    return;
                }
                if (attachmentRobotCollidesWithRobotSpheres<A, B>(attachment_vec, std::get<B>(spheres)))
                {
                    has_disallowed = true;
                }
            });
        });

        if (has_disallowed)
        {
            return false;
        }

        if (vamp::collision::detail::attachments_vs_attachments_all_variant<kRake, false, 0>(attachments))
        {
            return false;
        }
    }

    return true;
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::subsetCollisionSwitch(const PoseArray &poses,
                                                     const std::vector<std::size_t> &active,
                                                     bool self) const
{
    if (active.empty())
    {
        throw std::invalid_argument("VampInstance: no robot poses provided");
    }

    if (active.size() == kRobotCount)
    {
        throw std::logic_error("VampInstance: subsetCollisionSwitch should not handle full robot set");
    }

    if (active.size() == 1)
    {
        const std::size_t idx = active[0];
        if (idx >= kRobotCount)
        {
            throw std::out_of_range("VampInstance: robot index out of range");
        }
        const auto fn = subset_collision_single_dispatch_[idx];
        if (fn == nullptr)
        {
            throw std::invalid_argument("VampInstance: unsupported robot subset");
        }
        return fn(*this, poses, self);
    }

    if (active.size() == 2)
    {
        std::size_t a = active[0];
        std::size_t b = active[1];
        if (a == b)
        {
            throw std::invalid_argument("VampInstance: duplicate robot index in subset");
        }
        if (a > b)
        {
            std::swap(a, b);
        }
        if (b >= kRobotCount)
        {
            throw std::out_of_range("VampInstance: robot index out of range");
        }
        const auto fn = subset_collision_pair_dispatch_[a][b];
        if (fn == nullptr)
        {
            throw std::invalid_argument("VampInstance: unsupported robot pair selection");
        }
        return fn(*this, poses, self);
    }

    throw std::invalid_argument("VampInstance: unsupported robot subset");
}


template <typename... RobotTs>
bool VampInstance<RobotTs...>::subsetMotionSwitch(const PoseArray &start,
                                                  const PoseArray &goal,
                                                  const std::vector<std::size_t> &active,
                                                  double step_size,
                                                  bool self)
{
    if (active.empty())
    {
        throw std::invalid_argument("VampInstance: no robot poses provided");
    }

    auto sorted = active;
   std::sort(sorted.begin(), sorted.end());

    if (sorted.size() == 1)
    {
        const std::size_t idx = sorted[0];
        if (idx >= kRobotCount)
        {
            throw std::out_of_range("VampInstance: robot index out of range");
        }
        const auto fn = subset_motion_single_dispatch_[idx];
        if (fn == nullptr)
        {
            throw std::invalid_argument("VampInstance: unsupported robot subset");
        }
        return fn(*this, start, goal, step_size, self);
    }

    if (sorted.size() == 2)
    {
        const std::size_t a = sorted[0];
        const std::size_t b = sorted[1];
        if (a == b || b >= kRobotCount)
        {
            throw std::invalid_argument("VampInstance: invalid robot pair selection");
        }
        const auto fn = subset_motion_pair_dispatch_[a][b];
        if (fn == nullptr)
        {
            throw std::invalid_argument("VampInstance: unsupported robot pair selection");
        }
        return fn(*this, start, goal, step_size, self);
    }

    throw std::invalid_argument("VampInstance: unsupported robot subset");
}


template <typename... RobotTs>
template <std::size_t... I>
bool VampInstance<RobotTs...>::checkCollisionPack(const PoseArray &poses,
                                                  bool self,
                                                  std::index_sequence<I...>)
{
    const bool valid = self
                           ? vamp::collision::fkcc_multi_self<kRake>(makeState<I>(*poses[I])...)
                           : checkCollisionPackWithAttachmentAllowances(
                                 poses,
                                 collision_filter_.empty() ? nullptr : &collision_filter_,
                                 std::index_sequence<I...>{});

    ++num_collision_checks_;
    return valid;
}


template <typename... RobotTs>
bool VampInstance<RobotTs...>::checkCollision(const std::vector<RobotPose> &poses, bool self, bool /*debug*/)
{
    if (poses.empty())
    {
        throw std::invalid_argument("VampInstance: no poses provided");
    }

    PoseArray gathered = gatherPoses(poses, true);

    std::vector<std::size_t> active;
    active.reserve(kRobotCount);
    for (std::size_t idx = 0; idx < kRobotCount; ++idx)
    {
        if (gathered[idx] != nullptr)
        {
            active.push_back(idx);
        }
    }

    if (active.empty())
    {
        throw std::invalid_argument("VampInstance: no valid robot pose indices supplied");
    }

    bool is_collision_free = false;
    if (active.size() == kRobotCount)
    {
        is_collision_free = checkCollisionPack(gathered, self, std::make_index_sequence<kRobotCount>{});
    }
    else
    {
        is_collision_free = subsetCollisionSwitch(gathered, active, self);
        ++num_collision_checks_;
    }

    // Mirror the public API convention: true signals that a collision was detected.
    return !is_collision_free;
}


template <typename... RobotTs>
bool VampInstance<RobotTs...>::setCollision(
    const std::string &obj_name,
    const std::string &link_name,
    bool allow)
{
    const bool matched = setCollisionDispatch(
        obj_name,
        link_name,
        allow,
        std::make_index_sequence<kRobotCount>{});

    if (!matched)
    {
        return false;
    }

    rebuildCollisionFilter();
    return true;
}


template <typename... RobotTs>
template <std::size_t... I>
bool VampInstance<RobotTs...>::checkMotionPack(const PoseArray &start,
                                               const PoseArray &goal,
                                               double step_size,
                                               bool self,
                                               std::index_sequence<I...>)
{
    static_assert(sizeof...(I) > 0, "checkMotionPack requires at least one robot");

    const double effective_step = (step_size > 0.0) ? step_size : 0.1;

    const std::array<double, sizeof...(I)> distances = {computeDistance(*start[I], *goal[I])...};
    double max_distance = 0.0;
    for (double distance : distances)
    {
        max_distance = std::max(max_distance, distance);
    }

    auto collect_active = [&](const PoseArray &array)
    {
        std::vector<RobotPose> result;
        result.reserve(sizeof...(I));
        (result.push_back(*array[I]), ...);
        return result;
    };

    if (max_distance <= std::numeric_limits<double>::epsilon())
    {
        return checkCollision(collect_active(goal), self);
    }

    if (checkCollision(collect_active(start), self))
    {
        return true;
    }

    const std::size_t total_steps = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(max_distance / effective_step)));
    const std::size_t iterations = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::ceil(static_cast<double>(total_steps) / static_cast<double>(kRake))));

    std::array<float, kRake> lane_percents{};
    for (std::size_t lane = 0; lane < kRake; ++lane)
    {
        lane_percents[lane] = static_cast<float>(lane + 1) / static_cast<float>(kRake);
    }

    auto motion_data = std::tuple<MotionCacheEntry<I>...>{};

    ([&]
     {
         using Robot = RobotAt<I>;
         auto &data = std::get<MotionCacheEntry<I>>(motion_data);
         auto &block = data.block;
         auto &back = data.backstep;
         back.resize(Robot::dimension);
         const auto &start_pose = *start[I];
         const auto &goal_pose = *goal[I];

         for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
         {
             const double start_value = start_pose.joint_values[joint];
             const double delta = goal_pose.joint_values[joint] - start_value;

             std::array<float, kRake> lane_values{};
             for (std::size_t lane = 0; lane < kRake; ++lane)
             {
                 lane_values[lane] = static_cast<float>(start_value + delta * lane_percents[lane]);
             }

             block[joint] = typename Robot::template ConfigurationBlock<kRake>::RowT(lane_values);

             back[joint] = (iterations > 1) ?
                               static_cast<float>(
                                   delta /
                                   (static_cast<double>(iterations) * static_cast<double>(kRake))) :
                               0.0F;
         }
     }(),
     ...);

    for (std::size_t iter = 0; iter < iterations; ++iter)
    {
        bool segment_is_valid = false;
        if (self)
        {
            segment_is_valid = vamp::collision::fkcc_multi_self<kRake>(
                vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                    std::get<MotionCacheEntry<I>>(motion_data).block,
                    base_transforms_[I],
                    attachmentForRobot<I>())...);
        }
        else
        {
            if constexpr (sizeof...(I) == kRobotCount)
            {
                segment_is_valid = fkccMultiAllWithAttachmentAllowances(
                    collision_filter_.empty() ? nullptr : &collision_filter_,
                    std::index_sequence<I...>{},
                    vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                        std::get<MotionCacheEntry<I>>(motion_data).block,
                        base_transforms_[I],
                        attachmentForRobot<I>())...);
            }
            else
            {
                if (collision_filter_.empty())
                {
                    segment_is_valid = vamp::collision::fkcc_multi_all<kRake>(
                        environment_,
                        vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                            std::get<MotionCacheEntry<I>>(motion_data).block,
                            base_transforms_[I],
                            attachmentForRobot<I>())...);
                }
                else
                {
                    const auto local_filter = makeSubsetFilter<I...>(collision_filter_);
                    segment_is_valid = vamp::collision::fkcc_multi_all<kRake>(
                        environment_,
                        local_filter,
                        vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                            std::get<MotionCacheEntry<I>>(motion_data).block,
                            base_transforms_[I],
                            attachmentForRobot<I>())...);
                }
            }
        }

        ++num_collision_checks_;

        if (!segment_is_valid)
        {
            return true;
        }

        if (iter + 1 == iterations)
        {
            break;
        }

        ([&]
         {
             using Robot = RobotAt<I>;
             auto &block = std::get<MotionCacheEntry<I>>(motion_data).block;
             const auto &back = std::get<MotionCacheEntry<I>>(motion_data).backstep;
             using Row = typename Robot::template ConfigurationBlock<kRake>::RowT;
             for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
             {
                 const float step = back[joint];
                 if (step == 0.0F)
                 {
                     continue;
                 }
                 block[joint] = block[joint] - Row::fill(step);
             }
         }(),
         ...);
    }

    // Reaching this point means no collision was detected along the sampled motion.
    return false;
}

template <typename... RobotTs>
template <std::size_t... I>
bool VampInstance<RobotTs...>::checkTrajectoryPack(const std::array<const RobotTrajectory *, kRobotCount> &trajectories,
                                                   std::size_t offset,
                                                   std::size_t lane_count,
                                                   bool self,
                                                   bool cross,
                                                   std::index_sequence<I...>)
{
    static_assert(sizeof...(I) > 0, "checkTrajectoryPack requires at least one robot");

    const std::size_t lanes = std::max<std::size_t>(1, lane_count);

    auto trajectory_data = std::tuple<TrajectoryCacheEntry<I>...>{};

    ([&]
     {
         using Robot = RobotAt<I>;
         auto &entry = std::get<TrajectoryCacheEntry<I>>(trajectory_data);
         auto &block = entry.block;

         const auto *robot_traj = trajectories[I];
         if (!robot_traj)
         {
             throw std::invalid_argument("VampInstance: missing trajectory for required robot index");
         }

        const auto &poses = robot_traj->trajectory;
        for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
        {
            std::array<float, kRake> lane_values{};
            for (std::size_t lane = 0; lane < kRake; ++lane)
            {
                const std::size_t available = (offset < poses.size()) ? (poses.size() - offset) : 0U;
                if (available == 0U)
                {
                    throw std::out_of_range("VampInstance: trajectory offset exceeds available waypoints");
                }

                const std::size_t fill = std::min<std::size_t>(available, lanes);
                const std::size_t sample_lane = (lane < fill) ? lane : (fill - 1U);
                const std::size_t step_index = offset + sample_lane;
                const auto &pose = poses[step_index];
                ensurePoseDimension<Robot>(pose);
                lane_values[lane] = static_cast<float>(pose.joint_values[joint]);
            }

            block[joint] = typename Robot::template ConfigurationBlock<kRake>::RowT(lane_values);
         }
     }(),
     ...);

    bool segment_is_valid = false;

    if (cross)
    {
        const bool cross_valid = vamp::collision::fkcc_multi_cross_attach<kRake>(
            vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                std::get<TrajectoryCacheEntry<I>>(trajectory_data).block,
                base_transforms_[I],
                attachmentForRobot<I>())...);

        // Environment obstacles (e.g., the panda table) are not part of the cross sweep check above.
        // Add a second pass against the environment so sweeps mirror MoveIt’s table collisions.
        bool env_valid = true;
        if (!self && cross_valid)
        {
            if (collision_filter_.empty())
            {
                env_valid = vamp::collision::fkcc_multi_all<kRake>(
                    environment_,
                    vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                        std::get<TrajectoryCacheEntry<I>>(trajectory_data).block,
                        base_transforms_[I],
                        attachmentForRobot<I>())...);
            }
            else
            {
                const auto local_filter = makeSubsetFilter<I...>(collision_filter_);
                env_valid = vamp::collision::fkcc_multi_all<kRake>(
                    environment_,
                    local_filter,
                    vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                        std::get<TrajectoryCacheEntry<I>>(trajectory_data).block,
                        base_transforms_[I],
                        attachmentForRobot<I>())...);
            }
        }

        segment_is_valid = cross_valid && env_valid;
    }
    else if (self)
    {
        segment_is_valid = vamp::collision::fkcc_multi_self<kRake>(
            vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                std::get<TrajectoryCacheEntry<I>>(trajectory_data).block,
                base_transforms_[I],
                attachmentForRobot<I>())...);
    }
    else
    {
        if (collision_filter_.empty())
        {
            segment_is_valid = vamp::collision::fkcc_multi_all<kRake>(
                environment_,
                vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                    std::get<TrajectoryCacheEntry<I>>(trajectory_data).block,
                    base_transforms_[I],
                    attachmentForRobot<I>())...);
        }
        else
        {
            const auto local_filter = makeSubsetFilter<I...>(collision_filter_);
            segment_is_valid = vamp::collision::fkcc_multi_all<kRake>(
                environment_,
                local_filter,
                vamp::collision::make_multi_robot_state<RobotAt<I>, kRake>(
                    std::get<TrajectoryCacheEntry<I>>(trajectory_data).block,
                    base_transforms_[I],
                    attachmentForRobot<I>())...);
        }
    }

    ++num_collision_checks_;
    return !segment_is_valid;
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::subsetTrajectorySwitch(
    const std::array<const RobotTrajectory *, kRobotCount> &trajectories,
    const std::vector<std::size_t> &active,
    std::size_t offset,
    std::size_t lane_count,
    bool self,
    bool cross)
{
    if (active.empty())
    {
        throw std::invalid_argument("VampInstance: no robot trajectories provided");
    }

    auto sorted = active;
    std::sort(sorted.begin(), sorted.end());

    if (sorted.size() == kRobotCount)
    {
        return checkTrajectoryPack(
            trajectories,
            offset,
            lane_count,
            self,
            cross,
            std::make_index_sequence<kRobotCount>{});
    }

    if (sorted.size() == 1)
    {
        const std::size_t idx = sorted[0];
        if (idx >= kRobotCount)
        {
            throw std::out_of_range("VampInstance: robot index out of range for trajectory check");
        }
        const auto fn = subset_trajectory_single_dispatch_[idx];
        if (fn == nullptr)
        {
            throw std::invalid_argument("VampInstance: unsupported robot subset for trajectory check");
        }
        return fn(*this, trajectories, offset, lane_count, self, cross);
    }

    if (sorted.size() == 2)
    {
        const std::size_t a = sorted[0];
        const std::size_t b = sorted[1];
        if (a == b || b >= kRobotCount)
        {
            throw std::invalid_argument("VampInstance: invalid robot pair selection for trajectory check");
        }
        const auto fn = subset_trajectory_pair_dispatch_[a][b];
        if (fn == nullptr)
        {
            throw std::invalid_argument("VampInstance: unsupported robot pair selection for trajectory check");
        }
        return fn(*this, trajectories, offset, lane_count, self, cross);
    }

    throw std::invalid_argument("VampInstance: unsupported robot subset for trajectory check");
}


template <typename... RobotTs>
bool VampInstance<RobotTs...>::checkMultiRobotMotion(const std::vector<RobotPose> &start,
                                                     const std::vector<RobotPose> &goal,
                                                     double step_size,
                                                     bool self)
{
    if (start.size() != goal.size())
    {
        throw std::invalid_argument("VampInstance: start and goal pose counts must match");
    }

    auto start_gathered = gatherPoses(start, true);
    auto goal_gathered = gatherPoses(goal, true);

    std::vector<std::size_t> active;
    active.reserve(kRobotCount);
    for (std::size_t idx = 0; idx < kRobotCount; ++idx)
    {
        const bool have_start = start_gathered[idx] != nullptr;
        const bool have_goal = goal_gathered[idx] != nullptr;

        if (have_start != have_goal)
        {
            throw std::invalid_argument("VampInstance: start/goal pose mismatch for robot index");
        }

        if (have_start)
        {
            active.push_back(idx);
        }
    }

    if (active.empty())
    {
        throw std::invalid_argument("VampInstance: no robot poses provided for motion check");
    }

    if (active.size() == kRobotCount)
    {
        return checkMotionPack(
            start_gathered,
            goal_gathered,
            step_size,
            self,
            std::make_index_sequence<kRobotCount>{});
    }

    return subsetMotionSwitch(start_gathered, goal_gathered, active, step_size, self);
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::checkMultiRobotTrajectory(const MRTrajectory &trajectory,
                                                         bool self)
{
    if (trajectory.empty())
    {
        throw std::invalid_argument("VampInstance: multi-robot trajectory is empty");
    }

    std::array<const RobotTrajectory *, kRobotCount> mapped{};
    mapped.fill(nullptr);

    std::size_t step_count = 0;
    bool step_count_initialized = false;

    for (const auto &robot_traj : trajectory)
    {
        const int robot_id = robot_traj.robot_id;
        if (robot_id < 0 || robot_id >= static_cast<int>(kRobotCount))
        {
            throw std::out_of_range("VampInstance: robot id out of range in trajectory");
        }

        auto index = static_cast<std::size_t>(robot_id);
        if (mapped[index] != nullptr)
        {
            throw std::invalid_argument("VampInstance: duplicate trajectory for robot id");
        }

        mapped[index] = &robot_traj;

        if (!robot_traj.trajectory.empty())
        {
            if (!step_count_initialized)
            {
                step_count = robot_traj.trajectory.size();
                step_count_initialized = true;
            }
            else if (robot_traj.trajectory.size() != step_count)
            {
                throw std::invalid_argument("VampInstance: trajectories must share waypoint count");
            }
        }
    }

    if (!step_count_initialized)
    {
        return false;
    }

    std::vector<std::size_t> active;
    active.reserve(kRobotCount);
    for (std::size_t idx = 0; idx < kRobotCount; ++idx)
    {
        if (mapped[idx])
        {
            if (mapped[idx]->trajectory.size() != step_count)
            {
                throw std::invalid_argument("VampInstance: trajectories must share waypoint count");
            }
            active.push_back(idx);
        }
    }

    if (active.empty())
    {
        throw std::invalid_argument("VampInstance: no robot trajectories provided");
    }

    std::size_t offset = 0;
    while (offset < step_count)
    {
        const std::size_t lane_count = std::min<std::size_t>(kRake, step_count - offset);
        if (subsetTrajectorySwitch(mapped, active, offset, lane_count, self, false))
        {
            return true;
        }
        offset += lane_count;
    }

    return false;
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::checkMultiRobotSweep(const MRTrajectory &trajectory,
                                                    bool self)
{
    if constexpr (kRobotCount < 2U)
    {
        return false;
    }

    if (trajectory.empty())
    {
        throw std::invalid_argument("VampInstance: sweep trajectory set is empty");
    }

    std::array<const RobotTrajectory *, kRobotCount> mapped{};
    mapped.fill(nullptr);

    std::vector<std::size_t> active;
    active.reserve(trajectory.size());

    std::size_t lane_count = 0;

    for (const auto &robot_traj : trajectory)
    {
        const int robot_id = robot_traj.robot_id;
        if (robot_id < 0 || robot_id >= static_cast<int>(kRobotCount))
        {
            throw std::out_of_range("VampInstance: robot id out of range in sweep trajectory");
        }

        const auto index = static_cast<std::size_t>(robot_id);
        if (mapped[index] != nullptr)
        {
            throw std::invalid_argument("VampInstance: duplicate trajectory for robot id in sweep check");
        }

        if (robot_traj.trajectory.empty())
        {
            throw std::invalid_argument("VampInstance: empty robot trajectory in sweep check");
        }

        if (robot_traj.trajectory.size() > kRake)
        {
            throw std::invalid_argument("VampInstance: sweep check supports at most kRake waypoints per robot");
        }

        mapped[index] = &robot_traj;
        active.push_back(index);
        lane_count = std::max<std::size_t>(lane_count, robot_traj.trajectory.size());
    }

    if (active.size() < 2U)
    {
        throw std::invalid_argument("VampInstance: sweep check requires at least two robot trajectories");
    }

    if (lane_count == 0U)
    {
        return false;
    }

    if (lane_count > kRake)
    {
        throw std::invalid_argument("VampInstance: sweep lane count exceeds SIMD width");
    }

    std::sort(active.begin(), active.end());
    active.erase(std::unique(active.begin(), active.end()), active.end());

    return subsetTrajectorySwitch(mapped, active, 0, lane_count, self, true);
}


template <typename... RobotTs>
template <typename Robot>
double VampInstance<RobotTs...>::computeDistanceImpl(const RobotPose &a, const RobotPose &b)
{
    ensurePoseDimension<Robot>(a);
    ensurePoseDimension<Robot>(b);

    double sum_l1 = 0.0;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        const double diff = a.joint_values[i] - b.joint_values[i];
        sum_l1 += std::abs(diff);
    }
    return sum_l1;
}

template <typename... RobotTs>
template <typename Robot>
double VampInstance<RobotTs...>::computeDistanceDimImpl(const RobotPose &a,
                                                        const RobotPose &b,
                                                        int dim)
{
    ensurePoseDimension<Robot>(a);
    ensurePoseDimension<Robot>(b);
    if (dim < 0 || dim >= static_cast<int>(Robot::dimension))
    {
        throw std::out_of_range("VampInstance: joint dimension out of range");
    }

    const std::size_t idx = static_cast<std::size_t>(dim);
    return std::abs(a.joint_values[idx] - b.joint_values[idx]);
}


template <typename... RobotTs>
double VampInstance<RobotTs...>::computeDistance(const RobotPose &a, const RobotPose &b) const
{
    if (a.robot_id != b.robot_id)
    {
        throw std::invalid_argument("VampInstance: distance requested between different robots");
    }

    if (a.robot_id < 0 || a.robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }

    const std::size_t index = static_cast<std::size_t>(a.robot_id);
    return compute_distance_dispatch_[index](a, b);
}

template <typename... RobotTs>
double VampInstance<RobotTs...>::computeDistance(const RobotPose &a, const RobotPose &b, int dim) const
{
    if (a.robot_id != b.robot_id)
    {
        throw std::invalid_argument("VampInstance: distance requested between different robots");
    }

    if (a.robot_id < 0 || a.robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }

    const std::size_t index = static_cast<std::size_t>(a.robot_id);
    return compute_distance_dim_dispatch_[index](a, b, dim);
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::connect(const RobotPose &a,
                                       const RobotPose &b,
                                       double col_step_size,
                                       bool self)
{
    if (a.robot_id != b.robot_id)
    {
        throw std::invalid_argument("VampInstance: connect requested between different robots");
    }

    const double step_size = (col_step_size > 0.0) ? col_step_size : 0.1;
    return !checkMultiRobotMotion({a}, {b}, step_size, self);
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::steer(const RobotPose &a,
                                     const RobotPose &b,
                                     double max_dist,
                                     RobotPose &result,
                                     double col_step_size)
{
    const double distance = computeDistance(a, b);
    if (distance <= max_dist)
    {
        if (connect(a, b, col_step_size))
        {
            result = b;
            return true;
        }
        return false;
    }

    const double ratio = max_dist / distance;
    RobotPose candidate = interpolate(a, b, ratio);
    if (!checkCollision({candidate}, false) && connect(a, candidate, col_step_size))
    {
        result = std::move(candidate);
        return true;
    }

    return false;
}

template <typename... RobotTs>
template <typename Robot>
bool VampInstance<RobotTs...>::sampleImpl(RobotPose &pose)
{
    std::uniform_real_distribution<float> dist(0.0F, 1.0F);
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        const float min_v = Robot::s_a[i];
        const float max_v = Robot::s_a[i] + Robot::s_m[i];
        const float sample = min_v + (max_v - min_v) * dist(rng_);
        pose.joint_values[i] = static_cast<double>(sample);
    }
    return true;
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::sample(RobotPose &pose)
{
    if (pose.robot_id < 0 || pose.robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }
    pose.joint_values.resize(getRobotDOF(pose.robot_id));
    const std::size_t index = static_cast<std::size_t>(pose.robot_id);
    return sample_dispatch_[index](*this, pose);
}

template <typename... RobotTs>
template <typename Robot>
RobotPose VampInstance<RobotTs...>::interpolateImpl(const RobotPose &a,
                                                    const RobotPose &b,
                                                    double t)
{
    ensurePoseDimension<Robot>(a);
    ensurePoseDimension<Robot>(b);

    RobotPose result = a;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        result.joint_values[i] = a.joint_values[i] +
                                 t * (b.joint_values[i] - a.joint_values[i]);
    }
    return result;
}

template <typename... RobotTs>
template <typename Robot>
double VampInstance<RobotTs...>::interpolateDimImpl(const RobotPose &a,
                                                    const RobotPose &b,
                                                    double t,
                                                    int dim)
{
    ensurePoseDimension<Robot>(a);
    ensurePoseDimension<Robot>(b);

    if (dim < 0 || dim >= static_cast<int>(Robot::dimension))
    {
        throw std::out_of_range("VampInstance: joint dimension out of range");
    }

    const std::size_t idx = static_cast<std::size_t>(dim);
    return a.joint_values[idx] + t * (b.joint_values[idx] - a.joint_values[idx]);
}

template <typename... RobotTs>
RobotPose VampInstance<RobotTs...>::interpolate(const RobotPose &a,
                                                const RobotPose &b,
                                                double t) const
{
    if (a.robot_id != b.robot_id)
    {
        throw std::invalid_argument("VampInstance: interpolate requested between different robots");
    }

    if (a.robot_id < 0 || a.robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }

    const std::size_t index = static_cast<std::size_t>(a.robot_id);
    return interpolate_dispatch_[index](a, b, t);
}

template <typename... RobotTs>
double VampInstance<RobotTs...>::interpolate(const RobotPose &a,
                                             const RobotPose &b,
                                             double t,
                                             int dim) const
{
    if (a.robot_id != b.robot_id)
    {
        throw std::invalid_argument("VampInstance: interpolate requested between different robots");
    }

    if (a.robot_id < 0 || a.robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }

    const std::size_t index = static_cast<std::size_t>(a.robot_id);
    return interpolate_dim_dispatch_[index](a, b, t, dim);
}

template <typename... RobotTs>
Eigen::Quaternionf VampInstance<RobotTs...>::quaternionFromObject(const Object &obj)
{
    Eigen::Quaternionf q(
        static_cast<float>(obj.qw),
        static_cast<float>(obj.qx),
        static_cast<float>(obj.qy),
        static_cast<float>(obj.qz));

    const float norm_sq = q.squaredNorm();
    if (!std::isfinite(norm_sq) || norm_sq < 1e-12F)
    {
        q = Eigen::Quaternionf::Identity();
    }
    else
    {
        q.normalize();
    }

    return q;
}

template <typename... RobotTs>
vamp::collision::Cuboid<float> VampInstance<RobotTs...>::makeCuboidShape(const Object &obj)
{
    const Eigen::Quaternionf q = quaternionFromObject(obj);
    const Eigen::Matrix3f rotation = q.toRotationMatrix();

    const float half_x = static_cast<float>(obj.length * 0.5);
    const float half_y = static_cast<float>(obj.width * 0.5);
    const float half_z = static_cast<float>(obj.height * 0.5);

    auto cuboid = vamp::collision::Cuboid<float>(
        static_cast<float>(obj.x),
        static_cast<float>(obj.y),
        static_cast<float>(obj.z),
        rotation(0, 0),
        rotation(1, 0),
        rotation(2, 0),
        rotation(0, 1),
        rotation(1, 1),
        rotation(2, 1),
        rotation(0, 2),
        rotation(1, 2),
        rotation(2, 2),
        half_x,
        half_y,
        half_z);

    cuboid.name = obj.name;
    return cuboid;
}

template <typename... RobotTs>
vamp::collision::Cylinder<float> VampInstance<RobotTs...>::makeCylinderShape(const Object &obj)
{
    const Eigen::Quaternionf q = quaternionFromObject(obj);
    const Eigen::Matrix3f rotation = q.toRotationMatrix();

    const float length = static_cast<float>(obj.length);
    const Eigen::Vector3f axis = rotation.col(2) * length;
    const Eigen::Vector3f center(
        static_cast<float>(obj.x),
        static_cast<float>(obj.y),
        static_cast<float>(obj.z));

    const Eigen::Vector3f start = center - axis * 0.5F;
    const float axis_sq = axis.squaredNorm();
    const float safe_axis_sq = std::max(axis_sq, 1e-12F);
    const float rdv = 1.0F / safe_axis_sq;

    auto cylinder = vamp::collision::Cylinder<float>(
        start.x(),
        start.y(),
        start.z(),
        axis.x(),
        axis.y(),
        axis.z(),
        static_cast<float>(obj.radius),
        rdv);

    cylinder.name = obj.name;
    return cylinder;
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::rebuildEnvironment()
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    environment_input_ = {};
    if (environment_static_)
    {
        // Start from the static environment if available
        environment_input_ = vamp::collision::Environment<float>(*environment_static_);
    }

    for (const auto &name : movable_objects_)
    {
        const auto it = objects_.find(name);
        if (it == objects_.end())
        {
            continue;
        }

        const Object &obj = it->second;
        switch (obj.shape)
        {
        case Object::Shape::Box:
            environment_input_.cuboids.emplace_back(makeCuboidShape(obj));
            break;
        case Object::Shape::Cylinder:
            environment_input_.cylinders.emplace_back(makeCylinderShape(obj));
            break;
        default:
            break;
        }
    }

    environment_input_.sort();
    environment_ = vamp::collision::Environment<vamp::FloatVector<kRake>>(environment_input_);
    vamp::collision::detail::clear_environment_transform_cache<kRake>();
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::addMoveableObject(const Object &obj)
{
    if (obj.name.empty())
    {
        throw std::invalid_argument("VampInstance: object name must be non-empty");
    }

    if (obj.shape != Object::Shape::Box && obj.shape != Object::Shape::Cylinder)
    {
        throw std::invalid_argument("VampInstance: unsupported object shape");
    }

    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        objects_[obj.name] = obj;
        meshcat_dirty_objects_.insert(obj.name);
        if (obj.state == Object::State::Attached)
        {
            movable_objects_.erase(obj.name);
        }
        else
        {
            movable_objects_.insert(obj.name);
        }
    }
    rebuildEnvironment();
    if (visualization_instance_)
    {
        visualization_instance_->addMoveableObject(obj);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::moveObject(const Object &obj)
{
    if (objects_.find(obj.name) == objects_.end())
    {
        addMoveableObject(obj);
        return;
    }

    if (obj.shape != Object::Shape::Box && obj.shape != Object::Shape::Cylinder)
    {
        throw std::invalid_argument("VampInstance: unsupported object shape");
    }

    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        objects_[obj.name] = obj;
        meshcat_dirty_objects_.insert(obj.name);
        if (obj.state == Object::State::Attached)
        {
            movable_objects_.erase(obj.name);
        }
        else
        {
            movable_objects_.insert(obj.name);
        }
    }
    rebuildEnvironment();
    if (visualization_instance_)
    {
        visualization_instance_->moveObject(obj);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
}

template <typename... RobotTs>
Eigen::Vector3d VampInstance<RobotTs...>::getEndEffectorPositionFromPose(const RobotPose &pose) const
{
    if (pose.robot_id < 0 || pose.robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }
    const std::size_t index = static_cast<std::size_t>(pose.robot_id);
    return end_effector_dispatch_[index](*this, pose);
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::attachObjectToRobot(const std::string &name,
                                                   int robot_id,
                                                   const std::string &link_name,
                                                   const RobotPose &pose)
{
    if (robot_id < 0 || robot_id >= static_cast<int>(kRobotCount))
    {
        throw std::out_of_range("VampInstance: robot id out of range");
    }

    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        auto it = objects_.find(name);
        if (it == objects_.end())
        {
            throw std::invalid_argument("VampInstance: object not found for attachment");
        }

        Object &obj = it->second;
        const Eigen::Isometry3f ee_tf = endEffectorTransform(static_cast<std::size_t>(robot_id), pose);
        Eigen::Isometry3f relative_tf = Eigen::Isometry3f::Identity();
        if (obj.state == Object::State::Attached)
        {
            // If the object is already marked as attached, treat the stored attachment pose as authoritative.
            // This is important for environment "tool" attachments (e.g., gripper rods) and for loading
            // scenes/skillplans that already include attach poses.
            relative_tf = attachmentTransformFromObject(obj);
        }
        else
        {
            const Eigen::Isometry3f object_tf = objectPoseToIsometry(obj);
            relative_tf = ee_tf.inverse() * object_tf;
        }

        const Eigen::Vector3f rel_translation = relative_tf.translation();
        obj.x_attach = static_cast<double>(rel_translation.x());
        obj.y_attach = static_cast<double>(rel_translation.y());
        obj.z_attach = static_cast<double>(rel_translation.z());

        Eigen::Quaternionf rel_q(relative_tf.linear());
        if (rel_q.norm() > 1e-6F)
        {
            rel_q.normalize();
        }
        else
        {
            rel_q = Eigen::Quaternionf::Identity();
        }
        obj.qx_attach = static_cast<double>(rel_q.x());
        obj.qy_attach = static_cast<double>(rel_q.y());
        obj.qz_attach = static_cast<double>(rel_q.z());
        obj.qw_attach = static_cast<double>(rel_q.w());

        // Keep the world pose consistent with the attachment for the provided robot pose.
        const Eigen::Isometry3f object_tf = ee_tf * relative_tf;
        const Eigen::Vector3f world_translation = object_tf.translation();
        obj.x = static_cast<double>(world_translation.x());
        obj.y = static_cast<double>(world_translation.y());
        obj.z = static_cast<double>(world_translation.z());

        Eigen::Quaternionf world_q(object_tf.linear());
        if (world_q.norm() > 1e-6F)
        {
            world_q.normalize();
        }
        else
        {
            world_q = Eigen::Quaternionf::Identity();
        }
        obj.qx = static_cast<double>(world_q.x());
        obj.qy = static_cast<double>(world_q.y());
        obj.qz = static_cast<double>(world_q.z());
        obj.qw = static_cast<double>(world_q.w());

        auto attachment = makeAttachmentFromObject(obj);
        if (!attachment)
        {
            throw std::invalid_argument("VampInstance: object cannot be converted into an attachment");
        }

        obj.state = Object::State::Attached;
        obj.robot_id = robot_id;
        obj.parent_link = link_name;

        movable_objects_.erase(name);
        attachments_[static_cast<std::size_t>(robot_id)] = *attachment;
        meshcat_dirty_objects_.insert(name);
    }

    rebuildEnvironment();
    if (visualization_instance_)
    {
        visualization_instance_->attachObjectToRobot(name, robot_id, link_name, pose);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::detachObjectFromRobot(const std::string &name, const RobotPose &pose)
{
    int robot_id = -1;
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        auto it = objects_.find(name);
        if (it == objects_.end())
        {
            return;
        }

        Object &obj = it->second;
        robot_id = obj.robot_id;
        if (obj.state != Object::State::Attached)
        {
            return;
        }

        if (robot_id >= 0 && robot_id < static_cast<int>(kRobotCount))
        {
            const Eigen::Isometry3f ee_tf = endEffectorTransform(static_cast<std::size_t>(robot_id), pose);
            const Eigen::Isometry3f relative_tf = attachmentTransformFromObject(obj);
            const Eigen::Isometry3f object_tf = ee_tf * relative_tf;

            const Eigen::Vector3f world_translation = object_tf.translation();
            obj.x = static_cast<double>(world_translation.x());
            obj.y = static_cast<double>(world_translation.y());
            obj.z = static_cast<double>(world_translation.z());

            Eigen::Quaternionf world_q(object_tf.linear());
            if (world_q.norm() > 1e-6F)
            {
                world_q.normalize();
            }
            else
            {
                world_q = Eigen::Quaternionf::Identity();
            }
            obj.qx = static_cast<double>(world_q.x());
            obj.qy = static_cast<double>(world_q.y());
            obj.qz = static_cast<double>(world_q.z());
            obj.qw = static_cast<double>(world_q.w());
        }

        obj.state = Object::State::Static;
        obj.robot_id = -1;
        obj.parent_link = "world";

        if (robot_id >= 0 && robot_id < static_cast<int>(kRobotCount))
        {
            attachments_[static_cast<std::size_t>(robot_id)].reset();
            attachment_collision_overrides_[static_cast<std::size_t>(robot_id)].erase(name);
        }

        movable_objects_.insert(name);
        meshcat_dirty_objects_.insert(name);
    }
    rebuildEnvironment();
    if (visualization_instance_)
    {
        visualization_instance_->detachObjectFromRobot(name, pose);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::setObjectColor(const std::string &name, double r, double g, double b, double a)
{
    if (visualization_instance_)
    {
        visualization_instance_->setObjectColor(name, r, g, b, a);
    }
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        meshcat_object_colors_[name] = {r, g, b, a};
        meshcat_dirty_objects_.insert(name);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::removeObject(const std::string &name)
{
    bool filter_updated = false;
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        const auto it = objects_.find(name);
        if (it == objects_.end())
        {
            return;
        }

        objects_.erase(it);
        movable_objects_.erase(name);
        meshcat_deleted_objects_.insert(name);
        meshcat_dirty_objects_.erase(name);
        meshcat_object_colors_.erase(name);
        for (auto &overrides : link_collision_overrides_)
        {
            auto map_it = overrides.find(name);
            if (map_it != overrides.end())
            {
                overrides.erase(map_it);
                filter_updated = true;
            }
        }
        for (auto &attachments : attachment_collision_overrides_)
        {
            if (attachments.erase(name) > 0U)
            {
                filter_updated = true;
            }
        }
    }
    if (filter_updated)
    {
        rebuildCollisionFilter();
    }
    if (visualization_instance_)
    {
        visualization_instance_->removeObject(name);
    }
    if (meshcat_enabled_)
    {
        meshcat_dirty_ = true;
        if (meshcat_options_.auto_flush)
        {
            publishMeshcatScene();
        }
    }
    rebuildEnvironment();
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::updateScene() {
    if (visualization_instance_) {
        visualization_instance_->updateScene();
    }
    if (meshcat_enabled_) {
        publishMeshcatScene();
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::resetScene(bool reset_sim) {
    if (visualization_instance_) {
        visualization_instance_->resetScene(reset_sim);
    }
    if (meshcat_enabled_ && reset_sim) {
        resetMeshcatScene(reset_sim);
    }
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        objects_.clear();
        movable_objects_.clear();
    }
    rebuildEnvironment();
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::printKnownObjects() const
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    std::cout << "[VAMP] Robots:" << std::endl;
    for (std::size_t i = 0; i < robot_shadow_.size(); ++i)
    {
        std::cout << "  [" << i << "] ";
        if (i < robot_names_.size())
        {
            std::cout << robot_names_[i];
        }
        else
        {
            std::cout << "robot_" << i;
        }

        const RobotPose &pose = robot_shadow_[i];
        if (pose.joint_values.empty() && pose.hand_values.empty())
        {
            std::cout << " (pose not set)" << std::endl;
            continue;
        }

        std::cout << " joints=[";
        for (std::size_t j = 0; j < pose.joint_values.size(); ++j)
        {
            if (j > 0U)
            {
                std::cout << ", ";
            }
            std::cout << pose.joint_values[j];
        }
        std::cout << "]";

        if (!pose.hand_values.empty())
        {
            std::cout << " hand=[";
            for (std::size_t j = 0; j < pose.hand_values.size(); ++j)
            {
                if (j > 0U)
                {
                    std::cout << ", ";
                }
                std::cout << pose.hand_values[j];
            }
            std::cout << "]";
        }
        std::cout << std::endl;
    }

    auto print_object_pose = [&](const Object &obj, const std::string &indent) {
        const Eigen::Isometry3f tf = poseForObject(obj);
        const Eigen::Vector3f t = tf.translation();
        Eigen::Quaternionf q(tf.linear());
        if (q.norm() > 1e-6F)
        {
            q.normalize();
        }
        std::cout << indent << obj.name << " pos=("
                  << t.x() << ", " << t.y() << ", " << t.z()
                  << ") quat=(" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")";
    };

    auto print_object_size = [&](const Object &obj) {
        switch (obj.shape)
        {
        case Object::Shape::Box:
            std::cout << " size=(" << obj.length << ", " << obj.width << ", " << obj.height << ")";
            break;
        case Object::Shape::Cylinder:
            std::cout << " radius=" << obj.radius << " length=" << obj.length;
            break;
        default:
            break;
        }
    };

    std::cout << "[VAMP] Movable objects:" << std::endl;
    for (const auto &name : movable_objects_)
    {
        const auto it = objects_.find(name);
        if (it == objects_.end())
        {
            continue;
        }
        std::cout << "  ";
        print_object_pose(it->second, "");
        print_object_size(it->second);
        std::cout << " state=" << static_cast<int>(it->second.state) << std::endl;
    }

    std::cout << "[VAMP] Attached objects:" << std::endl;
    for (const auto &kv : objects_)
    {
        const Object &obj = kv.second;
        if (obj.state != Object::State::Attached)
        {
            continue;
        }
        const bool has_robot_name = obj.robot_id >= 0 &&
                                    static_cast<std::size_t>(obj.robot_id) < robot_names_.size();
        const std::string robot_label = has_robot_name ?
                                            robot_names_[static_cast<std::size_t>(obj.robot_id)] :
                                            std::string("robot_") + std::to_string(obj.robot_id);
        std::cout << "  ";
        print_object_pose(obj, "");
        print_object_size(obj);
        std::cout << " attached_to=" << robot_label << " link=" << obj.parent_link << std::endl;
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::enableMeshcat(const std::string &host, std::uint16_t port)
{
    meshcat_options_.host = host;
    meshcat_options_.port = port;
    meshcat_enabled_ = true;
    visualization_instance_.reset();
    meshcat_dirty_ = true;
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        meshcat_needs_full_scene_ = true;
        meshcat_dirty_objects_.clear();
        meshcat_deleted_objects_.clear();
    }
    meshcat_connected_ = false;
    if (meshcat_socket_.is_open())
    {
        boost::system::error_code ec;
        meshcat_socket_.close(ec);
    }
    startMeshcatWorker();
    publishMeshcatScene();
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::disableMeshcat()
{
    meshcat_enabled_ = false;
    meshcat_connected_ = false;
    stopMeshcatWorker();
    if (meshcat_socket_.is_open())
    {
        boost::system::error_code ec;
        meshcat_socket_.close(ec);
    }
}

template <typename... RobotTs>
template <std::size_t Index>
void VampInstance<RobotTs...>::appendRobotSpheres(Json::Value &out) const
{
    if (Index >= robot_shadow_.size())
    {
        return;
    }
    const RobotPose &pose = robot_shadow_[Index];
    if (pose.joint_values.empty())
    {
        return;
    }

    using Robot = RobotAt<Index>;
    typename Robot::template Spheres<kRake> spheres{};
    Robot::sphere_fk(configurationBlockFromPose<Robot>(pose), spheres);

    const Eigen::Isometry3f base_tf = base_transforms_[Index];
    for (std::size_t i = 0; i < Robot::n_spheres; ++i)
    {
        const float lx = static_cast<float>(spheres.x[{i, 0}]);
        const float ly = static_cast<float>(spheres.y[{i, 0}]);
        const float lz = static_cast<float>(spheres.z[{i, 0}]);
        const float radius = static_cast<float>(spheres.r[{i, 0}]);
        const Eigen::Vector3f world = base_tf * Eigen::Vector3f(lx, ly, lz);

        Json::Value entry;
        entry["robot_id"] = static_cast<int>(Index);
        if (Index < robot_names_.size())
        {
            entry["robot"] = robot_names_[Index];
        }
        entry["link"] = linkNameForSphere<Robot>(i);
        entry["sphere_index"] = static_cast<int>(i);
        entry["center"] = toJsonArray(world);
        entry["radius"] = radius;

        if (Index < meshcat_robot_colors_.size())
        {
            entry["rgba"] = toJsonArray(meshcat_robot_colors_[Index]);
        }
        out.append(std::move(entry));
    }
}

template <typename... RobotTs>
Eigen::Isometry3f VampInstance<RobotTs...>::poseForObject(const Object &obj) const
{
    if (obj.state == Object::State::Attached && obj.robot_id >= 0 &&
        static_cast<std::size_t>(obj.robot_id) < robot_shadow_.size())
    {
        const RobotPose &pose = robot_shadow_[static_cast<std::size_t>(obj.robot_id)];
        if (!pose.joint_values.empty())
        {
            const Eigen::Isometry3f ee_tf = endEffectorTransform(static_cast<std::size_t>(obj.robot_id), pose);
            const Eigen::Isometry3f rel_tf = attachmentTransformFromObject(obj);
            return ee_tf * rel_tf;
        }
    }
    return objectPoseToIsometry(obj);
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::publishMeshcatScene()
{
    if (!meshcat_enabled_ || !meshcat_dirty_)
    {
        return;
    }

    Json::Value msg;
    msg["action"] = "update";
    if (meshcat_debug_)
    {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        msg["timestamp"] = std::chrono::duration<double>(now).count();
    }

    Json::Value robot_spheres(Json::arrayValue);
    [&]<std::size_t... I>(std::index_sequence<I...>)
    {
        (appendRobotSpheres<I>(robot_spheres), ...);
    }(std::make_index_sequence<kRobotCount>{});
    msg["robot_spheres"] = std::move(robot_spheres);

    Json::Value objects(Json::arrayValue);
    Json::Value deleted_objects(Json::arrayValue);
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);

        auto append_object = [&](const Object &obj) {
            Json::Value entry;
            entry["name"] = obj.name;
            entry["state"] = obj.state;
            entry["parent_link"] = obj.parent_link;
            entry["robot_id"] = obj.robot_id;

            const Eigen::Isometry3f pose_tf = poseForObject(obj);
            const Eigen::Vector3f t = pose_tf.translation();
            Eigen::Quaternionf q(pose_tf.linear());
            if (q.norm() > 1e-6F)
            {
                q.normalize();
            }
            entry["position"] = toJsonArray(t);
            entry["quaternion"] = Json::Value(Json::arrayValue);
            entry["quaternion"].append(q.x());
            entry["quaternion"].append(q.y());
            entry["quaternion"].append(q.z());
            entry["quaternion"].append(q.w());

            std::array<double, 4> rgba = {0.0, 1.0, 0.0, 1.0};
            const auto color_it = meshcat_object_colors_.find(obj.name);
            if (color_it != meshcat_object_colors_.end())
            {
                rgba = color_it->second;
            }
            entry["rgba"] = toJsonArray(rgba);

            switch (obj.shape)
            {
            case Object::Shape::Box:
                entry["type"] = "box";
                entry["size"] = Json::Value(Json::arrayValue);
                entry["size"].append(obj.length);
                entry["size"].append(obj.width);
                entry["size"].append(obj.height);
                break;
            case Object::Shape::Sphere:
                entry["type"] = "sphere";
                entry["radius"] = obj.radius;
                break;
            case Object::Shape::Cylinder:
                entry["type"] = "cylinder";
                entry["radius"] = obj.radius;
                entry["length"] = (obj.length > 0.0) ? obj.length : obj.height;
                break;
            default:
                entry["type"] = "unknown";
                break;
            }

            objects.append(std::move(entry));
        };

        if (meshcat_needs_full_scene_)
        {
            for (const auto &kv : objects_)
            {
                append_object(kv.second);
            }
        }
        else
        {
            for (const auto &kv : objects_)
            {
                const Object &obj = kv.second;
                if (obj.state == Object::State::Attached || meshcat_dirty_objects_.count(obj.name) > 0U)
                {
                    append_object(obj);
                }
            }
        }

        for (const auto &name : meshcat_deleted_objects_)
        {
            deleted_objects.append(name);
        }
        meshcat_dirty_objects_.clear();
        meshcat_deleted_objects_.clear();
        meshcat_needs_full_scene_ = false;
    }

    if (objects.size() > 0U)
    {
        msg["objects"] = std::move(objects);
    }
    if (deleted_objects.size() > 0U)
    {
        msg["deleted_objects"] = std::move(deleted_objects);
    }

    if (meshcat_debug_)
    {
        const auto robot_count = msg["robot_spheres"].size();
        const auto obj_count = msg.isMember("objects") ? msg["objects"].size() : 0U;
        std::cerr << "[meshcat] publishScene robots=" << robot_count
                  << " objects=" << obj_count << std::endl;
    }
    sendMeshcatJson(msg);
    meshcat_dirty_ = false;
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::resetMeshcatScene(bool /*reset_sim*/)
{
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        meshcat_object_colors_.clear();
        meshcat_needs_full_scene_ = true;
        meshcat_dirty_objects_.clear();
        meshcat_deleted_objects_.clear();
    }
    meshcat_dirty_ = true;

    Json::Value msg;
    msg["action"] = "delete_all";
    const std::string payload = Json::writeString(meshcat_writer_builder_, msg) + "\n";
    enqueueMeshcatPayload(payload, true);
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::sendMeshcatJson(const Json::Value &msg)
{
    const std::string payload = Json::writeString(meshcat_writer_builder_, msg) + "\n";
    enqueueMeshcatPayload(payload, false);
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::connectMeshcat()
{
    try
    {
        meshcat_io_ctx_.restart();
        meshcat_socket_ = boost::asio::ip::tcp::socket(meshcat_io_ctx_);
        boost::asio::ip::tcp::resolver resolver(meshcat_io_ctx_);
        const auto endpoints = resolver.resolve(meshcat_options_.host, std::to_string(meshcat_options_.port));
        boost::asio::connect(meshcat_socket_, endpoints);
        meshcat_connected_ = true;
        {
            std::lock_guard<std::mutex> lock(objects_mutex_);
            meshcat_needs_full_scene_ = true;
            meshcat_dirty_objects_.clear();
            meshcat_deleted_objects_.clear();
        }
        std::cerr << "[meshcat] connected to " << meshcat_options_.host << ":" << meshcat_options_.port << std::endl;
    }
    catch (const std::exception &ex)
    {
        meshcat_connected_ = false;
        std::cerr << "[meshcat] connection failed: " << ex.what() << std::endl;
    }
}

template <typename... RobotTs>
bool VampInstance<RobotTs...>::ensureMeshcatConnected()
{
    if (meshcat_connected_)
    {
        return true;
    }
    connectMeshcat();
    return meshcat_connected_;
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::enqueueMeshcatPayload(const std::string &payload, bool is_reset)
{
    // Skip duplicate payloads to reduce traffic.
    if (payload == meshcat_last_payload_)
    {
        return;
    }
    meshcat_last_payload_ = payload;
    {
        std::unique_lock<std::mutex> lock(meshcat_mutex_);
        if (is_reset)
        {
            meshcat_queue_.clear();
            meshcat_queue_.push_back(payload);
            meshcat_queue_has_reset_ = true;
        }
        else if (meshcat_queue_has_reset_)
        {
            // Preserve the reset at the front, but keep only the latest update behind it.
            if (meshcat_queue_.size() == 1U)
            {
                meshcat_queue_.push_back(payload);
            }
            else
            {
                meshcat_queue_.back() = payload;
            }
        }
        else
        {
            // Keep only the latest payload to avoid backlog; drop older ones.
            meshcat_queue_.clear();
            meshcat_queue_.push_back(payload);
            meshcat_queue_has_reset_ = false;
        }
    }
    meshcat_cv_.notify_one();
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::startMeshcatWorker()
{
    if (meshcat_worker_running_)
    {
        return;
    }
    meshcat_worker_running_ = true;
    meshcat_thread_ = std::thread([this]() { runMeshcatWorker(); });
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::stopMeshcatWorker()
{
    if (!meshcat_worker_running_)
    {
        return;
    }
    {
        std::unique_lock<std::mutex> lock(meshcat_mutex_);
        meshcat_worker_running_ = false;
    }
    meshcat_cv_.notify_all();
    if (meshcat_thread_.joinable())
    {
        meshcat_thread_.join();
    }
}

template <typename... RobotTs>
void VampInstance<RobotTs...>::runMeshcatWorker()
{
    while (true)
    {
        std::string payload;
        {
            std::unique_lock<std::mutex> lock(meshcat_mutex_);
            meshcat_cv_.wait(lock, [this]() { return !meshcat_worker_running_ || !meshcat_queue_.empty(); });
            if (!meshcat_worker_running_ && meshcat_queue_.empty())
            {
                return;
            }
            if (meshcat_queue_has_reset_)
            {
                payload = std::move(meshcat_queue_.front());
                meshcat_queue_.pop_front();
                meshcat_queue_has_reset_ = false;
            }
            else
            {
                payload = std::move(meshcat_queue_.back());
                meshcat_queue_.clear();
            }
        }

        if (!meshcat_enabled_)
        {
            continue;
        }
        if (!ensureMeshcatConnected())
        {
            continue;
        }
        try
        {
            boost::asio::write(meshcat_socket_, boost::asio::buffer(payload.data(), payload.size()));
            if (meshcat_debug_)
            {
                std::cerr << "[meshcat] sent " << payload.size() << " bytes (async)\n";
            }
        }
        catch (const std::exception &ex)
        {
            meshcat_connected_ = false;
            std::cerr << "[meshcat] async send failed: " << ex.what() << std::endl;
        }
    }
}

template <typename... RobotTs>
template <typename Robot>
std::string VampInstance<RobotTs...>::linkNameForSphere(std::size_t sphere_index)
{
    if constexpr (vamp::robots::LinkMapping<Robot>::available)
    {
        const auto &mapping = vamp::robots::LinkMapping<Robot>::sphere_to_link;
        const auto &names = vamp::robots::LinkMapping<Robot>::link_names;
        if (sphere_index < mapping.size())
        {
            const std::size_t link_idx = mapping[sphere_index];
            if (link_idx < names.size())
            {
                return std::string(names[link_idx]);
            }
        }
    }
    return std::string("sphere_") + std::to_string(sphere_index);
}

template <typename... RobotTs>
Json::Value VampInstance<RobotTs...>::toJsonArray(const Eigen::Vector3f &vec)
{
    Json::Value arr(Json::arrayValue);
    arr.append(vec.x());
    arr.append(vec.y());
    arr.append(vec.z());
    return arr;
}

template <typename... RobotTs>
Json::Value VampInstance<RobotTs...>::toJsonArray(const std::array<double, 4> &rgba)
{
    Json::Value arr(Json::arrayValue);
    arr.append(rgba[0]);
    arr.append(rgba[1]);
    arr.append(rgba[2]);
    arr.append(rgba[3]);
    return arr;
}

template <typename... RobotTs>
std::array<double, 4> VampInstance<RobotTs...>::colorFromPalette(std::size_t index)
{
    constexpr std::array<std::array<double, 4>, 6> palette{{
        {0.1, 0.6, 1.0, 0.85},
        {1.0, 0.55, 0.2, 0.85},
        {0.25, 0.8, 0.5, 0.85},
        {0.9, 0.35, 0.7, 0.85},
        {0.95, 0.85, 0.2, 0.85},
        {0.55, 0.55, 0.95, 0.85},
    }};
    return palette[index % palette.size()];
}

#endif // MR_PLANNER_VAMP_INSTANCE_INCLUDE_IMPL

#undef MR_PLANNER_VAMP_INSTANCE_INCLUDE_IMPL

#endif // VAMP_INSTANCE_H
