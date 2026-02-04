#ifndef MR_PLANNER_INSTANCE_H
#define MR_PLANNER_INSTANCE_H

#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <memory>
#include <vector>
#include <random>
#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cstdint>
#include "mr_planner/planning/pose_hash.h"
#include "mr_planner/planning/voxel_grid.h"

#include <array>
#include <cassert>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#if MR_PLANNER_WITH_ROS
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#endif


// Abstract base class for the planning scene interface

struct Object  {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & name;
        ar & state;
        ar & parent_link;
        ar & robot_id;
        ar & x;
        ar & y;
        ar & z;
        ar & qx;
        ar & qy;
        ar & qz;
        ar & qw;
        ar & shape;
        ar & radius;
        ar & length;
        ar & width;
        ar & height;
        ar & mesh_path;
    }    
    
    enum State {
        Static = 0,
        Attached = 1,
        Supported = 2,
        Handover = 3,
    };
    enum Shape {
        Box = 0,
        Sphere = 1,
        Cylinder = 2,
        Mesh = 3,
    };

    Object() = default;
    Object(const std::string &name, const std::string& parent_link, State state, double x, double y, double z, double qx, double qy, double qz, double qw):
        name(name), parent_link(parent_link), state(state), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw) 
        {}
    
    std::string name;
    // mode of the object
    State state = Static;
    std::string parent_link;
    int robot_id = -1;

    // geometry of the object
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double x_attach = 0.0, y_attach = 0.0, z_attach = 0.0;
    double qx_attach = 0.0, qy_attach = 0.0, qz_attach = 0.0, qw_attach = 1.0;

    // collision shape of the object
    Shape shape = Box;
    double radius = 0.0;
    double length = 0.0; // x
    double width = 0.0; // y
    double height = 0.0; // z
    std::string mesh_path;
};

struct RobotMode {

    enum Type {
        Free = 0,
        Carry = 1,
        Hold = 2,
    };
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & type;
        ar & carried_obj;
        ar & held_obj;
        ar & ee_link;
        ar & obj;
    }
    
    Type type = Free;
    std::string carried_obj;
    std::string held_obj;
    std::string ee_link;
    std::shared_ptr<Object> obj;
};


struct RobotTrajectory
{
    template <class Archive>
    void serialize(Archive &archive, const unsigned int /*version*/)
    {
        archive & robot_id;
        archive & trajectory;
        archive & times;
        archive & act_ids;
        archive & cost;
    }

    int robot_id{0};
    std::vector<RobotPose> trajectory;
    std::vector<double> times;
    std::vector<int> act_ids;
    double cost{0.0};
    int num_col_checks{0};
    int num_nodes_expanded{0};
};

using MRTrajectory = std::vector<RobotTrajectory>;

struct LinkCollision
{
    int robot_a{-1};
    Object link_a;
    int robot_b{-1};
    Object link_b;
};

// Forward declaration of the hash function
namespace std {
    template <>
    struct hash<RobotPose> {
        std::size_t operator()(const RobotPose& pose) const {
            std::size_t h1 = std::hash<int>()(pose.robot_id);
            std::size_t h2 = std::hash<std::string>()(pose.robot_name);

            // Custom hash function for std::vector<double>
            auto hash_vector = [](const std::vector<double>& vec) {
                std::size_t seed = 0;
                for (double val : vec) {
                    seed ^= std::hash<double>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }
                return seed;
            };

            std::size_t h3 = hash_vector(pose.joint_values);
            std::size_t h4 = hash_vector(pose.hand_values);

            return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3); // Combine the hash values
        }
    };
}

class PlanInstance {
public:
    using Point3f = std::array<float, 3>;
    using PointCloud = std::vector<Point3f>;

    virtual void setVisualizationInstance(const std::shared_ptr<PlanInstance> &instance) {}
    virtual void enableMeshcat(const std::string &/*host*/, std::uint16_t /*port*/) {}
    virtual void disableMeshcat() {}
    virtual void setNumberOfRobots(int num_robots);
    virtual void setRobotNames(const std::vector<std::string>& robot_names) {
        robot_names_ = robot_names;
    }
    virtual void setHandNames(const std::vector<std::string>& hand_names) {
        hand_names_ = hand_names;
    }
    virtual std::string getRobotName(int id) const {
        assert (id < robot_names_.size());
        return robot_names_[id];
    }
    virtual void setStartPose(int robot_id, const std::vector<double>& pose);
    virtual void setGoalPose(int robot_id, const std::vector<double>& pose);
    virtual bool has_intersection(const std::vector<int> &sorted_vector_a, const std::vector<int> &sorted_vector_b) 
        { throw std::runtime_error("Not implemented");};
    virtual std::vector<int> getOccupiedVoxels(const RobotPose& pose, std::shared_ptr<VoxelGrid> grid)
        { throw std::runtime_error("Not implemented");};
    virtual bool checkVertexCollisionByVoxels(const std::vector<RobotPose> &poses, std::shared_ptr<VoxelGrid> grid)
        { throw std::runtime_error("Not implemented");};
    virtual std::pair<bool, std::unordered_set<int>> checkEdgeCollisionByVoxels(const std::vector<PoseEdge> &edges, 
                std::shared_ptr<VoxelGrid> grid) { throw std::runtime_error("Not implemented");};
    virtual bool testVoxelCollisionCheck(const std::vector<RobotPose> &poses, std::shared_ptr<VoxelGrid> grid)
        { throw std::runtime_error("Not implemented");};
    virtual bool checkCollision(const std::vector<RobotPose> &poses, bool self, bool debug=false) = 0;

    // Returns true as soon as any interpolated waypoint along the motion is found in collision.
    // A return value of false indicates that every sampled waypoint is collision free.
    virtual bool checkMultiRobotMotion(const std::vector<RobotPose> &start,
                                       const std::vector<RobotPose> &goal,
                                       double step_size,
                                       bool self=false);
    virtual bool checkMultiRobotTrajectory(const MRTrajectory &trajectory,
                                           bool self=false);
    virtual bool checkMultiRobotSweep(const MRTrajectory &trajectory,
                                      bool self=false) = 0;
    double computeMotionStepSize(const std::vector<RobotPose> &start,
                                 const std::vector<RobotPose> &goal,
                                 int num_samples) const;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b) const = 0;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b, int dim) const = 0;
    virtual bool connect(const RobotPose& a, const RobotPose& b, double col_step_size = 0.1, bool debug=false) = 0;
    virtual bool steer(const RobotPose& a, const RobotPose& b, double max_dist,  RobotPose& result, double col_step_size = 0.1) = 0;
    virtual bool sample(RobotPose &pose) = 0;
    virtual double getVMax(int robot_id);
    virtual void setVmax(double vmax);
    virtual void setRandomSeed(unsigned int seed) { (void)seed; }
    virtual RobotPose interpolate(const RobotPose &a, const RobotPose&b, double t) const = 0;
    virtual double interpolate(const RobotPose &a, const RobotPose&b, double t, int dim) const = 0;
    virtual void addMoveableObject(const Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void moveObject(const Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void removeObject(const std::string& name) { throw std::runtime_error("Not implemented");};
    virtual void moveRobot(int robot_id, const RobotPose& pose) { throw std::runtime_error("Not implemented");};
    virtual void attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) { throw std::runtime_error("Not implemented");};
    virtual void setRobotBaseTransform(int robot_id, const Eigen::Isometry3d &transform) {}
    virtual void detachObjectFromRobot(const std::string& name, const RobotPose &pose) { throw std::runtime_error("Not implemented");};
    virtual void setObjectColor(const std::string &/*name*/, double /*r*/, double /*g*/, double /*b*/, double /*a*/) {}
    virtual Eigen::Vector3d getEndEffectorPositionFromPose(const RobotPose &pose) const = 0;
    virtual void updateScene() = 0;
    virtual void resetScene(bool reset_sim) = 0;
    virtual void plotEE(const RobotPose& pose, int marker_id) {throw std::runtime_error("Not implemented");};
    virtual void setPadding(double padding) {throw std::runtime_error("Not implemented");};
    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) { throw std::runtime_error("Not implemented");};
    virtual void printKnownObjects() const { throw std::runtime_error("Not implemented");};
    virtual void setPointCloud(const PointCloud &/*points*/,
                               float /*r_min*/,
                               float /*r_max*/,
                               float /*r_point*/) { throw std::runtime_error("Not implemented"); }
    virtual void clearPointCloud() { throw std::runtime_error("Not implemented"); }
    virtual bool hasPointCloud() const { return false; }
    virtual std::size_t pointCloudSize() const { return 0; }
    virtual PointCloud filterSelfFromPointCloud(const PointCloud &/*points*/,
                                                const std::vector<RobotPose> &/*poses*/,
                                                float /*padding*/) const
    {
        throw std::runtime_error("Not implemented");
    }

    virtual std::uint64_t environmentVersion() const { return environment_version_.load(); }
    // Save/restore full planning scene snapshots
    virtual void pushScene() { throw std::runtime_error("Not implemented"); };
    virtual void popScene(bool apply_to_sim = true) { throw std::runtime_error("Not implemented"); };
    virtual int numCollisionChecks();
    // Optional debug hook: return colliding link pairs for the provided poses.
    virtual std::vector<LinkCollision> debugCollidingLinks(const std::vector<RobotPose> &poses, bool self)
    {
        (void)poses;
        (void)self;
        return {};
    }

    // Returns the end-effector transform in the world frame for the provided pose.
    // Default implementation returns the position with identity rotation.
    virtual Eigen::Isometry3d getEndEffectorTransformFromPose(const RobotPose &pose) const
    {
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.translation() = getEndEffectorPositionFromPose(pose);
        return tf;
    }

    struct InverseKinematicsOptions
    {
        static constexpr double kPi = 3.14159265358979323846;

        std::optional<std::vector<double>> seed;
        std::optional<std::vector<std::vector<double>>> fixed_joints;
        int max_restarts{1};
        int max_iters{100};
        double tol_pos{0.025};
        double tol_ang{15.0 * kPi / 180.0};
        double step_scale{1.0};
        double damping{1e-3};
        bool self_only{false};
    };

    // Returns a joint configuration that reaches `target` within tolerances. Implementations may return
    // std::nullopt when no solution was found. Collision checking is optional and controlled by `options`.
    virtual std::optional<std::vector<double>> inverseKinematics(
        int robot_id,
        const Eigen::Isometry3d &target,
        const InverseKinematicsOptions &options)
    {
        (void)robot_id;
        (void)target;
        (void)options;
        return std::nullopt;
    }
    // Additional methods for future functionalities can be added here
    virtual ~PlanInstance() = default;

    virtual std::string instanceType() const {
        return instance_type_;
    }

    virtual int getNumberOfRobots() const {
        return num_robots_;
    }

    virtual std::vector<RobotPose> getStartPoses() const {
        return start_poses_;
    }

    virtual std::vector<RobotPose> getGoalPoses() const {
        return goal_poses_;
    }

    virtual RobotPose getStartPose(int robot_id) const {
        assert (robot_id < start_poses_.size());
        return start_poses_[robot_id];
    }

    virtual RobotPose getGoalPose(int robot_id) const {
        assert (robot_id < goal_poses_.size());
        return goal_poses_[robot_id];
    }

    virtual void setPoseName(const std::string& pose_name) {
        pose_name_ = pose_name;
    }
    virtual std::string getPoseName() const {
        return pose_name_;
    }

    virtual RobotPose initRobotPose(int robot_id) const;

    virtual void setRobotDOF(int robot_id, size_t dof);

    virtual void setHandDof(int robot_id, size_t dof);

    virtual size_t getRobotDOF(int robot_id) const;

    virtual size_t getHandDOF(int robot_id) const;

    virtual bool hasObject(const std::string& name) const {
        return objects_.find(name) != objects_.end();
    }

    virtual Object getObject(const std::string& name) const {
        return objects_.at(name);
    }

    virtual std::vector<Object> getAttachedObjects(int robot_id) const {
        std::vector<Object> attached_objects;
        for (const auto& obj : objects_) {
            if (obj.second.robot_id == robot_id && obj.second.state == Object::State::Attached) {
                attached_objects.push_back(obj.second);
            }
        }
        return attached_objects;
    }

protected:
#if MR_PLANNER_WITH_ROS
    static shape_msgs::SolidPrimitive getPrimitive(const Object &obj);
    static geometry_msgs::Pose getPose(const Object &obj);
#endif
    void bumpEnvironmentVersion() { ++environment_version_; }

    int num_robots_;
    double v_max_ = 1.0;
    std::vector<RobotPose> start_poses_;
    std::vector<size_t> robot_dof_, hand_dof_;
    std::vector<RobotPose> goal_poses_;
    std::vector<std::string> robot_names_, hand_names_;
    std::unordered_map<std::string, Object> objects_;

    int num_collision_checks_ = 0;
    std::atomic<std::uint64_t> environment_version_{0};
    std::string pose_name_;
    std::string instance_type_ = "PlanInstance";

};

// Concrete implementations are provided in dedicated headers.

#endif // MR_PLANNER_INSTANCE_H
