#ifndef MR_PLANNER_ROADMAP_H
#define MR_PLANNER_ROADMAP_H

#include "mr_planner/core/instance.h" // Include the abstract problem instance definition
#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/core/graph.h"
#include "mr_planner/planning/pose_hash.h"
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <thread>
#include <execution> // for parallel algorithms

/* For hash reference: RobotPose class here
struct RobotPose {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & robot_id;
        ar & robot_name;
        ar & joint_values;
        ar & hand_values;
    }
    int robot_id;
    // RobotMode mode;
    std::string robot_name; // same as group name in moveit
    std::vector<double> joint_values;
    std::vector<double> hand_values;

    bool operator==(const RobotPose &other) const {
        return robot_id == other.robot_id &&
               robot_name == other.robot_name &&
               joint_values == other.joint_values &&
               hand_values == other.hand_values;
    }
};
*/

struct AStar {
    std::shared_ptr<Vertex> vertex;
    std::shared_ptr<AStar> parent;
    double g; // cost from start to current node
    double h; // heuristic cost from current node to goal

    double f() const { return g + h; }

    bool operator==(const AStar &other) const {
        return vertex->id == other.vertex->id;
    }

    AStar(std::shared_ptr<Vertex>, double g, double h, std::shared_ptr<AStar> parent) : vertex(vertex), g(g), h(h), parent(parent) {}
    AStar() : vertex(nullptr), g(0), h(0), parent(nullptr) {}
};

class CompareAStar {
public:
    bool operator()(const AStar &a, const AStar &b) {
        return a.f() >= b.f();
    }
};

class CompareEdge {
public:
    bool operator()(const std::pair<double, std::shared_ptr<Vertex>> &a, const std::pair<double, std::shared_ptr<Vertex>> &b) {
        return a.first > b.first;
    }
};

class RoadMap {
public:
    RoadMap(std::shared_ptr<PlanInstance> instance, int robot_id);

    void setInstance(std::shared_ptr<PlanInstance> instance);

    void setBuildEnabled(bool enabled);

    bool buildEnabled() const { return build_enabled_; }

    void setNumSamples(int num_samples) { num_samples_ = num_samples; }
    void setMaxDist(double max_dist) { max_dist_ = max_dist; }

    void buildRoadmap();

    bool sampleConditionally(std::shared_ptr<Vertex> &new_sample);

    bool validateMotion(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v);

    bool connected(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v);

    std::shared_ptr<Graph> getRoadmap();

    void updateVertexCollisionMap(const std::pair<RobotPose, RobotPose> &vertex_pair, bool collision);

    void updateEdgeCollisionMap(const std::pair<PoseEdge, PoseEdge> &edge_pair, bool collision);

    void updateTargetCollisionMap(const std::pair<RobotPose, PoseEdge> &vertex_edge_pair, bool collision);

    void updateEdgeOpeningMap(const PoseEdgeUndirected &edge, bool opening);

    void updateRepresentitiveMap(const RobotPose &pose, const std::shared_ptr<Vertex> &representitive);

    void updateEnvironmentCollisionMap(const std::pair<RobotPose, RobotPose> &vertex_pair, bool collision);

    int queryVertexCollisionMap(const std::pair<RobotPose, RobotPose> &vertex_pair) const;

    int queryEdgeCollisionMap(const std::pair<PoseEdge, PoseEdge> &edge_pair) const;

    int queryTargetCollisionMap(const std::pair<RobotPose, PoseEdge> &vertex_edge_pair) const;

    const std::unordered_map<PoseEdgeUndirected, bool, PoseEdgeUndirectedHash>& queryEdgeOpeningMap() const;

    std::unordered_map<RobotPose, int, RobotPoseHash> getStartGoalMap();

    void updateStartGoalMap(const RobotPose &pose, int id);

    void analyzePoseDistribution() const;

    // SPARS
    bool connect(const RobotPose &u, const RobotPose &v);

    std::vector<std::shared_ptr<Vertex>> findVisibleGuards(const std::shared_ptr<Vertex> &new_sample);

    bool addGuard(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards);

    bool addConnector(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards);

    bool addInterface(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards);

    bool addShortcut(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards);

    std::shared_ptr<Vertex> computeRepresentitive(const RobotPose &pose);

    void updateRepresentitiveForAll(std::shared_ptr<Vertex> &sample);

    bool share_interface(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v);

    std::vector<std::shared_ptr<Vertex>> max_spanner_path(const std::shared_ptr<Vertex> &v, const std::shared_ptr<Vertex> &v1, const std::shared_ptr<Vertex> &v2);

    std::vector<std::shared_ptr<Vertex>> interface_support(const std::shared_ptr<Vertex> &v, const std::shared_ptr<Vertex> &v1);

    std::vector<std::shared_ptr<Vertex>> computeShortestPath(const std::shared_ptr<Vertex> &q, const std::vector<std::shared_ptr<Vertex>> &support_poses);

    void addPath(const std::shared_ptr<Vertex> &v1, const std::vector<std::shared_ptr<Vertex>> pi_d, const std::shared_ptr<Vertex> &v2);

    // PRM*
    void addSampleToPRMStar(const std::shared_ptr<Vertex> &new_sample);

    void connectAllEdges();

    Eigen::Vector3d getEndEffectorPositionFromPose(const RobotPose &pose) const;

    void printToFile(const std::string& filename) const;

protected:
    int robot_id_ = -1;
    int num_samples_ = 5000;
    double max_dist_ = 2.0; // guard radius
    double epsilon_ = 2.0; // Support parameter for SPARS
    double t = 1.5; // Asympototic optimality factor
    int max_failures_ = 5000; // For SPARS
    double eta_ = 8.0; // For PRM*
    std::shared_ptr<Graph> roadmap_;
    std::shared_ptr<Graph> prm_star_;
    std::shared_ptr<PlanInstance> instance_;
    std::vector<RobotPose> start_poses_;
    std::vector<RobotPose> goal_poses_;
    mutable boost::shared_mutex vertex_map_mutex_;
    std::unordered_map<std::pair<RobotPose, RobotPose>, bool, VertexPairHash> vertex_collision_map_;
    mutable boost::shared_mutex edge_map_mutex_;
    std::unordered_map<std::pair<PoseEdge, PoseEdge>, bool, EdgePairHash> edge_collision_map_;
    mutable boost::shared_mutex target_map_mutex_;
    std::unordered_map<std::pair<RobotPose, PoseEdge>, bool, TargetHash> vertex_edge_map_;
    std::unordered_map<RobotPose, int, RobotPoseHash> start_goal_map_;
    std::unordered_map<PoseEdgeUndirected, bool, PoseEdgeUndirectedHash> edge_opening_map_; // false means the edge is not part of the SPARS
    std::unordered_map<RobotPose, std::shared_ptr<Vertex>, RobotPoseHash> representitive_map_; // Store the representitive of each dense roadmap vertex
    std::unordered_map<std::pair<RobotPose, RobotPose>, bool, VertexPairHash> environment_collision_map_; // Cache for environment collision checking
    bool build_enabled_ = true;

private:
    RoadMap() {} // private default ctor for deserialization

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & robot_id_;
        ar & num_samples_;
        ar & max_dist_;
        ar & epsilon_;
        ar & t;
        ar & max_failures_;
        ar & eta_;
        ar & roadmap_;
        ar & prm_star_;
        ar & start_poses_;
        ar & goal_poses_;
        ar & vertex_collision_map_;
        ar & edge_collision_map_;
        ar & vertex_edge_map_;
        ar & start_goal_map_;
        ar & edge_opening_map_;
        ar & representitive_map_;
        ar & environment_collision_map_;
    }
};

#endif // MR_PLANNER_ROADMAP_H
