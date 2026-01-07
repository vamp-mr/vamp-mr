#ifndef MR_PLANNER_POSE_HASH_H
#define MR_PLANNER_POSE_HASH_H
#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/utility.hpp> // for std::pair
#include <boost/serialization/string.hpp>
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
#include <set>
#include <string>
#include <utility>

// Helper to hash combine like boost
inline void hash_combine(std::size_t &seed, const std::size_t &val) {
    seed ^= val + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

/*
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

    bool operator==(const RobotPose& other) const {
        if (robot_id != other.robot_id || robot_name != other.robot_name)
            return false;

        if (joint_values.size() != other.joint_values.size() ||
            hand_values.size() != other.hand_values.size())
            return false;

        for (size_t i = 0; i < joint_values.size(); ++i) {
            if (std::abs(joint_values[i] - other.joint_values[i]) > 0.01)
                return false;
        }

        for (size_t i = 0; i < hand_values.size(); ++i) {
            if (std::abs(hand_values[i] - other.hand_values[i]) > 0.01)
                return false;
        }

        return true;
    }

    RobotPose() {}
};

struct RobotPoseHash {
    std::size_t operator()(const RobotPose &pose) const {
        std::size_t seed = std::hash<int>{}(pose.robot_id);
        hash_combine(seed, std::hash<std::string>{}(pose.robot_name));

        auto quantize = [](double val) {
            return static_cast<int>(std::round(val * 100)); // 1e-2 precision
        };

        for (double val : pose.joint_values) {
            hash_combine(seed, std::hash<int>{}(quantize(val)));
        }
        for (double val : pose.hand_values) {
            hash_combine(seed, std::hash<int>{}(quantize(val)));
        }

        return seed;
    }
};
*/

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

    // Define the equality operator
    bool operator==(const RobotPose& other) const {
        return robot_id == other.robot_id &&
               robot_name == other.robot_name &&
               joint_values == other.joint_values &&
               hand_values == other.hand_values;
    }

};

struct RobotPoseHash {
    std::size_t operator()(const RobotPose &pose) const {
        std::size_t seed = std::hash<int>{}(pose.robot_id);
        hash_combine(seed, std::hash<std::string>{}(pose.robot_name));
        for (double val : pose.joint_values) {
            hash_combine(seed, std::hash<double>{}(val));
        }
        for (double val : pose.hand_values) {
            hash_combine(seed, std::hash<double>{}(val));
        }
        return seed;
    }
};

class PoseEdge {
public:
    RobotPose start;
    RobotPose end;

    PoseEdge() {}
    PoseEdge(const RobotPose &start, const RobotPose &end) : start(start), end(end) {}

    bool operator==(const PoseEdge &other) const {
        return start == other.start && end == other.end;
    }

    // Serialization helper
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & start;
        ar & end;
    }
};

struct PoseEdgeHash {
    std::size_t operator()(const PoseEdge &edge) const {
        RobotPoseHash poseHasher;
        std::size_t seed = poseHasher(edge.start);
        hash_combine(seed, poseHasher(edge.end));
        return seed;
    }
};

// For building voxel maps
class PoseEdgeUndirected {
public:
    RobotPose p1;
    RobotPose p2;

    PoseEdgeUndirected() {}
    PoseEdgeUndirected(const RobotPose &pose1, const RobotPose &pose2)
        : p1(pose1), p2(pose2) {}

    bool operator==(const PoseEdgeUndirected &other) const {
        return (p1 == other.p1 && p2 == other.p2) ||
               (p1 == other.p2 && p2 == other.p1);
    }

    // Serialization helper
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & p1;
        ar & p2;
    }
};

struct PoseEdgeUndirectedHash {
    std::size_t operator()(const PoseEdgeUndirected &edge) const {
        RobotPoseHash pose_hasher;
        std::size_t h1 = pose_hasher(edge.p1);
        std::size_t h2 = pose_hasher(edge.p2);

        if (h1 > h2) std::swap(h1, h2);
        std::size_t seed = h1;
        hash_combine(seed, h2);
        return seed;
    }
};

struct VertexPairHash {
    std::size_t operator()(const std::pair<RobotPose, RobotPose> &pair) const {
        RobotPoseHash hasher;
        std::size_t h1 = hasher(pair.first);
        std::size_t h2 = hasher(pair.second);
        return h1 < h2 ? h1 ^ (h2 << 1) : h2 ^ (h1 << 1); // symmetric
    }
};

struct EdgePairHash {
    std::size_t operator()(const std::pair<PoseEdge, PoseEdge> &pair) const {
        PoseEdgeHash hasher;
        std::size_t h1 = hasher(pair.first);
        std::size_t h2 = hasher(pair.second);
        return h1 < h2 ? h1 ^ (h2 << 1) : h2 ^ (h1 << 1); // symmetric
    }
};

struct TargetHash {
    std::size_t operator()(const std::pair<RobotPose, PoseEdge> &pair) const {
        RobotPoseHash poseHasher;
        PoseEdgeHash edgeHasher;
        std::size_t h1 = poseHasher(pair.first);
        std::size_t h2 = edgeHasher(pair.second);
        return h1 < h2 ? h1 ^ (h2 << 1) : h2 ^ (h1 << 1); // symmetric
    }
};

struct StateHash {
    std::size_t operator()(const std::pair<RobotPose, int> &state) const {
        std::size_t seed = RobotPoseHash{}(state.first);
        hash_combine(seed, std::hash<int>{}(state.second));
        return seed;
    }
};

#endif // MR_PLANNER_POSE_HASH_H
