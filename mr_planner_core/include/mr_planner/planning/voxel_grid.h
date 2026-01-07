#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

#include <Eigen/Geometry>
#include <unordered_map>
#include <vector>
#include <optional>
#include <set>
#include <iostream>
#include "mr_planner/planning/pose_hash.h"

#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/array.hpp>

struct Voxel {
    int id;
    Eigen::Vector3i index;
    Eigen::Vector3d center;
};

namespace std {
    template<>
    struct hash<Eigen::Vector3i> {
        size_t operator()(const Eigen::Vector3i& v) const {
            size_t h1 = std::hash<int>()(v.x());
            size_t h2 = std::hash<int>()(v.y());
            size_t h3 = std::hash<int>()(v.z());
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

class VoxelGrid {
public:
    VoxelGrid() = default;

    VoxelGrid(const Eigen::Vector3d& min_bound, const Eigen::Vector3d& max_bound, double resolution)
        : min_bound_(min_bound), max_bound_(max_bound), resolution_(resolution) {
        size_ = ((max_bound_ - min_bound_) / resolution_).cast<int>();
        int id = 0;
        for (int x = 0; x < size_.x(); ++x) {
            for (int y = 0; y < size_.y(); ++y) {
                for (int z = 0; z < size_.z(); ++z) {
                    Eigen::Vector3i idx(x, y, z);
                    Eigen::Vector3d center = min_bound_ + resolution_ * (idx.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5));
                    voxels_.push_back({id++, idx, center});
                    index_map_[idx] = voxels_.back().id;
                    all_indices.push_back(voxels_.back().id);
                }
            }
        }
    }

    VoxelGrid(const VoxelGrid &other)
        : min_bound_(other.min_bound_), max_bound_(other.max_bound_), size_(other.size_), resolution_(other.resolution_),
          voxels_(other.voxels_), index_map_(other.index_map_), vertex_occupied_voxels_map_(other.vertex_occupied_voxels_map_) {}

    VoxelGrid& operator=(const VoxelGrid &other) {
        if (this != &other) {
            min_bound_ = other.min_bound_;
            max_bound_ = other.max_bound_;
            size_ = other.size_;
            resolution_ = other.resolution_;
            voxels_ = other.voxels_;
            index_map_ = other.index_map_;
            vertex_occupied_voxels_map_ = other.vertex_occupied_voxels_map_;
        }
        return *this;
    }

    int getVoxelId(const Eigen::Vector3d& point) const {
        Eigen::Vector3i idx = ((point - min_bound_) / resolution_ - Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>();
        if ((idx.array() < 0).any() || (idx.array() >= size_.array()).any()) {
            std::cout << "Point out of bounds: " << point.transpose() << std::endl;
            return -1;
        }
        auto it = index_map_.find(idx);
        if(it == index_map_.end()) {
            std::cout << "Voxel not found for point: " << point.transpose() << std::endl;
            return -1;
        }
        return it->second;
    }

    double resolution() const { return resolution_; }

    const std::vector<Voxel>& getVoxels() const { return voxels_; }

    void updatePoseOccupiedVoxels(const RobotPose& pose, const std::vector<int>& occupied_voxels) {
        vertex_occupied_voxels_map_[pose] = occupied_voxels;
    }

    std::vector<int> queryPoseOccupiedVoxels(const RobotPose& pose) const {
        auto it = vertex_occupied_voxels_map_.find(pose);
        if (it != vertex_occupied_voxels_map_.end()) {
            // std::cout << "Pose found in voxel occupancy map!\n";
            return it->second;
        }
        std::cout << "Pose not registered in voxel occupancy map!\n";
        return all_indices;
    }

    Eigen::Vector3d min_bound_, max_bound_;
    Eigen::Vector3i size_;
    double resolution_;
    std::vector<Voxel> voxels_;
    std::unordered_map<Eigen::Vector3i, int> index_map_;
    std::vector<int> all_indices;
    std::unordered_map<RobotPose, std::vector<int>, RobotPoseHash> vertex_occupied_voxels_map_;
};

// Eigen serialization helpers
namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, Eigen::Vector3d & v, const unsigned int /*version*/) {
    ar & v.x();
    ar & v.y();
    ar & v.z();
}

template<class Archive>
void serialize(Archive & ar, Eigen::Vector3i & v, const unsigned int /*version*/) {
    ar & v.x();
    ar & v.y();
    ar & v.z();
}

} // namespace serialization
} // namespace boost

// Serialize Voxel
namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, Voxel & v, const unsigned int /*version*/) {
    ar & v.id;
    ar & v.index;
    ar & v.center;
}

} // namespace serialization
} // namespace boost

// Serialize VoxelGrid
namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, VoxelGrid & grid, const unsigned int /*version*/) {
    ar & grid.min_bound_;
    ar & grid.max_bound_;
    ar & grid.size_;
    ar & grid.resolution_;
    ar & grid.voxels_;
    ar & grid.index_map_;
    ar & grid.all_indices;
    ar & grid.vertex_occupied_voxels_map_;
}

} // namespace serialization
} // namespace boost

#endif // VOXEL_GRID_H