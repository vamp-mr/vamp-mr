#pragma once

#include <cassert>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "mr_planner/planning/pose_hash.h"

#if MR_PLANNER_WITH_VAMP
#include <Eigen/Dense>
#include <nigh/impl/space_base.hpp>
#include <nigh/kdtree_batch.hpp>
#include <nigh/metric/lp.hpp>
#include <nigh/metric/space.hpp>
#include <nigh/nigh_forward.hpp>
#endif

namespace mr_planner::planning
{
inline constexpr std::size_t kLinearScanThreshold = 64;
inline constexpr std::size_t kCompositeShortlistK = 64;
inline constexpr std::size_t kArenaChunkPoints = 4096;
inline constexpr bool kKDTreeAvailable =
#if MR_PLANNER_WITH_VAMP
    true;
#else
    false;
#endif

inline void pack_joints_l1(const RobotPose &pose, float *out, std::size_t dims)
{
    assert(out != nullptr);
    assert(pose.joint_values.size() == dims);
    for (std::size_t i = 0; i < dims; ++i)
    {
        out[i] = static_cast<float>(pose.joint_values[i]);
    }
}

class ChunkedPointArena
{
public:
    void reset(std::size_t dims, std::size_t points_per_chunk = kArenaChunkPoints)
    {
        dims_ = dims;
        points_per_chunk_ = std::max<std::size_t>(points_per_chunk, 1);
        chunks_.clear();
        chunk_used_points_ = 0;
    }

    [[nodiscard]] auto dims() const noexcept -> std::size_t { return dims_; }

    [[nodiscard]] auto alloc_point() -> float *
    {
        assert(dims_ > 0);
        if (chunks_.empty() || chunk_used_points_ >= points_per_chunk_)
        {
            chunks_.push_back(std::unique_ptr<float[]>(new float[dims_ * points_per_chunk_]));
            chunk_used_points_ = 0;
        }
        float *base = chunks_.back().get();
        float *ptr = base + (chunk_used_points_ * dims_);
        ++chunk_used_points_;
        return ptr;
    }

private:
    std::size_t dims_{0};
    std::size_t points_per_chunk_{kArenaChunkPoints};
    std::vector<std::unique_ptr<float[]>> chunks_;
    std::size_t chunk_used_points_{0};
};

#if MR_PLANNER_WITH_VAMP
using NNKey = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 1>, Eigen::Unaligned>;
#endif
}  // namespace mr_planner::planning

#if MR_PLANNER_WITH_VAMP
namespace unc::robotics::nigh::metric
{
    template <>
    struct Space<mr_planner::planning::NNKey, LP<1>> : impl::SpaceBase<impl::Dynamic>
    {
    private:
        using Base = impl::SpaceBase<impl::Dynamic>;

    public:
        using Type = mr_planner::planning::NNKey;
        using Metric = LP<1>;
        using Distance = float;

        using Base::Base;

        static auto isValid(const Type &v) -> bool { return v.allFinite(); }

        static auto coeff(const Type &v, std::size_t i) -> float
        {
            assert(static_cast<Eigen::Index>(i) < v.size());
            return v(static_cast<Eigen::Index>(i));
        }

        auto distance(const Type &a, const Type &b) const -> Distance
        {
            const unsigned n = Base::dimensions();
            assert(a.size() == static_cast<Eigen::Index>(n));
            assert(b.size() == static_cast<Eigen::Index>(n));
            Distance sum = 0.0F;
            for (unsigned i = 0; i < n; ++i)
            {
                sum += std::abs(a(static_cast<Eigen::Index>(i)) - b(static_cast<Eigen::Index>(i)));
            }
            return sum;
        }
    };
}  // namespace unc::robotics::nigh::metric
#endif

namespace mr_planner::planning
{
template <typename PayloadT>
class PoseKDTreeIndex
{
public:
    using Payload = PayloadT;

    void reset(std::size_t dims, std::size_t points_per_chunk = kArenaChunkPoints)
    {
        dims_ = dims;
        payloads_.clear();
        arena_.reset(dims, points_per_chunk);

#if MR_PLANNER_WITH_VAMP
        using SpaceT = unc::robotics::nigh::metric::Space<NNKey, unc::robotics::nigh::metric::LP<1>>;
        tree_ = std::make_unique<TreeT>(SpaceT(static_cast<unsigned>(dims_)), NodeKeyT(dims_));
#else
        (void)dims_;
#endif
    }

    [[nodiscard]] auto available() const noexcept -> bool { return kKDTreeAvailable; }
    [[nodiscard]] auto dims() const noexcept -> std::size_t { return dims_; }
    [[nodiscard]] auto size() const noexcept -> std::size_t { return payloads_.size(); }

    auto clear() -> void
    {
        payloads_.clear();
        arena_.reset(dims_);
#if MR_PLANNER_WITH_VAMP
        if (tree_)
        {
            tree_->clear();
        }
#endif
    }

    auto insert(const float *point, const Payload &payload) -> bool
    {
        if (!kKDTreeAvailable || dims_ == 0 || point == nullptr)
        {
            return false;
        }
#if MR_PLANNER_WITH_VAMP
        if (!tree_)
        {
            return false;
        }
        float *stored = arena_.alloc_point();
        std::memcpy(stored, point, dims_ * sizeof(float));
        const std::size_t index = payloads_.size();
        payloads_.push_back(payload);
        tree_->insert(NodeT{index, stored});
        return true;
#else
        (void)point;
        (void)payload;
        return false;
#endif
    }

    auto nearest(const float *query, Payload *out_payload, float *out_dist = nullptr) const -> bool
    {
        if (!kKDTreeAvailable || dims_ == 0 || query == nullptr || out_payload == nullptr)
        {
            return false;
        }
#if MR_PLANNER_WITH_VAMP
        if (!tree_ || payloads_.empty())
        {
            return false;
        }

        auto res = tree_->nearest(NNKey(query, static_cast<Eigen::Index>(dims_)));
        if (!res)
        {
            return false;
        }
        const auto &[node, dist] = *res;
        if (node.index >= payloads_.size())
        {
            return false;
        }
        *out_payload = payloads_[node.index];
        if (out_dist != nullptr)
        {
            *out_dist = dist;
        }
        return true;
#else
        (void)out_dist;
        return false;
#endif
    }

    [[nodiscard]] auto nearest_k(const float *query, std::size_t k) const -> std::vector<std::pair<Payload, float>>
    {
        std::vector<std::pair<Payload, float>> out;
        if (!kKDTreeAvailable || dims_ == 0 || query == nullptr || k == 0)
        {
            return out;
        }
#if MR_PLANNER_WITH_VAMP
        if (!tree_ || payloads_.empty())
        {
            return out;
        }
        k = std::min(k, payloads_.size());
        std::vector<std::pair<NodeT, float>> results;
        results.reserve(k);
        tree_->nearest(results, NNKey(query, static_cast<Eigen::Index>(dims_)), k);
        out.reserve(results.size());
        for (const auto &[node, dist] : results)
        {
            if (node.index >= payloads_.size())
            {
                continue;
            }
            out.emplace_back(payloads_[node.index], dist);
        }
        return out;
#else
        return out;
#endif
    }

private:
    std::size_t dims_{0};
    ChunkedPointArena arena_;
    std::vector<Payload> payloads_;

#if MR_PLANNER_WITH_VAMP
    struct NodeT
    {
        std::size_t index{0};
        float *ptr{nullptr};
    };

    struct NodeKeyT
    {
        explicit NodeKeyT(std::size_t dims_in = 0) : dims(dims_in) {}

        auto operator()(const NodeT &node) const noexcept -> NNKey
        {
            return NNKey(node.ptr, static_cast<Eigen::Index>(dims));
        }

        std::size_t dims{0};
    };

    using TreeT = unc::robotics::nigh::Nigh<
        NodeT,
        unc::robotics::nigh::metric::Space<NNKey, unc::robotics::nigh::metric::LP<1>>,
        NodeKeyT,
        unc::robotics::nigh::NoThreadSafety,
        unc::robotics::nigh::KDTreeBatch<128>>;

    std::unique_ptr<TreeT> tree_;
#endif
};
}  // namespace mr_planner::planning
