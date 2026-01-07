#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <vamp/collision/shapes.hh>
#include <vector>

namespace vamp::collision
{
    template <typename DataT>
    struct Attachment
    {
        Attachment(const Eigen::Transform<DataT, 3, Eigen::Isometry> &tf) noexcept : tf(std::move(tf))
        {
        }

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Eigen::Transform<float, 3, Eigen::Isometry> &tf) noexcept
          : Attachment(convert_transform<float>(tf))
        {
        }

        Attachment(const Attachment &) = default;

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Attachment<float> &o) noexcept : Attachment(convert_transform<float>(o.tf))
        {
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }
        }

        template <typename OtherDataT,
                  typename = std::enable_if_t<std::is_same_v<DataT, float> && not std::is_same_v<OtherDataT, DataT>>>
        Attachment(const Attachment<OtherDataT> &o) noexcept : Attachment(convert_transform<OtherDataT>(o.tf))
        {
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }
        }

        template <typename DT = DataT, typename = std::enable_if_t<std::is_same_v<DT, float>>>
        void add_cuboid(const Cuboid<float> &cuboid,
                        float sphere_radius = -1.0F,
                        std::size_t min_spheres = 0) noexcept
        {
            constexpr float kEpsilon = 1e-6F;

            const Eigen::Vector3f center(cuboid.x, cuboid.y, cuboid.z);
            const Eigen::Vector3f axes[] = {
                Eigen::Vector3f(cuboid.axis_1_x, cuboid.axis_1_y, cuboid.axis_1_z),
                Eigen::Vector3f(cuboid.axis_2_x, cuboid.axis_2_y, cuboid.axis_2_z),
                Eigen::Vector3f(cuboid.axis_3_x, cuboid.axis_3_y, cuboid.axis_3_z)};
            const float half_lengths[] = {cuboid.axis_1_r, cuboid.axis_2_r, cuboid.axis_3_r};

            std::size_t longest_axis = 0U;
            if (half_lengths[1] > half_lengths[longest_axis])
            {
                longest_axis = 1U;
            }
            if (half_lengths[2] > half_lengths[longest_axis])
            {
                longest_axis = 2U;
            }

            const float half_length = half_lengths[longest_axis];
            Eigen::Vector3f direction = axes[longest_axis];
            const float dir_norm = direction.norm();
            if (dir_norm <= kEpsilon)
            {
                return;
            }
            direction /= dir_norm;

            const float cross_radius = std::max(
                half_lengths[(longest_axis + 1U) % 3U],
                half_lengths[(longest_axis + 2U) % 3U]);
            float radius = (sphere_radius > 0.0F) ? std::min(sphere_radius, cross_radius) : cross_radius;
            if (radius <= kEpsilon)
            {
                return;
            }

            append_linear_spheres(center, direction, half_length, radius, min_spheres);
        }

        template <typename DT = DataT, typename = std::enable_if_t<std::is_same_v<DT, float>>>
        void add_cylinder(const Cylinder<float> &cylinder,
                          float sphere_radius = -1.0F,
                          std::size_t min_spheres = 0) noexcept
        {
            constexpr float kEpsilon = 1e-6F;

            Eigen::Vector3f axis(cylinder.xv, cylinder.yv, cylinder.zv);
            const float axis_length = axis.norm();
            if (axis_length <= kEpsilon)
            {
                return;
            }

            Eigen::Vector3f direction = axis / axis_length;
            const Eigen::Vector3f start(cylinder.x1, cylinder.y1, cylinder.z1);
            const Eigen::Vector3f center = start + direction * (axis_length * 0.5F);

            const float radius_limit = std::min(cylinder.r, axis_length * 0.5F);
            float radius = (sphere_radius > 0.0F) ? std::min(sphere_radius, radius_limit) : radius_limit;
            if (radius <= kEpsilon)
            {
                return;
            }

            append_linear_spheres(center, direction, axis_length * 0.5F, radius, min_spheres);
        }

        std::vector<Sphere<DataT>> spheres;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        Eigen::Transform<DataT, 3, Eigen::Isometry> tf;

        inline void pose(const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) const noexcept
        {
            const auto &n_tf = p_tf * tf;

            posed_spheres.resize(spheres.size());
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                Eigen::Matrix<DataT, 3, 1> sp(s.x, s.y, s.z);
                auto tfs = n_tf * sp;
                posed_spheres[i] = Sphere<DataT>(tfs[0], tfs[1], tfs[2], s.r);
            }
        }

    private:
        template <typename DT = DataT, typename = std::enable_if_t<std::is_same_v<DT, float>>>
        void append_linear_spheres(const Eigen::Vector3f &center,
                                   const Eigen::Vector3f &direction,
                                   float half_length,
                                   float radius,
                                   std::size_t min_spheres) noexcept
        {
            constexpr float kEpsilon = 1e-6F;
            if (radius <= kEpsilon)
            {
                return;
            }

            Eigen::Vector3f dir = direction;
            const float dir_norm = dir.norm();
            if (dir_norm <= kEpsilon)
            {
                return;
            }
            dir /= dir_norm;

            const float axis_length = half_length * 2.0F;
            std::size_t count = 1U;
            if (axis_length > (2.0F * radius + kEpsilon))
            {
                const float ratio = axis_length / (2.0F * radius);
                count = static_cast<std::size_t>(std::ceil(ratio));
                if (count < 2U)
                {
                    count = 2U;
                }
            }
            if (min_spheres > 0U && count < min_spheres)
            {
                count = min_spheres;
            }

            if (count == 1U)
            {
                spheres.emplace_back(center.x(), center.y(), center.z(), radius);
                return;
            }

            const float usable = std::max(0.0F, axis_length - 2.0F * radius);
            const float step = (count > 1U) ? (usable / static_cast<float>(count - 1U)) : 0.0F;
            const float start = -half_length + radius;

            for (std::size_t i = 0U; i < count; ++i)
            {
                const float offset = start + step * static_cast<float>(i);
                const Eigen::Vector3f position = center + dir * offset;
                spheres.emplace_back(position.x(), position.y(), position.z(), radius);
            }
        }

        template <typename FromDataT>
        static auto convert_transform(const Eigen::Transform<FromDataT, 3, Eigen::Isometry> &input) noexcept
            -> Eigen::Transform<DataT, 3, Eigen::Isometry>
        {
            Eigen::Transform<DataT, 3, Eigen::Isometry> output;
            const auto &in_matrix = input.matrix();
            auto &out_matrix = output.matrix();

            for (int row = 0; row < 4; ++row)
            {
                for (int col = 0; col < 4; ++col)
                {
                    out_matrix(row, col) = detail::cast_component<DataT>(in_matrix(row, col));
                }
            }

            return output;
        }
    };
}  // namespace vamp::collision
