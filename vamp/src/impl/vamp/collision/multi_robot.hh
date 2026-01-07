#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/robots/link_mapping.hh>

namespace vamp::collision
{
    struct MultiRobotCollisionFilter
    {
        struct RobotFilter
        {
            std::vector<std::unordered_set<std::string>> per_sphere_objects;
            std::unordered_set<std::string> attachment_objects;

            inline auto ensure_sphere_capacity(std::size_t count) -> void
            {
                if (per_sphere_objects.size() < count)
                {
                    per_sphere_objects.resize(count);
                }
            }

            inline auto active() const noexcept -> bool
            {
                if (not attachment_objects.empty())
                {
                    return true;
                }

                for (const auto &entries : per_sphere_objects)
                {
                    if (not entries.empty())
                    {
                        return true;
                    }
                }

                return false;
            }

            inline auto allow_sphere(std::size_t sphere_index, std::string_view object) -> void
            {
                ensure_sphere_capacity(sphere_index + 1);
                per_sphere_objects[sphere_index].emplace(object);
            }

            inline auto allow_attachment(std::string_view object) -> void
            {
                attachment_objects.emplace(object);
            }

            inline auto allows_sphere(std::size_t sphere_index, std::string_view object) const -> bool
            {
                if (sphere_index >= per_sphere_objects.size())
                {
                    return false;
                }

                const auto &entries = per_sphere_objects[sphere_index];
                if (entries.empty())
                {
                    return false;
                }

                if (entries.find(std::string(object)) != entries.end())
                {
                    return true;
                }

                return entries.find(std::string("*")) != entries.end();
            }

            inline auto allows_attachment(std::string_view object) const -> bool
            {
                if (attachment_objects.empty())
                {
                    return false;
                }

                if (attachment_objects.find(std::string(object)) != attachment_objects.end())
                {
                    return true;
                }

                return attachment_objects.find(std::string("*")) != attachment_objects.end();
            }
        };

        inline auto empty() const noexcept -> bool
        {
            return std::none_of(per_robot.begin(), per_robot.end(), [](const RobotFilter &filter) {
                return filter.active();
            });
        }

        inline auto ensure_robot(std::size_t index, std::size_t sphere_count) -> RobotFilter &
        {
            if (index >= per_robot.size())
            {
                per_robot.resize(index + 1U);
            }

            auto &filter = per_robot[index];
            filter.ensure_sphere_capacity(sphere_count);
            return filter;
        }

        inline auto robot(std::size_t index) noexcept -> RobotFilter *
        {
            if (index >= per_robot.size())
            {
                return nullptr;
            }

            return &per_robot[index];
        }

        inline auto robot(std::size_t index) const noexcept -> const RobotFilter *
        {
            if (index >= per_robot.size())
            {
                return nullptr;
            }

            return &per_robot[index];
        }

        std::vector<RobotFilter> per_robot;
    };

    template <typename RobotT, std::size_t rake>
    struct MultiRobotState
    {
        using RobotType = RobotT;
        using ConfigurationBlock = typename RobotType::template ConfigurationBlock<rake>;

        ConfigurationBlock configuration;
        Eigen::Isometry3f base_transform{Eigen::Isometry3f::Identity()};
        const collision::Attachment<float> *attachment = nullptr;
    };

    template <typename RobotT, std::size_t rake>
    inline auto make_multi_robot_state(
        const typename RobotT::template ConfigurationBlock<rake> &configuration,
        const Eigen::Isometry3f &base_transform = Eigen::Isometry3f::Identity(),
        const collision::Attachment<float> *attachment = nullptr) noexcept
        -> MultiRobotState<RobotT, rake>
    {
        return {configuration, base_transform, attachment};
    }

    struct DebugBody
    {
        std::size_t robot_index = 0;
        std::size_t sphere_index = 0;
        std::size_t link_index = 0;
        bool is_attachment = false;
        std::size_t attachment_index = 0;
        std::string name;
    };

    struct DebugCollisionPose
    {
        // shape: tag describing the primitive source, values: per-lane center (x/y/z), dimensions
        // (l/w/h) and orientation quaternion (qx/qy/qz/qw). Capsules encode l as axial length
        // including caps, cylinders use the raw axial length, w/h as 2*radius; spheres set
        // l=w=h=2*radius and identity orientation.
        std::string shape;
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> z;
        std::vector<float> l;
        std::vector<float> w;
        std::vector<float> h;
        std::vector<float> qx;
        std::vector<float> qy;
        std::vector<float> qz;
        std::vector<float> qw;
    };

    struct DebugLinkLinkCollision
    {
        DebugBody first;
        DebugBody second;
        DebugCollisionPose first_pose;
        DebugCollisionPose second_pose;
    };

    struct DebugLinkObjectCollision
    {
        DebugBody link;
        DebugCollisionPose link_pose;
        std::string object;
        std::optional<DebugCollisionPose> object_pose;
    };

    struct DebugFkccMultiAllResult
    {
        std::vector<DebugLinkLinkCollision> link_link;
        std::vector<DebugLinkObjectCollision> link_object;
    };

    namespace detail
    {
        template <typename Robot>
        inline auto link_info_from_sphere(std::size_t sphere_index) -> std::pair<std::size_t, std::string>
        {
            std::size_t link_index = sphere_index;
            std::string link_name;

            if constexpr (vamp::robots::LinkMapping<Robot>::available)
            {
                const auto &mapping = vamp::robots::LinkMapping<Robot>::sphere_to_link;
                if (sphere_index < mapping.size())
                {
                    link_index = mapping[sphere_index];
                    const auto &names = vamp::robots::LinkMapping<Robot>::link_names;
                    if (link_index < names.size())
                    {
                        link_name = std::string(names[link_index]);
                    }
                }
            }

            if (link_name.empty())
            {
                link_name = std::string("sphere_") + std::to_string(sphere_index);
            }

            return {link_index, link_name};
        }

        template <typename Robot>
        inline auto make_link_body(std::size_t robot_index, std::size_t sphere_index) -> DebugBody
        {
            const auto [link_index, link_name] = link_info_from_sphere<Robot>(sphere_index);

            DebugBody body;
            body.robot_index = robot_index;
            body.sphere_index = sphere_index;
            body.link_index = link_index;
            body.name = link_name;
            return body;
        }

        inline auto make_attachment_body(std::size_t robot_index, std::size_t attachment_index) -> DebugBody
        {
            DebugBody body;
            body.robot_index = robot_index;
            body.sphere_index = attachment_index;
            body.link_index = attachment_index;
            body.is_attachment = true;
            body.attachment_index = attachment_index;
            body.name = std::string("attachment_") + std::to_string(attachment_index);
            return body;
        }

        template <typename ValueT>
        inline auto extract_scalars(const ValueT &value) -> std::vector<float>
        {
            if constexpr (vamp::is_vector<ValueT>::value)
            {
                const auto raw = value.to_array();

                std::vector<float> lanes;
                lanes.reserve(ValueT::num_scalars);
                for (std::size_t i = 0; i < ValueT::num_scalars; ++i)
                {
                    lanes.push_back(static_cast<float>(raw[i]));
                }

                return lanes;
            }
            else
            {
                return {static_cast<float>(value)};
            }
        }

        inline auto set_identity_quaternion(DebugCollisionPose &pose, std::size_t lane_count) -> void
        {
            pose.qx.assign(lane_count, 0.0F);
            pose.qy.assign(lane_count, 0.0F);
            pose.qz.assign(lane_count, 0.0F);
            pose.qw.assign(lane_count, 1.0F);
        }

        template <typename XT, typename YT, typename ZT>
        inline auto make_pose_with_dims(
            std::string shape,
            const XT &x,
            const YT &y,
            const ZT &z,
            const std::vector<float> &l,
            const std::vector<float> &w,
            const std::vector<float> &h) -> DebugCollisionPose
        {
            DebugCollisionPose pose;
            pose.shape = std::move(shape);
            pose.x = extract_scalars(x);
            pose.y = extract_scalars(y);
            pose.z = extract_scalars(z);
            pose.l = l;
            pose.w = w;
            pose.h = h;

            const auto lane_count =
                std::max<std::size_t>({pose.x.size(), pose.y.size(), pose.z.size(), pose.l.size(), pose.w.size(), pose.h.size()});
            set_identity_quaternion(pose, lane_count == 0 ? 1 : lane_count);
            return pose;
        }

        template <typename SpheresT>
        inline auto make_pose_from_sphere_index(const SpheresT &spheres, std::size_t sphere_index)
            -> DebugCollisionPose
        {
            const auto r_lanes = extract_scalars(spheres.r[sphere_index]);
            std::vector<float> dims_l;
            std::vector<float> dims_w;
            std::vector<float> dims_h;
            dims_l.reserve(r_lanes.size());
            dims_w.reserve(r_lanes.size());
            dims_h.reserve(r_lanes.size());
            for (const auto r_value : r_lanes)
            {
                const float diameter = 2.0F * r_value;
                dims_l.push_back(diameter);
                dims_w.push_back(diameter);
                dims_h.push_back(diameter);
            }

            return make_pose_with_dims(
                std::string("sphere"),
                spheres.x[sphere_index],
                spheres.y[sphere_index],
                spheres.z[sphere_index],
                dims_l,
                dims_w,
                dims_h);
        }

        template <typename SphereT>
        inline auto make_pose_from_sphere(const SphereT &sphere, std::string shape = "sphere") -> DebugCollisionPose
        {
            const auto r_lanes = extract_scalars(sphere.r);
            std::vector<float> dims_l;
            std::vector<float> dims_w;
            std::vector<float> dims_h;
            dims_l.reserve(r_lanes.size());
            dims_w.reserve(r_lanes.size());
            dims_h.reserve(r_lanes.size());

            for (const auto r_value : r_lanes)
            {
                const float diameter = 2.0F * r_value;
                dims_l.push_back(diameter);
                dims_w.push_back(diameter);
                dims_h.push_back(diameter);
            }

            return make_pose_with_dims(shape, sphere.x, sphere.y, sphere.z, dims_l, dims_w, dims_h);
        }

        inline auto unit_vector_or(const Eigen::Vector3f &v, const Eigen::Vector3f &fallback) -> Eigen::Vector3f
        {
            const float norm = v.norm();
            if (norm < 1e-6F)
            {
                return fallback;
            }
            return v / norm;
        }

        inline auto quaternion_from_direction(const Eigen::Vector3f &direction) -> Eigen::Quaternionf
        {
            const auto z = unit_vector_or(direction, Eigen::Vector3f::UnitZ());
            Eigen::Vector3f helper = std::abs(z.z()) < 0.9F ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
            auto x = z.cross(helper);
            if (x.squaredNorm() < 1e-8F)
            {
                helper = Eigen::Vector3f::UnitY();
                x = z.cross(helper);
            }
            x.normalize();
            const auto y = z.cross(x).normalized();

            Eigen::Matrix3f basis;
            basis.col(0) = x;
            basis.col(1) = y;
            basis.col(2) = z;

            Eigen::Quaternionf q(basis);
            q.normalize();
            return q;
        }

        template <typename CapsuleT>
        inline auto make_pose_from_capsule(const CapsuleT &capsule, std::string shape, bool include_caps)
            -> DebugCollisionPose
        {
            const auto start_x = extract_scalars(capsule.x1);
            const auto start_y = extract_scalars(capsule.y1);
            const auto start_z = extract_scalars(capsule.z1);
            const auto delta_x = extract_scalars(capsule.xv);
            const auto delta_y = extract_scalars(capsule.yv);
            const auto delta_z = extract_scalars(capsule.zv);
            const auto r_lanes = extract_scalars(capsule.r);

            const std::size_t lane_count = std::max<std::size_t>(
                {start_x.size(), start_y.size(), start_z.size(), delta_x.size(), delta_y.size(), delta_z.size(), r_lanes.size()});

            DebugCollisionPose pose;
            pose.shape = std::move(shape);
            pose.x.reserve(lane_count);
            pose.y.reserve(lane_count);
            pose.z.reserve(lane_count);
            pose.l.reserve(lane_count);
            pose.w.reserve(lane_count);
            pose.h.reserve(lane_count);
            pose.qx.reserve(lane_count);
            pose.qy.reserve(lane_count);
            pose.qz.reserve(lane_count);
            pose.qw.reserve(lane_count);

            for (std::size_t lane = 0; lane < lane_count; ++lane)
            {
                const float sx = (lane < start_x.size()) ? start_x[lane] : start_x.back();
                const float sy = (lane < start_y.size()) ? start_y[lane] : start_y.back();
                const float sz = (lane < start_z.size()) ? start_z[lane] : start_z.back();
                const float dx = (lane < delta_x.size()) ? delta_x[lane] : delta_x.back();
                const float dy = (lane < delta_y.size()) ? delta_y[lane] : delta_y.back();
                const float dz = (lane < delta_z.size()) ? delta_z[lane] : delta_z.back();
                const float r = (lane < r_lanes.size()) ? r_lanes[lane] : r_lanes.back();

                const Eigen::Vector3f delta(dx, dy, dz);
                const float length = delta.norm();
                const Eigen::Vector3f center = Eigen::Vector3f(sx, sy, sz) + 0.5F * delta;
                const auto q = quaternion_from_direction(delta);

                pose.x.push_back(center.x());
                pose.y.push_back(center.y());
                pose.z.push_back(center.z());
                // Encode capsule/cylinder as oriented box-like dimensions: length along its axis, diameter as width/height.
                pose.l.push_back(length + (include_caps ? 2.0F * r : 0.0F));
                pose.w.push_back(2.0F * r);
                pose.h.push_back(2.0F * r);
                pose.qx.push_back(q.x());
                pose.qy.push_back(q.y());
                pose.qz.push_back(q.z());
                pose.qw.push_back(q.w());
            }

            return pose;
        }

        inline auto quaternion_from_axes(
            const Eigen::Vector3f &axis1,
            const Eigen::Vector3f &axis2,
            const Eigen::Vector3f &axis3) -> Eigen::Quaternionf
        {
            auto x = unit_vector_or(axis1, Eigen::Vector3f::UnitX());
            auto y = unit_vector_or(axis2, Eigen::Vector3f::UnitY());
            auto z = x.cross(y);
            if (z.squaredNorm() < 1e-8F)
            {
                y = x.unitOrthogonal();
                z = x.cross(y);
            }
            z.normalize();
            y = z.cross(x).normalized();

            Eigen::Matrix3f basis;
            basis.col(0) = x;
            basis.col(1) = y;
            basis.col(2) = z;

            Eigen::Quaternionf q(basis);
            q.normalize();
            return q;
        }

        template <typename CuboidT>
        inline auto make_pose_from_cuboid(const CuboidT &cuboid, std::string shape) -> DebugCollisionPose
        {
            const auto axis_1_x = extract_scalars(cuboid.axis_1_x);
            const auto axis_1_y = extract_scalars(cuboid.axis_1_y);
            const auto axis_1_z = extract_scalars(cuboid.axis_1_z);
            const auto axis_2_x = extract_scalars(cuboid.axis_2_x);
            const auto axis_2_y = extract_scalars(cuboid.axis_2_y);
            const auto axis_2_z = extract_scalars(cuboid.axis_2_z);
            const auto axis_3_x = extract_scalars(cuboid.axis_3_x);
            const auto axis_3_y = extract_scalars(cuboid.axis_3_y);
            const auto axis_3_z = extract_scalars(cuboid.axis_3_z);
            const auto axis_1_r = extract_scalars(cuboid.axis_1_r);
            const auto axis_2_r = extract_scalars(cuboid.axis_2_r);
            const auto axis_3_r = extract_scalars(cuboid.axis_3_r);

            const auto cx = extract_scalars(cuboid.x);
            const auto cy = extract_scalars(cuboid.y);
            const auto cz = extract_scalars(cuboid.z);

            const std::size_t lane_count = std::max<std::size_t>(
                {cx.size(),
                 cy.size(),
                 cz.size(),
                 axis_1_x.size(),
                 axis_1_y.size(),
                 axis_1_z.size(),
                 axis_2_x.size(),
                 axis_2_y.size(),
                 axis_2_z.size(),
                 axis_3_x.size(),
                 axis_3_y.size(),
                 axis_3_z.size(),
                 axis_1_r.size(),
                 axis_2_r.size(),
                 axis_3_r.size()});

            DebugCollisionPose pose;
            pose.shape = std::move(shape);
            pose.x.reserve(lane_count);
            pose.y.reserve(lane_count);
            pose.z.reserve(lane_count);
            pose.l.reserve(lane_count);
            pose.w.reserve(lane_count);
            pose.h.reserve(lane_count);
            pose.qx.reserve(lane_count);
            pose.qy.reserve(lane_count);
            pose.qz.reserve(lane_count);
            pose.qw.reserve(lane_count);

            for (std::size_t lane = 0; lane < lane_count; ++lane)
            {
                const auto a1 = Eigen::Vector3f(
                    (lane < axis_1_x.size()) ? axis_1_x[lane] : axis_1_x.back(),
                    (lane < axis_1_y.size()) ? axis_1_y[lane] : axis_1_y.back(),
                    (lane < axis_1_z.size()) ? axis_1_z[lane] : axis_1_z.back());
                const auto a2 = Eigen::Vector3f(
                    (lane < axis_2_x.size()) ? axis_2_x[lane] : axis_2_x.back(),
                    (lane < axis_2_y.size()) ? axis_2_y[lane] : axis_2_y.back(),
                    (lane < axis_2_z.size()) ? axis_2_z[lane] : axis_2_z.back());
                const auto a3 = Eigen::Vector3f(
                    (lane < axis_3_x.size()) ? axis_3_x[lane] : axis_3_x.back(),
                    (lane < axis_3_y.size()) ? axis_3_y[lane] : axis_3_y.back(),
                    (lane < axis_3_z.size()) ? axis_3_z[lane] : axis_3_z.back());

                const float l = 2.0F * ((lane < axis_1_r.size()) ? axis_1_r[lane] : axis_1_r.back());
                const float w = 2.0F * ((lane < axis_2_r.size()) ? axis_2_r[lane] : axis_2_r.back());
                const float h = 2.0F * ((lane < axis_3_r.size()) ? axis_3_r[lane] : axis_3_r.back());

                const auto q = quaternion_from_axes(a1, a2, a3);

                pose.x.push_back((lane < cx.size()) ? cx[lane] : cx.back());
                pose.y.push_back((lane < cy.size()) ? cy[lane] : cy.back());
                pose.z.push_back((lane < cz.size()) ? cz[lane] : cz.back());
                pose.l.push_back(l);
                pose.w.push_back(w);
                pose.h.push_back(h);
                pose.qx.push_back(q.x());
                pose.qy.push_back(q.y());
                pose.qz.push_back(q.z());
                pose.qw.push_back(q.w());
            }

            return pose;
        }

        template <typename SpheresT>
        inline auto apply_transform(SpheresT &spheres, const Eigen::Isometry3f &tf) noexcept -> void
        {
            const auto rotation = tf.linear();
            const auto translation = tf.translation();

            const auto rx0 = rotation(0, 0);
            const auto rx1 = rotation(0, 1);
            const auto rx2 = rotation(0, 2);
            const auto ry0 = rotation(1, 0);
            const auto ry1 = rotation(1, 1);
            const auto ry2 = rotation(1, 2);
            const auto rz0 = rotation(2, 0);
            const auto rz1 = rotation(2, 1);
            const auto rz2 = rotation(2, 2);

            auto &x = spheres.x;
            auto &y = spheres.y;
            auto &z = spheres.z;

            auto transformed_x = x * rx0 + y * rx1 + z * rx2;
            auto transformed_y = x * ry0 + y * ry1 + z * ry2;
            auto transformed_z = x * rz0 + y * rz1 + z * rz2;

            transformed_x = transformed_x + translation.x();
            transformed_y = transformed_y + translation.y();
            transformed_z = transformed_z + translation.z();

            x = transformed_x;
            y = transformed_y;
            z = transformed_z;
        }

        template <std::size_t rake>
        inline auto apply_transform(
            std::vector<collision::Sphere<FloatVector<rake>>> &spheres,
            const Eigen::Isometry3f &tf) noexcept -> void
        {
            const auto rotation = tf.linear();
            const auto translation = tf.translation();

            const auto rx0 = rotation(0, 0);
            const auto rx1 = rotation(0, 1);
            const auto rx2 = rotation(0, 2);
            const auto ry0 = rotation(1, 0);
            const auto ry1 = rotation(1, 1);
            const auto ry2 = rotation(1, 2);
            const auto rz0 = rotation(2, 0);
            const auto rz1 = rotation(2, 1);
            const auto rz2 = rotation(2, 2);

            for (auto &sphere : spheres)
            {
                const auto x = sphere.x;
                const auto y = sphere.y;
                const auto z = sphere.z;

                sphere.x = x * rx0 + y * rx1 + z * rx2 + translation.x();
                sphere.y = x * ry0 + y * ry1 + z * ry2 + translation.y();
                sphere.z = x * rz0 + y * rz1 + z * rz2 + translation.z();
            }
        }

        template <typename RowVectorT>
        inline auto rotate_row_left(RowVectorT &row) noexcept -> void
        {
            if constexpr (RowVectorT::num_scalars <= 1)
            {
                return;
            }

            const auto values = row.to_array();
            using Scalar = typename RowVectorT::S::ScalarT;
            std::array<Scalar, RowVectorT::num_scalars> rotated{};
            std::copy_n(values.begin(), RowVectorT::num_scalars, rotated.begin());
            std::rotate(rotated.begin(), rotated.begin() + 1, rotated.end());
            row = RowVectorT(rotated);
        }

        template <typename VectorT>
        inline auto rotate_vector_rows_left(VectorT &vector) noexcept -> void
        {
            if constexpr (VectorT::num_rows == 0)
            {
                return;
            }
            else
            {
                [&]<std::size_t... I>(std::index_sequence<I...>)
                {
                    (..., rotate_row_left(vector.row(I)));
                }(std::make_index_sequence<VectorT::num_rows>{});
            }
        }

        template <typename SpheresT>
        inline auto rotate_robot_spheres_left(SpheresT &spheres) noexcept -> void
        {
            rotate_vector_rows_left(spheres.x);
            rotate_vector_rows_left(spheres.y);
            rotate_vector_rows_left(spheres.z);
            rotate_vector_rows_left(spheres.r);
        }

        template <std::size_t rake>
        inline auto rotate_attachment_spheres_left(
            std::vector<collision::Sphere<FloatVector<rake>>> &attachments) noexcept -> void
        {
            for (auto &sphere : attachments)
            {
                rotate_vector_rows_left(sphere.x);
                rotate_vector_rows_left(sphere.y);
                rotate_vector_rows_left(sphere.z);
                rotate_vector_rows_left(sphere.r);
            }
        }

        template <std::size_t rake, typename TestFn, typename RotateFn>
        inline auto run_lane_shifts(TestFn &&test, RotateFn &&rotate) noexcept -> bool
        {
            for (std::size_t shift = 0; shift < rake; ++shift)
            {
                if (test())
                {
                    return true;
                }

                if constexpr (rake > 1)
                {
                    if (shift + 1 < rake)
                    {
                        rotate();
                    }
                }
            }

            return false;
        }

        inline auto is_identity_transform(const Eigen::Isometry3f &tf) noexcept -> bool
        {
            return tf.linear().isIdentity(1e-6F) && tf.translation().isZero(1e-6F);
        }

        inline auto transform_environment_to_robot_frame(
            collision::Environment<float> &environment,
            const Eigen::Isometry3f &base_transform) noexcept -> void
        {
            const Eigen::Matrix3f rotation_inv = base_transform.linear().transpose();
            const Eigen::Vector3f translation = base_transform.translation();

            auto transform_point = [&](float x, float y, float z) noexcept -> Eigen::Vector3f
            {
                Eigen::Vector3f result{x, y, z};
                result -= translation;
                result = rotation_inv * result;
                return result;
            };

            auto transform_direction = [&](float x, float y, float z) noexcept -> Eigen::Vector3f
            {
                return rotation_inv * Eigen::Vector3f{x, y, z};
            };

            for (auto &sphere : environment.spheres)
            {
                const Eigen::Vector3f point = transform_point(sphere.x, sphere.y, sphere.z);
                sphere.x = point.x();
                sphere.y = point.y();
                sphere.z = point.z();
                const float distance = point.norm();
                sphere.min_distance = distance - sphere.r;
            }

            auto transform_segment = [&](auto &segment) noexcept
            {
                const Eigen::Vector3f start(segment.x1, segment.y1, segment.z1);
                const Eigen::Vector3f end = start + Eigen::Vector3f(segment.xv, segment.yv, segment.zv);

                const Eigen::Vector3f start_local = transform_point(start.x(), start.y(), start.z());
                const Eigen::Vector3f end_local = transform_point(end.x(), end.y(), end.z());
                const Eigen::Vector3f delta = end_local - start_local;

                segment.x1 = start_local.x();
                segment.y1 = start_local.y();
                segment.z1 = start_local.z();
                segment.xv = delta.x();
                segment.yv = delta.y();
                segment.zv = delta.z();
            };

            for (auto &capsule : environment.capsules)
            {
                transform_segment(capsule);
                capsule.min_distance = capsule.compute_min_distance();
            }
            for (auto &capsule : environment.z_aligned_capsules)
            {
                transform_segment(capsule);
                capsule.min_distance = capsule.compute_min_distance();
            }
            for (auto &cylinder : environment.cylinders)
            {
                transform_segment(cylinder);
                cylinder.min_distance = cylinder.compute_min_distance();
            }

            auto transform_cuboid = [&](auto &cuboid) noexcept
            {
                const Eigen::Vector3f center = transform_point(cuboid.x, cuboid.y, cuboid.z);
                cuboid.x = center.x();
                cuboid.y = center.y();
                cuboid.z = center.z();

                const Eigen::Vector3f axis1 = transform_direction(cuboid.axis_1_x, cuboid.axis_1_y, cuboid.axis_1_z);
                cuboid.axis_1_x = axis1.x();
                cuboid.axis_1_y = axis1.y();
                cuboid.axis_1_z = axis1.z();

                const Eigen::Vector3f axis2 = transform_direction(cuboid.axis_2_x, cuboid.axis_2_y, cuboid.axis_2_z);
                cuboid.axis_2_x = axis2.x();
                cuboid.axis_2_y = axis2.y();
                cuboid.axis_2_z = axis2.z();

                const Eigen::Vector3f axis3 = transform_direction(cuboid.axis_3_x, cuboid.axis_3_y, cuboid.axis_3_z);
                cuboid.axis_3_x = axis3.x();
                cuboid.axis_3_y = axis3.y();
                cuboid.axis_3_z = axis3.z();
            };

            for (auto &cuboid : environment.cuboids)
            {
                transform_cuboid(cuboid);
                cuboid.min_distance = cuboid.compute_min_distance();
            }
            for (auto &cuboid : environment.z_aligned_cuboids)
            {
                transform_cuboid(cuboid);
                cuboid.min_distance = cuboid.compute_min_distance();
            }

            environment.sort();
        }

        template <std::size_t rake>
        struct EnvironmentTransformCache
        {
            struct Entry
            {
                Eigen::Isometry3f transform;
                collision::Environment<FloatVector<rake>> environment;
            };

            inline static thread_local std::vector<Entry> storage{};

            inline static auto clear() noexcept -> void
            {
                storage.clear();
            }
        };

        template <std::size_t rake>
        inline auto clear_environment_transform_cache() noexcept -> void
        {
            EnvironmentTransformCache<rake>::clear();
        }

        template <std::size_t rake>
        inline auto environment_entry_for_transform(
            const collision::Environment<FloatVector<rake>> &source_environment,
            const Eigen::Isometry3f &base_transform)
            -> typename EnvironmentTransformCache<rake>::Entry &
        {
            auto &storage = EnvironmentTransformCache<rake>::storage;
            for (auto &entry : storage)
            {
                if (entry.transform.linear().isApprox(base_transform.linear(), 1e-6F) &&
                    entry.transform.translation().isApprox(base_transform.translation(), 1e-6F))
                {
                    return entry;
                }
            }

            collision::Environment<FloatVector<rake>> transformed =
                is_identity_transform(base_transform) ?
                    collision::Environment<FloatVector<rake>>(source_environment) :
                    [&]
                    {
                        collision::Environment<float> float_environment(source_environment);
                        transform_environment_to_robot_frame(float_environment, base_transform);
                        return collision::Environment<FloatVector<rake>>(float_environment);
                    }();

            storage.push_back({base_transform, std::move(transformed)});
            return storage.back();
        }

        template <typename Robot, std::size_t rake>
        inline auto collisions_allowed(
            const MultiRobotCollisionFilter::RobotFilter &filter,
            const collision::Environment<FloatVector<rake>> &environment,
            const typename Robot::template ConfigurationBlock<rake> &configuration,
            bool attachment_present) noexcept -> bool;

        template <typename RobotStateT, std::size_t rake>
        inline auto process_robot_state(
            const collision::Environment<FloatVector<rake>> &source_environment,
            const RobotStateT &state,
            typename RobotStateT::RobotType::template Spheres<rake> &spheres,
            std::vector<collision::Sphere<FloatVector<rake>>> &attachment_spheres,
            const MultiRobotCollisionFilter::RobotFilter *filter) noexcept -> bool
        {
            using Robot = typename RobotStateT::RobotType;

            auto &entry = detail::environment_entry_for_transform<rake>(
                source_environment, state.base_transform);
            auto &robot_environment = entry.environment;

            auto saved_attachments = robot_environment.attachments;

            if (state.attachment)
            {
                robot_environment.attachments.emplace(*state.attachment);
            }
            else
            {
                robot_environment.attachments.reset();
            }

            bool valid = state.attachment ?
                             Robot::template fkcc_attach<rake>(robot_environment, state.configuration) :
                             Robot::template fkcc<rake>(robot_environment, state.configuration);

            if (not valid)
            {
                if (filter == nullptr)
                {
                    robot_environment.attachments = std::move(saved_attachments);
                    return false;
                }

                if (not collisions_allowed<Robot, rake>(
                        *filter,
                        robot_environment,
                        state.configuration,
                        state.attachment != nullptr))
                {
                    robot_environment.attachments = std::move(saved_attachments);
                    return false;
                }

                valid = true;
            }

            Robot::sphere_fk(state.configuration, spheres);
            apply_transform(spheres, state.base_transform);

            attachment_spheres.clear();
            if (robot_environment.attachments)
            {
                attachment_spheres = robot_environment.attachments->posed_spheres;
                apply_transform<rake>(attachment_spheres, state.base_transform);
            }

            robot_environment.attachments = std::move(saved_attachments);

            return true;
        }

        template <typename RobotStateT, std::size_t rake>
        inline auto process_robot_state_self(
            const RobotStateT &state,
            typename RobotStateT::RobotType::template Spheres<rake> &spheres,
            std::vector<collision::Sphere<FloatVector<rake>>> &attachment_spheres) noexcept -> bool
        {
            using Robot = typename RobotStateT::RobotType;

            collision::Environment<FloatVector<rake>> robot_environment;

            if (state.attachment)
            {
                robot_environment.attachments.emplace(*state.attachment);
            }

            const bool valid = state.attachment ?
                                  Robot::template fkcc_attach<rake>(robot_environment, state.configuration) :
                                  Robot::template fkcc<rake>(robot_environment, state.configuration);

            if (not valid)
            {
                // fkcc_multi_* returns false when any SIMD lane detects a collision.
                return false;
            }

            Robot::sphere_fk(state.configuration, spheres);
            apply_transform(spheres, state.base_transform);

            attachment_spheres.clear();
            if (robot_environment.attachments)
            {
                attachment_spheres = robot_environment.attachments->posed_spheres;
                apply_transform<rake>(attachment_spheres, state.base_transform);
            }

            return true;
        }

        template <typename SpheresA, typename SpheresB>
        inline auto spheres_collide(const SpheresA &a, const SpheresB &b) noexcept -> bool
        {
            constexpr std::size_t count_a = std::remove_reference_t<decltype(a.x)>::num_rows;
            constexpr std::size_t count_b = std::remove_reference_t<decltype(b.x)>::num_rows;

            // Each coordinate component is a FloatVector<rake>, so every arithmetic
            // operation below runs across all SIMD lanes in lockstep. The outer loops
            // are only over the small, fixed set of robot spheres.
            for (std::size_t i = 0; i < count_a; ++i)
            {
                const auto ax = a.x[i];
                const auto ay = a.y[i];
                const auto az = a.z[i];
                const auto ar = a.r[i];

                for (std::size_t j = 0; j < count_b; ++j)
                {
                    const auto bx = b.x[j];
                    const auto by = b.y[j];
                    const auto bz = b.z[j];
                    const auto br = b.r[j];

                    if (not collision::sphere_sphere_sql2(ax, ay, az, ar, bx, by, bz, br).test_zero())
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        template <std::size_t rake>
        inline auto spheres_collide(
            const collision::Sphere<FloatVector<rake>> &a,
            const collision::Sphere<FloatVector<rake>> &b) noexcept -> bool
        {
            return not collision::sphere_sphere_sql2(a, b).test_zero();
        }

        template <typename AttachmentVec, typename SpheresT>
        inline auto attachments_vs_spheres(const AttachmentVec &attachments, const SpheresT &spheres) noexcept -> bool
        {
            if (attachments.empty())
            {
                return false;
            }

            constexpr std::size_t count = std::remove_reference_t<decltype(spheres.x)>::num_rows;

            for (const auto &attachment : attachments)
            {
                for (std::size_t i = 0; i < count; ++i)
                {
                    if (not collision::sphere_sphere_sql2(
                                attachment,
                                spheres.x[i],
                                spheres.y[i],
                                spheres.z[i],
                                spheres.r[i])
                                .test_zero())
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        template <typename AttachmentVecA, typename AttachmentVecB>
        inline auto attachments_vs_attachments(
            const AttachmentVecA &a,
            const AttachmentVecB &b) noexcept -> bool
        {
            if (a.empty() || b.empty())
            {
                return false;
            }

            for (const auto &sphere_a : a)
            {
                for (const auto &sphere_b : b)
                {
                    if (not collision::sphere_sphere_sql2(sphere_a, sphere_b).test_zero())
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        template <std::size_t rake, bool cross, typename SpheresA, typename SpheresB>
        inline auto spheres_collide_variant(const SpheresA &a, const SpheresB &b) noexcept -> bool
        {
            if constexpr (not cross)
            {
                return spheres_collide(a, b);
            }
            else
            {
                auto rotated = b;
                return run_lane_shifts<rake>(
                    [&]() { return spheres_collide(a, rotated); },
                    [&]() { rotate_robot_spheres_left(rotated); });
            }
        }

        template <std::size_t rake, bool cross, typename AttachmentVec, typename SpheresT>
        inline auto attachments_vs_spheres_variant(
            const AttachmentVec &attachments,
            const SpheresT &spheres) noexcept -> bool
        {
            if (attachments.empty())
            {
                return false;
            }

            if constexpr (not cross)
            {
                return attachments_vs_spheres(attachments, spheres);
            }
            else
            {
                auto rotated = spheres;
                return run_lane_shifts<rake>(
                    [&]() { return attachments_vs_spheres(attachments, rotated); },
                    [&]() { rotate_robot_spheres_left(rotated); });
            }
        }

        template <std::size_t rake, bool cross, typename AttachmentVecA, typename AttachmentVecB>
        inline auto attachments_vs_attachments_variant(
            const AttachmentVecA &a,
            const AttachmentVecB &b) noexcept -> bool
        {
            if (a.empty() || b.empty())
            {
                return false;
            }

            if constexpr (not cross)
            {
                return attachments_vs_attachments(a, b);
            }
            else
            {
                auto rotated = b;
                return run_lane_shifts<rake>(
                    [&]() { return attachments_vs_attachments(a, rotated); },
                    [&]() { rotate_attachment_spheres_left<rake>(rotated); });
            }
        }

        template <std::size_t rake, bool cross, std::size_t I, std::size_t J, typename Tuple>
        inline auto spheres_cross_pair_variant(const Tuple &spheres) noexcept -> bool
        {
            if constexpr (I >= std::tuple_size_v<Tuple>)
            {
                return false;
            }
            else if constexpr (J >= std::tuple_size_v<Tuple>)
            {
                return false;
            }
            else
            {
                if (spheres_collide_variant<rake, cross>(std::get<I>(spheres), std::get<J>(spheres)))
                {
                    return true;
                }

                return spheres_cross_pair_variant<rake, cross, I, J + 1>(spheres);
            }
        }

        template <std::size_t rake, bool cross, std::size_t I, typename Tuple>
        inline auto spheres_cross_variant(const Tuple &spheres) noexcept -> bool
        {
            if constexpr (I >= std::tuple_size_v<Tuple>)
            {
                return false;
            }
            else
            {
                if (spheres_cross_pair_variant<rake, cross, I, I + 1>(spheres))
                {
                    return true;
                }

                return spheres_cross_variant<rake, cross, I + 1>(spheres);
            }
        }

        template <std::size_t rake, bool cross, std::size_t I, std::size_t J, typename AttachTuple, typename SphereTuple>
        inline auto attachments_vs_spheres_pair_variant(
            const AttachTuple &attachments,
            const SphereTuple &spheres) noexcept -> bool
        {
            if constexpr (J >= std::tuple_size_v<SphereTuple>)
            {
                return false;
            }
            else
            {
                if constexpr (J != I)
                {
                    if (attachments_vs_spheres_variant<rake, cross>(std::get<I>(attachments), std::get<J>(spheres)))
                    {
                        return true;
                    }
                }

                return attachments_vs_spheres_pair_variant<rake, cross, I, J + 1>(attachments, spheres);
            }
        }

        template <std::size_t rake, bool cross, std::size_t I, typename AttachTuple, typename SphereTuple>
        inline auto attachments_vs_spheres_all_variant(
            const AttachTuple &attachments,
            const SphereTuple &spheres) noexcept -> bool
        {
            if constexpr (I >= std::tuple_size_v<AttachTuple>)
            {
                return false;
            }
            else
            {
                if (attachments_vs_spheres_pair_variant<rake, cross, I, 0>(attachments, spheres))
                {
                    return true;
                }

                return attachments_vs_spheres_all_variant<rake, cross, I + 1>(attachments, spheres);
            }
        }

        template <std::size_t rake, bool cross, std::size_t I, std::size_t J, typename AttachTuple>
        inline auto attachments_vs_attachments_pair_variant(const AttachTuple &attachments) noexcept -> bool
        {
            if constexpr (J >= std::tuple_size_v<AttachTuple>)
            {
                return false;
            }
            else
            {
                if constexpr (J > I)
                {
                    if (attachments_vs_attachments_variant<rake, cross>(std::get<I>(attachments), std::get<J>(attachments)))
                    {
                        return true;
                    }
                }

                return attachments_vs_attachments_pair_variant<rake, cross, I, J + 1>(attachments);
            }
        }

        template <std::size_t rake, bool cross, std::size_t I, typename AttachTuple>
        inline auto attachments_vs_attachments_all_variant(const AttachTuple &attachments) noexcept -> bool
        {
            if constexpr (I >= std::tuple_size_v<AttachTuple>)
            {
                return false;
            }
            else
            {
                if (attachments_vs_attachments_pair_variant<rake, cross, I, 0>(attachments))
                {
                    return true;
                }

                return attachments_vs_attachments_all_variant<rake, cross, I + 1>(attachments);
            }
        }

        template <typename Tuple>
        inline auto attachments_present(const Tuple &attachments) noexcept -> bool
        {
            bool has_attachment = false;
            [&]<std::size_t... I>(std::index_sequence<I...>)
            {
                ((has_attachment = has_attachment || not std::get<I>(attachments).empty()), ...);
            }(std::make_index_sequence<std::tuple_size_v<Tuple>>{});

            return has_attachment;
        }

        template <std::size_t rake>
        inline auto find_environment_object_pose(
            const collision::Environment<FloatVector<rake>> &environment,
            std::string_view name) -> std::optional<DebugCollisionPose>
        {
            for (const auto &sphere : environment.spheres)
            {
                if (sphere.name == name)
                {
                    return make_pose_from_sphere(sphere);
                }
            }

            for (const auto &capsule : environment.capsules)
            {
                if (capsule.name == name)
                {
                    return make_pose_from_capsule(capsule, "capsule", true);
                }
            }

            for (const auto &capsule : environment.z_aligned_capsules)
            {
                if (capsule.name == name)
                {
                    return make_pose_from_capsule(capsule, "z_aligned_capsule", true);
                }
            }

            for (const auto &cylinder : environment.cylinders)
            {
                if (cylinder.name == name)
                {
                    return make_pose_from_capsule(cylinder, "cylinder", false);
                }
            }

            for (const auto &cuboid : environment.cuboids)
            {
                if (cuboid.name == name)
                {
                    return make_pose_from_cuboid(cuboid, "cuboid");
                }
            }

            for (const auto &cuboid : environment.z_aligned_cuboids)
            {
                if (cuboid.name == name)
                {
                    return make_pose_from_cuboid(cuboid, "z_aligned_cuboid");
                }
            }

            // Heightfields and CAPT pointclouds currently have no pose reporting in debug output.
            return std::nullopt;
        }

        inline auto set_contains(const std::unordered_set<std::string> &set, std::string_view value) -> bool
        {
            if (set.empty())
            {
                return false;
            }

            if (set.find(std::string(value)) != set.end())
            {
                return true;
            }

            return set.find(std::string("*")) != set.end();
        }

        template <typename Robot, std::size_t rake>
        inline auto collisions_allowed(
            const MultiRobotCollisionFilter::RobotFilter &filter,
            const collision::Environment<FloatVector<rake>> &environment,
            const typename Robot::template ConfigurationBlock<rake> &configuration,
            bool attachment_present) noexcept -> bool
        {
            if (not filter.active())
            {
                return false;
            }

            const auto debug = Robot::template fkcc_debug<rake>(environment, configuration);

            bool has_collision = false;

            for (std::size_t sphere_index = 0; sphere_index < debug.first.size(); ++sphere_index)
            {
                const auto &objects = debug.first[sphere_index];
                if (objects.empty())
                {
                    continue;
                }

                has_collision = true;
                for (const auto &object : objects)
                {
                    if (filter.per_sphere_objects.size() <= sphere_index)
                    {
                        return false;
                    }

                    const auto &allowed = filter.per_sphere_objects[sphere_index];
                    if (not set_contains(allowed, object))
                    {
                        return false;
                    }
                }
            }

            if (not debug.second.empty())
            {
                return false;
            }

            if (attachment_present && environment.attachments)
            {
                for (const auto &sphere : environment.attachments->posed_spheres)
                {
                    const auto collisions = sphere_environment_get_collisions(
                        environment,
                        sphere.x,
                        sphere.y,
                        sphere.z,
                        sphere.r);

                    if (collisions.empty())
                    {
                        continue;
                    }

                    has_collision = true;
                    for (const auto &object : collisions)
                    {
                        if (not set_contains(filter.attachment_objects, object))
                        {
                            return false;
                        }
                    }
                }
            }

            return has_collision;
        }

        template <typename RobotStateT, std::size_t rake>
        inline auto process_robot_state_debug(
            const collision::Environment<FloatVector<rake>> &source_environment,
            const RobotStateT &state,
            std::size_t robot_index,
            const MultiRobotCollisionFilter::RobotFilter *filter,
            typename RobotStateT::RobotType::template Spheres<rake> &spheres,
            std::vector<collision::Sphere<FloatVector<rake>>> &attachment_spheres,
            DebugFkccMultiAllResult &result) noexcept -> void
        {
            using Robot = typename RobotStateT::RobotType;

            auto &entry = detail::environment_entry_for_transform<rake>(
                source_environment, state.base_transform);
            auto &robot_environment = entry.environment;

            auto saved_attachments = robot_environment.attachments;

            if (state.attachment)
            {
                robot_environment.attachments.emplace(*state.attachment);
            }
            else
            {
                robot_environment.attachments.reset();
            }

            const auto debug = Robot::template fkcc_debug<rake>(robot_environment, state.configuration);

            if (state.attachment)
            {
                Robot::template fkcc_attach<rake>(robot_environment, state.configuration);
            }
            else
            {
                Robot::template fkcc<rake>(robot_environment, state.configuration);
            }

            Robot::sphere_fk(state.configuration, spheres);
            apply_transform(spheres, state.base_transform);

            for (std::size_t sphere_index = 0; sphere_index < debug.first.size(); ++sphere_index)
            {
                const auto &objects = debug.first[sphere_index];
                if (objects.empty())
                {
                    continue;
                }

                const auto link_pose = make_pose_from_sphere_index(spheres, sphere_index);

                for (const auto &object : objects)
                {
                    const bool allowed = (filter != nullptr) && filter->allows_sphere(sphere_index, object);
                    if (not allowed)
                    {
                        result.link_object.push_back(
                            {make_link_body<Robot>(robot_index, sphere_index),
                             link_pose,
                             object,
                             find_environment_object_pose<rake>(source_environment, object)});
                    }
                }
            }

            for (const auto &self_collision : debug.second)
            {
                result.link_link.push_back(
                    {make_link_body<Robot>(robot_index, self_collision.first),
                     make_link_body<Robot>(robot_index, self_collision.second),
                     make_pose_from_sphere_index(spheres, self_collision.first),
                     make_pose_from_sphere_index(spheres, self_collision.second)});
            }

            attachment_spheres.clear();
            if (robot_environment.attachments)
            {
                std::vector<std::pair<std::size_t, std::string>> attachment_hits;

                attachment_spheres = robot_environment.attachments->posed_spheres;

                for (std::size_t attachment_index = 0; attachment_index < attachment_spheres.size(); ++attachment_index)
                {
                    const auto &sphere = attachment_spheres[attachment_index];
                    const auto collisions = sphere_environment_get_collisions(
                        robot_environment,
                        sphere.x,
                        sphere.y,
                        sphere.z,
                        sphere.r);

                    if (collisions.empty())
                    {
                        continue;
                    }

                    for (const auto &object : collisions)
                    {
                        const bool allowed = (filter != nullptr) && filter->allows_attachment(object);
                        if (not allowed)
                        {
                            attachment_hits.emplace_back(attachment_index, object);
                        }
                    }
                }

                apply_transform<rake>(attachment_spheres, state.base_transform);

                for (const auto &hit : attachment_hits)
                {
                    const auto attachment_index = hit.first;
                    result.link_object.push_back(
                        {make_attachment_body(robot_index, attachment_index),
                         make_pose_from_sphere(attachment_spheres[attachment_index]),
                         hit.second,
                         find_environment_object_pose<rake>(source_environment, hit.second)});
                }
            }

            robot_environment.attachments = std::move(saved_attachments);
        }

        template <std::size_t rake, typename... States>
        inline auto fkcc_multi_all_attach_impl(
            const collision::Environment<FloatVector<rake>> &environment,
            const MultiRobotCollisionFilter *filter,
            const States &... states) noexcept -> bool
        {
            static_assert(sizeof...(States) > 0, "fkcc_multi_all_attach requires at least one robot state");

            auto spheres = std::tuple<typename States::RobotType::template Spheres<rake>...>{};
            auto attachments = std::tuple<
                std::conditional_t<true, std::vector<collision::Sphere<FloatVector<rake>>>, States>...>{};

            bool valid = true;

            [&]<std::size_t... I>(std::index_sequence<I...>)
            {
                (..., ([&]
                       {
                           using StateT = std::tuple_element_t<I, std::tuple<States...>>;
                           const auto *robot_filter =
                               (filter != nullptr) ? filter->robot(I) : nullptr;

                           if (not process_robot_state<StateT, rake>(
                                   environment,
                                   std::get<I>(std::forward_as_tuple(states...)),
                                   std::get<I>(spheres),
                                   std::get<I>(attachments),
                                   robot_filter))
                           {
                               valid = false;
                           }
                       }()));
            }(std::index_sequence_for<States...>{});

            if (not valid)
            {
                // fkcc_multi_self returns false when any SIMD lane collides.
                return false;
            }

            // Kick off the recursive robot/robot sphere sweep starting at tuple index 0.
            // Same pairwise robot sweep as above, but on the self-checked robot set.
            if (spheres_cross_variant<rake, false, 0>(spheres))
            {
                return false;
            }

            if (attachments_present(attachments))
            {
                if (attachments_vs_spheres_all_variant<rake, false, 0>(attachments, spheres))
                {
                    return false;
                }

                if (attachments_vs_attachments_all_variant<rake, false, 0>(attachments))
                {
                    return false;
                }
            }

            return true;
        }

        template <std::size_t rake, bool cross, typename... States>
        inline auto fkcc_multi_self_attach_impl_dispatch(const States &... states) noexcept -> bool
        {
            static_assert(sizeof...(States) > 0, "fkcc_multi_self_attach requires at least one robot state");

            auto spheres = std::tuple<typename States::RobotType::template Spheres<rake>...>{};
            auto attachments = std::tuple<
                std::conditional_t<true, std::vector<collision::Sphere<FloatVector<rake>>>, States>...>{};

            bool valid = true;

            [&]<std::size_t... I>(std::index_sequence<I...>)
            {
                (..., ([&]
                       {
                           using StateT = std::tuple_element_t<I, std::tuple<States...>>;
                           if (not detail::process_robot_state_self<StateT, rake>(
                                   std::get<I>(std::forward_as_tuple(states...)),
                                   std::get<I>(spheres),
                                   std::get<I>(attachments)))
                           {
                               valid = false;
                           }
                       }()));
            }(std::index_sequence_for<States...>{});

            if (not valid)
            {
                return false;
            }

            if (spheres_cross_variant<rake, cross, 0>(spheres))
            {
                return false;
            }

            if (attachments_present(attachments))
            {
                if (attachments_vs_spheres_all_variant<rake, cross, 0>(attachments, spheres))
                {
                    return false;
                }

                if (attachments_vs_attachments_all_variant<rake, cross, 0>(attachments))
                {
                    return false;
                }
            }

            return true;
        }

        template <std::size_t rake, typename... States>
        inline auto fkcc_multi_self_attach_impl(const States &... states) noexcept -> bool
        {
            return fkcc_multi_self_attach_impl_dispatch<rake, false>(states...);
        }

        template <std::size_t rake, typename... States>
        inline auto fkcc_multi_cross_attach_impl(const States &... states) noexcept -> bool
        {
            return fkcc_multi_self_attach_impl_dispatch<rake, true>(states...);
        }

        template <
            typename StateTuple,
            std::size_t rake,
            std::size_t I,
            std::size_t J,
            typename SphereTuple,
            typename AttachmentTuple>
        inline auto collect_cross_robot_collisions(
            const SphereTuple &spheres,
            const AttachmentTuple &attachments,
            DebugFkccMultiAllResult &result) noexcept -> void
        {
            if constexpr (I >= std::tuple_size_v<SphereTuple>)
            {
                return;
            }
            else if constexpr (J >= std::tuple_size_v<SphereTuple>)
            {
                collect_cross_robot_collisions<StateTuple, rake, I + 1, I + 2>(
                    spheres, attachments, result);
            }
            else
            {
                using StateA = std::tuple_element_t<I, StateTuple>;
                using StateB = std::tuple_element_t<J, StateTuple>;
                using RobotA = typename StateA::RobotType;
                using RobotB = typename StateB::RobotType;

                const auto &spheres_a = std::get<I>(spheres);
                const auto &spheres_b = std::get<J>(spheres);

                constexpr std::size_t count_a = std::remove_reference_t<decltype(spheres_a.x)>::num_rows;
                constexpr std::size_t count_b = std::remove_reference_t<decltype(spheres_b.x)>::num_rows;

                for (std::size_t a = 0; a < count_a; ++a)
                {
                    const auto ax = spheres_a.x[a];
                    const auto ay = spheres_a.y[a];
                    const auto az = spheres_a.z[a];
                    const auto ar = spheres_a.r[a];

                    for (std::size_t b = 0; b < count_b; ++b)
                    {
                        if (not collision::sphere_sphere_sql2(
                                    ax,
                                    ay,
                                    az,
                                    ar,
                                    spheres_b.x[b],
                                    spheres_b.y[b],
                                    spheres_b.z[b],
                                    spheres_b.r[b])
                                    .test_zero())
                        {
                            result.link_link.push_back(
                                {make_link_body<RobotA>(I, a),
                                 make_link_body<RobotB>(J, b),
                                 make_pose_from_sphere_index(spheres_a, a),
                                 make_pose_from_sphere_index(spheres_b, b)});
                        }
                    }
                }

                const auto &attachments_a = std::get<I>(attachments);
                const auto &attachments_b = std::get<J>(attachments);

                for (std::size_t attachment_index = 0; attachment_index < attachments_a.size(); ++attachment_index)
                {
                    const auto &attachment = attachments_a[attachment_index];
                    for (std::size_t b = 0; b < count_b; ++b)
                    {
                        if (not collision::sphere_sphere_sql2(
                                    attachment,
                                    spheres_b.x[b],
                                    spheres_b.y[b],
                                    spheres_b.z[b],
                                    spheres_b.r[b])
                                    .test_zero())
                        {
                            result.link_link.push_back(
                                {make_attachment_body(I, attachment_index),
                                 make_link_body<RobotB>(J, b),
                                 make_pose_from_sphere(attachment),
                                 make_pose_from_sphere_index(spheres_b, b)});
                        }
                    }

                    for (std::size_t other_attachment_index = 0;
                         other_attachment_index < attachments_b.size();
                         ++other_attachment_index)
                    {
                        if (not collision::sphere_sphere_sql2(
                                    attachment, attachments_b[other_attachment_index])
                                    .test_zero())
                        {
                            result.link_link.push_back(
                                {make_attachment_body(I, attachment_index),
                                 make_attachment_body(J, other_attachment_index),
                                 make_pose_from_sphere(attachment),
                                 make_pose_from_sphere(attachments_b[other_attachment_index])});
                        }
                    }
                }

                for (std::size_t attachment_index = 0; attachment_index < attachments_b.size(); ++attachment_index)
                {
                    const auto &attachment = attachments_b[attachment_index];
                    for (std::size_t a = 0; a < count_a; ++a)
                    {
                        if (not collision::sphere_sphere_sql2(
                                    attachment,
                                    spheres_a.x[a],
                                    spheres_a.y[a],
                                    spheres_a.z[a],
                                    spheres_a.r[a])
                                    .test_zero())
                        {
                            result.link_link.push_back(
                                {make_attachment_body(J, attachment_index),
                                 make_link_body<RobotA>(I, a),
                                 make_pose_from_sphere(attachment),
                                 make_pose_from_sphere_index(spheres_a, a)});
                        }
                    }
                }

                collect_cross_robot_collisions<StateTuple, rake, I, J + 1>(spheres, attachments, result);
            }
        }

        template <std::size_t rake, typename... States>
        inline auto debug_fkcc_multi_all_impl(
            const collision::Environment<FloatVector<rake>> &environment,
            const MultiRobotCollisionFilter *filter,
            const States &... states) noexcept -> DebugFkccMultiAllResult
        {
            static_assert(sizeof...(States) > 0, "debug_fkcc_multi_all requires at least one robot state");

            DebugFkccMultiAllResult result;

            auto spheres = std::tuple<typename States::RobotType::template Spheres<rake>...>{};
            auto attachments = std::tuple<
                std::conditional_t<true, std::vector<collision::Sphere<FloatVector<rake>>>, States>...>{};

            [&]<std::size_t... I>(std::index_sequence<I...>)
            {
                (..., detail::process_robot_state_debug<std::tuple_element_t<I, std::tuple<States...>>, rake>(
                          environment,
                          std::get<I>(std::forward_as_tuple(states...)),
                          I,
                          (filter != nullptr) ? filter->robot(I) : nullptr,
                          std::get<I>(spheres),
                          std::get<I>(attachments),
                          result));
            }(std::index_sequence_for<States...>{});

            using StateTuple = std::tuple<States...>;
            collect_cross_robot_collisions<StateTuple, rake, 0, 1>(spheres, attachments, result);

            return result;
        }
    }  // namespace detail

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_all_attach(
        const collision::Environment<FloatVector<rake>> &environment,
        const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_all_attach_impl<rake>(environment, nullptr, states...);
    }

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_all_attach(
        const collision::Environment<FloatVector<rake>> &environment,
        const MultiRobotCollisionFilter &filter,
        const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_all_attach_impl<rake>(environment, &filter, states...);
    }

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_all(
        const collision::Environment<FloatVector<rake>> &environment,
        const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_all_attach_impl<rake>(environment, nullptr, states...);
    }

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_all(
        const collision::Environment<FloatVector<rake>> &environment,
        const MultiRobotCollisionFilter &filter,
        const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_all_attach_impl<rake>(environment, &filter, states...);
    }

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_self_attach(const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_self_attach_impl<rake>(states...);
    }

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_self(const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_self_attach_impl<rake>(states...);
    }

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_cross_attach(const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_cross_attach_impl<rake>(states...);
    }

    template <std::size_t rake, typename... States>
    inline auto fkcc_multi_cross(const States &... states) noexcept -> bool
    {
        return detail::fkcc_multi_cross_attach_impl<rake>(states...);
    }

    template <std::size_t rake, typename... States>
    inline auto debug_fkcc_multi_all(
        const collision::Environment<FloatVector<rake>> &environment,
        const States &... states) noexcept -> DebugFkccMultiAllResult
    {
        return detail::debug_fkcc_multi_all_impl<rake>(environment, nullptr, states...);
    }

    template <std::size_t rake, typename... States>
    inline auto debug_fkcc_multi_all(
        const collision::Environment<FloatVector<rake>> &environment,
        const MultiRobotCollisionFilter &filter,
        const States &... states) noexcept -> DebugFkccMultiAllResult
    {
        return detail::debug_fkcc_multi_all_impl<rake>(environment, &filter, states...);
    }
}  // namespace vamp::collision
