#include <array>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/collision/multi_robot.hh>
#include <vamp/robots/gp4.hh>

namespace vc = vamp::collision;

using Robot = vamp::robots::GP4;
constexpr std::size_t rake = vamp::FloatVectorWidth;
using Block = Robot::template ConfigurationBlock<rake>;
using Row = Block::RowT;

inline auto pack_configuration(const std::array<float, Robot::dimension> &values) -> Block
{
    Block block = Block::zero_vector();
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        block[i] = Row::fill(values[i]);
    }
    return block;
}

inline auto make_environment() -> vc::Environment<vamp::FloatVector<rake>>
{
    vc::Environment<float> environment;

    environment.spheres.emplace_back(0.0F, 0.0F, 0.2F, 0.25F);
    environment.spheres.back().name = "center_sphere";

    environment.spheres.emplace_back(-0.35F, 0.2F, 0.25F, 0.12F);
    environment.spheres.back().name = "offset_sphere";

    environment.sort();
    return vc::Environment<vamp::FloatVector<rake>>(environment);
}

inline auto print_body(const vc::DebugBody &body, const std::vector<std::string> &labels) -> void
{
    const std::string &label = (body.robot_index < labels.size()) ? labels[body.robot_index] : std::string("robot");
    std::cout << label << " (index " << body.robot_index << "): ";

    if (body.is_attachment)
    {
        std::cout << "attachment[" << body.attachment_index << "]";
    }
    else
    {
        std::cout << "link[" << body.link_index << "] (" << body.name << ")";
    }

    std::cout << " sphere_idx=" << body.sphere_index;
}

inline auto print_pose(const vc::DebugCollisionPose &pose, std::string_view label, std::string_view indent = "    ")
    -> void
{
    auto lane_count = std::size_t{0};
    auto update_lane_count = [&](const std::vector<float> &values) { lane_count = std::max(lane_count, values.size()); };
    update_lane_count(pose.x);
    update_lane_count(pose.y);
    update_lane_count(pose.z);
    update_lane_count(pose.l);
    update_lane_count(pose.w);
    update_lane_count(pose.h);
    update_lane_count(pose.qx);
    update_lane_count(pose.qy);
    update_lane_count(pose.qz);
    update_lane_count(pose.qw);

    if (lane_count == 0)
    {
        lane_count = 1;
    }

    auto value_at = [](const std::vector<float> &values, std::size_t lane) -> std::optional<float>
    {
        if (lane < values.size())
        {
            return values[lane];
        }
        return std::nullopt;
    };

    for (std::size_t lane = 0; lane < lane_count; ++lane)
    {
        std::cout << indent << label;
        if (lane_count > 1)
        {
            std::cout << " lane " << lane;
        }
        std::cout << " (" << pose.shape << ")";

        if (auto x = value_at(pose.x, lane))
        {
            const auto y = value_at(pose.y, lane).value_or(0.0F);
            const auto z = value_at(pose.z, lane).value_or(0.0F);
            std::cout << " pos=(" << *x << ", " << y << ", " << z << ")";
        }

        if (auto l = value_at(pose.l, lane))
        {
            const auto w = value_at(pose.w, lane).value_or(0.0F);
            const auto h = value_at(pose.h, lane).value_or(0.0F);
            std::cout << " size(l,w,h)=(" << *l << ", " << w << ", " << h << ")";
        }

        if (auto qx = value_at(pose.qx, lane))
        {
            const auto qy = value_at(pose.qy, lane).value_or(0.0F);
            const auto qz = value_at(pose.qz, lane).value_or(0.0F);
            const auto qw = value_at(pose.qw, lane).value_or(1.0F);
            std::cout << " q=(" << *qx << ", " << qy << ", " << qz << ", " << qw << ")";
        }

        std::cout << "\n";
    }
}

inline auto print_result(const vc::DebugFkccMultiAllResult &result, const std::vector<std::string> &labels) -> void
{
    if (result.link_link.empty() && result.link_object.empty())
    {
        std::cout << "  collision-free\n";
        return;
    }

    for (const auto &collision : result.link_link)
    {
        std::cout << "  link-link: ";
        print_body(collision.first, labels);
        std::cout << " <-> ";
        print_body(collision.second, labels);
        std::cout << "\n";
        print_pose(collision.first_pose, "first");
        print_pose(collision.second_pose, "second");
    }

    for (const auto &collision : result.link_object)
    {
        std::cout << "  link-object: ";
        print_body(collision.link, labels);
        std::cout << " vs object '" << collision.object << "'\n";
        print_pose(collision.link_pose, "link");
        if (collision.object_pose)
        {
            print_pose(*collision.object_pose, "object");
        }
    }
}

int main()
{
    std::vector<std::string> robot_labels = {"arm_a", "arm_b"};

    auto environment = make_environment();

    Eigen::Isometry3f base_a = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f base_b = Eigen::Isometry3f::Identity();
    base_b.translate(Eigen::Vector3f(0.35F, -0.05F, 0.0F));

    vc::Attachment<float> attachment(Eigen::Isometry3f::Identity());
    attachment.spheres.emplace_back(0.0F, 0.0F, 0.12F, 0.06F);

    // Hardcoded pose pair: intentionally place the arms close together and near the center sphere.
    std::array<float, Robot::dimension> config_a = {0.0F, -1.4F, 0.2F, -1.5F, 0.2F, 0.0F};
    std::array<float, Robot::dimension> config_b = {0.0F, -1.3F, -0.1F, -1.6F, -0.1F, 0.0F};

    auto state_a = vc::make_multi_robot_state<Robot, rake>(pack_configuration(config_a), base_a, &attachment);
    auto state_b = vc::make_multi_robot_state<Robot, rake>(pack_configuration(config_b), base_b);

    std::cout << "Deterministic pair:\n";
    const auto deterministic = vc::debug_fkcc_multi_all<rake>(environment, state_a, state_b);
    print_result(deterministic, robot_labels);

    // Randomized samples around nominal to show reporting in diverse poses.
    std::mt19937 rng(7U);
    std::uniform_real_distribution<float> joint_noise(-0.25F, 0.25F);
    const std::array<float, Robot::dimension> nominal = {0.0F, -1.3F, 0.0F, -1.6F, 0.0F, 0.0F};

    std::cout << "\nRandom samples:\n";
    for (int sample = 0; sample < 5; ++sample)
    {
        std::array<float, Robot::dimension> sample_a = nominal;
        std::array<float, Robot::dimension> sample_b = nominal;

        for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
        {
            sample_a[joint] += joint_noise(rng);
            sample_b[joint] += joint_noise(rng);
        }

        auto random_state_a =
            vc::make_multi_robot_state<Robot, rake>(pack_configuration(sample_a), base_a, &attachment);
        auto random_state_b = vc::make_multi_robot_state<Robot, rake>(pack_configuration(sample_b), base_b);

        std::cout << "Sample " << sample << ":\n";
        const auto result = vc::debug_fkcc_multi_all<rake>(environment, random_state_a, random_state_b);
        print_result(result, robot_labels);
    }

    return 0;
}
