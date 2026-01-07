#include <array>
#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Geometry>

#include <vamp/collision/multi_robot.hh>
#include <vamp/robots/gp4.hh>

int main()
{
    using Robot = vamp::robots::GP4;
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    constexpr std::size_t iterations = 1000;

    std::mt19937 rng(42U);
    std::uniform_real_distribution<float> perturbation(-0.05F, 0.05F);

    auto environment = vamp::collision::Environment<vamp::FloatVector<rake>>{};

    const std::array<float, Robot::dimension> nominal = {0.F, 0.F, 0.F, -2.356F, 0.F, 0.F};

    Eigen::Isometry3f base_a = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f base_b = Eigen::Isometry3f::Identity();
    base_b.translate(Eigen::Vector3f(0.3F, 0.0F, 0.0F));

    using clock = std::chrono::high_resolution_clock;

    using PackedRow = typename Robot::template ConfigurationBlock<rake>::RowT;

    // Single-lane (fills every SIMD lane with the same sample) benchmark
    auto scalar_begin = clock::now();
    bool single_lane_collision_free = true;
    for (std::size_t iter = 0; iter < iterations; ++iter)
    {
        auto config_a_single = Robot::template ConfigurationBlock<rake>::zero_vector();
        auto config_b_single = Robot::template ConfigurationBlock<rake>::zero_vector();

        for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
        {
            const float sample_a = nominal[joint] + perturbation(rng);
            const float sample_b = nominal[joint] + perturbation(rng);
            const float value_a = (joint == 0) ? sample_a - 1.57F : sample_a;
            const float value_b = (joint == 0) ? sample_b + 1.57F : sample_b;

            config_a_single[joint] = PackedRow::fill(value_a);
            config_b_single[joint] = PackedRow::fill(value_b);
        }

        const auto state_a_single =
            vamp::collision::make_multi_robot_state<Robot, rake>(config_a_single, base_a);
        const auto state_b_single =
            vamp::collision::make_multi_robot_state<Robot, rake>(config_b_single, base_b);

        single_lane_collision_free &=
            vamp::collision::fkcc_multi_all<rake>(environment, state_a_single, state_b_single);
    }
    auto scalar_end = clock::now();

    // SIMD (rake = FloatVectorWidth) benchmark with packed configurations
    auto vector_begin = clock::now();
    bool vector_collision_free = true;
    for (std::size_t iter = 0; iter < iterations; ++iter)
    {
        auto config_a_vector = Robot::template ConfigurationBlock<rake>::zero_vector();
        auto config_b_vector = Robot::template ConfigurationBlock<rake>::zero_vector();

        for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
        {
            std::array<float, rake> lane_values_a{};
            std::array<float, rake> lane_values_b{};

            for (std::size_t lane = 0; lane < rake; ++lane)
            {
                lane_values_a[lane] = nominal[joint] + perturbation(rng);
                lane_values_b[lane] = nominal[joint] + perturbation(rng);
            }

            if (joint == 0)
            {
                for (std::size_t lane = 0; lane < rake; ++lane)
                {
                    lane_values_a[lane] -= 1.57F;
                    lane_values_b[lane] += 1.57F;
                }
            }

            config_a_vector[joint] = PackedRow(lane_values_a);
            config_b_vector[joint] = PackedRow(lane_values_b);
        }

        const auto state_a_vector = vamp::collision::make_multi_robot_state<Robot, rake>(config_a_vector, base_a);
        const auto state_b_vector = vamp::collision::make_multi_robot_state<Robot, rake>(config_b_vector, base_b);

        vector_collision_free &= vamp::collision::fkcc_multi_all<rake>(environment, state_a_vector, state_b_vector);
    }
    auto vector_end = clock::now();

    const double single_lane_ms = std::chrono::duration<double, std::milli>(scalar_end - scalar_begin).count();
    const double vector_ms = std::chrono::duration<double, std::milli>(vector_end - vector_begin).count();

    std::cout << (single_lane_collision_free ? "Single-lane pack: Collision free"
                                              : "Single-lane pack: In collision")
              << '\n';
    std::cout << "Single-lane total time: " << single_lane_ms << " ms (" << single_lane_ms / iterations
              << " ms per check)" << '\n';

    std::cout << (vector_collision_free ? "Vector: Collision free" : "Vector: In collision") << '\n';
    std::cout << "Vector total time: " << vector_ms << " ms (" << vector_ms / iterations
              << " ms per packed check, " << vector_ms / (iterations * rake)
              << " ms per configuration pair)" << std::endl;

    return (single_lane_collision_free && vector_collision_free) ? 0 : 1;
}
