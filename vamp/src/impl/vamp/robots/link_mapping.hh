#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>
#include <vamp/robots/gp4.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/sphere.hh>
#include <vamp/robots/ur5.hh>

namespace vamp::robots
{
    template <typename Robot>
    struct LinkMapping
    {
        static constexpr bool available = false;
        inline static constexpr std::array<std::string_view, 0> link_names{};
        inline static constexpr std::array<std::uint16_t, 0> sphere_to_link{};
    };

    template <>
    struct LinkMapping<GP4>
    {
        static constexpr bool available = true;
        inline static constexpr std::array<std::string_view, 9> link_names = {
            "base_link",
            "link_1",
            "link_2",
            "link_3",
            "link_4",
            "link_5",
            "link_6",
            "fts",
            "link_tool"};
        inline static constexpr std::array<std::uint16_t, 65> sphere_to_link = {
            0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
            2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 
            8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};

        static_assert(GP4::n_spheres == sphere_to_link.size(), "GP4 sphere mapping mismatch");
    };

    template <>
    struct LinkMapping<Panda>
    {
        static constexpr bool available = true;
        inline static constexpr std::array<std::string_view, 11> link_names = {
            "panda_link0",
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
            "panda_link7",
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger"};
        inline static constexpr std::array<std::uint16_t, 59> sphere_to_link = {
            0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
            4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6,
            7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
            8, 8, 8, 8, 8, 8, 8, 9, 9, 10, 10};

        static_assert(Panda::n_spheres == sphere_to_link.size(), "Panda sphere mapping mismatch");
    };

    template <>
    struct LinkMapping<Fetch>
    {
        static constexpr bool available = true;
        inline static constexpr std::array<std::string_view, 15> link_names = {
            "base_link",
            "torso_lift_link",
            "torso_lift_link_collision_2",
            "head_pan_link",
            "shoulder_pan_link",
            "shoulder_lift_link",
            "upperarm_roll_link",
            "elbow_flex_link",
            "forearm_roll_link",
            "wrist_flex_link",
            "wrist_roll_link",
            "gripper_link",
            "r_gripper_finger_link",
            "l_gripper_finger_link",
            "torso_fixed_link"};
        inline static constexpr std::array<std::uint16_t, 111> sphere_to_link = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
            1, 1, 1, 1, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
            3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
            3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6,
            6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8,
            8, 9, 9, 9, 9, 9, 9, 10, 10, 11, 11, 11, 11, 12, 12, 12,
            12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14};

        static_assert(Fetch::n_spheres == sphere_to_link.size(), "Fetch sphere mapping mismatch");
    };

    template <>
    struct LinkMapping<Baxter>
    {
        static constexpr bool available = true;
        inline static constexpr std::array<std::string_view, 33> link_names = {
            "torso",
            "head",
            "right_upper_shoulder",
            "right_lower_shoulder",
            "right_upper_elbow",
            "right_lower_elbow",
            "right_upper_forearm",
            "right_lower_forearm",
            "right_wrist",
            "right_hand",
            "left_upper_shoulder",
            "left_lower_shoulder",
            "left_upper_elbow",
            "left_lower_elbow",
            "left_upper_forearm",
            "left_lower_forearm",
            "left_wrist",
            "left_hand",
            "pedestal",
            "left_gripper_base",
            "l_gripper_l_finger",
            "l_gripper_l_finger_2",
            "l_gripper_l_finger_tip",
            "l_gripper_r_finger",
            "l_gripper_r_finger_2",
            "l_gripper_r_finger_tip",
            "right_gripper_base",
            "r_gripper_l_finger",
            "r_gripper_l_finger_2",
            "r_gripper_l_finger_tip",
            "r_gripper_r_finger",
            "r_gripper_r_finger_2",
            "r_gripper_r_finger_tip"};
        inline static constexpr std::array<std::uint16_t, 75> sphere_to_link = {
            0, 0, 0, 1, 2, 2, 3, 4, 4, 4, 5, 6, 6, 6, 7, 7,
            8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 14, 14, 14, 15, 15, 16,
            16, 17, 18, 19, 19, 20, 20, 21, 21, 21, 22, 22, 22, 22, 23, 23,
            24, 24, 24, 25, 25, 25, 25, 26, 26, 27, 27, 28, 28, 28, 29, 29,
            29, 29, 30, 30, 31, 31, 31, 32, 32, 32, 32};

        static_assert(Baxter::n_spheres == sphere_to_link.size(), "Baxter sphere mapping mismatch");
    };

    template <>
    struct LinkMapping<UR5>
    {
        static constexpr bool available = true;
        inline static constexpr std::array<std::string_view, 17> link_names = {
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "fts_robotside",
            "robotiq_85_base_link",
            "robotiq_85_left_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_left_inner_knuckle_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_right_finger_tip_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_right_finger_link"};
        inline static constexpr std::array<std::uint16_t, 40> sphere_to_link = {
            0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3,
            4, 4, 4, 5, 5, 5, 6, 7, 8, 8, 9, 10, 10, 10, 11, 12,
            12, 13, 14, 14, 15, 16, 16, 16};

        static_assert(UR5::n_spheres == sphere_to_link.size(), "UR5 sphere mapping mismatch");
    };

    template <>
    struct LinkMapping<Sphere>
    {
        static constexpr bool available = true;
        inline static constexpr std::array<std::string_view, 1> link_names = {"sphere"};
        inline static constexpr std::array<std::uint16_t, 1> sphere_to_link = {0};

        static_assert(Sphere::n_spheres == sphere_to_link.size(), "Sphere mapping mismatch");
    };

    template <typename Robot>
    inline auto link_index(std::string_view link_name) noexcept -> std::optional<std::size_t>
    {
        if constexpr (LinkMapping<Robot>::available)
        {
            const auto &names = LinkMapping<Robot>::link_names;
            for (std::size_t i = 0; i < names.size(); ++i)
            {
                if (names[i] == link_name)
                {
                    return i;
                }
            }
        }

        return std::nullopt;
    }

    template <typename Robot>
    inline auto collect_link_spheres(std::size_t link_index, std::vector<std::size_t> &out) -> bool
    {
        if constexpr (LinkMapping<Robot>::available)
        {
            const auto &mapping = LinkMapping<Robot>::sphere_to_link;
            out.clear();
            for (std::size_t i = 0; i < mapping.size(); ++i)
            {
                if (mapping[i] == link_index)
                {
                    out.push_back(i);
                }
            }
            return true;
        }

        return false;
    }

    template <typename Robot>
    inline auto collect_link_spheres(std::string_view link_name, std::vector<std::size_t> &out) -> bool
    {
        if (const auto index = link_index<Robot>(link_name))
        {
            return collect_link_spheres<Robot>(*index, out);
        }

        return false;
    }
 }
