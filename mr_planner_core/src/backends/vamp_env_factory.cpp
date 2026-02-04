#define MR_PLANNER_VAMP_INSTANCE_DECL_ONLY

#include <mr_planner/backends/vamp_env_factory.h>

#include <mr_planner/backends/vamp_presets.h>
#include <mr_planner/backends/vamp_plugin_loader.h>
#include <mr_planner/core/instance.h>
#include <mr_planner/core/logger.h>

#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>

#include <jsoncpp/json/json.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace vamp_env
{
namespace
{
    constexpr std::array<double, 4> kPandaGripperRodColor = {
        145.0 / 255.0,
        30.0 / 255.0,
        180.0 / 255.0,
        1.0,
    };

    Object make_box_object(const std::string &name,
                           double x,
                           double y,
                           double z,
                           double length,
                           double width,
                           double height)
    {
        Object obj;
        obj.name = name;
        obj.state = Object::State::Static;
        obj.parent_link = "world";
        obj.robot_id = -1;
        obj.x = x;
        obj.y = y;
        obj.z = z;
        obj.qx = 0.0;
        obj.qy = 0.0;
        obj.qz = 0.0;
        obj.qw = 1.0;
        obj.shape = Object::Shape::Box;
        obj.radius = 0.0;
        obj.length = length;
        obj.width = width;
        obj.height = height;
        obj.mesh_path.clear();
        return obj;
    }

    Eigen::Quaterniond quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
        return rz * ry * rx;
    }

    void add_panda_table_segments(PlanInstance &instance, const std::string &prefix)
    {
        struct Segment
        {
            double x;
            double y;
            double z;
            double length;
            double width;
            double height;
        };

        const std::array<Segment, 7> segments = {{{0.0, 0.0, 0.05, 0.6, 2.0, 0.1},
                                                  {0.75, 0.0, 0.05, 0.5, 2.0, 0.1},
                                                  {-0.75, 0.0, 0.05, 0.5, 2.0, 0.1},
                                                  {0.4, 0.55, 0.05, 0.2, 0.9, 0.1},
                                                  {0.4, -0.55, 0.05, 0.2, 0.9, 0.1},
                                                  {-0.4, 0.55, 0.05, 0.2, 0.9, 0.1},
                                                  {-0.4, -0.55, 0.05, 0.2, 0.9, 0.1}}};

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const auto &segment = segments[i];
            auto table = make_box_object(
                prefix + "_segment_" + std::to_string(i),
                segment.x,
                segment.y,
                segment.z,
                segment.length,
                segment.width,
                segment.height);
            instance.addMoveableObject(table);
        }
    }

    void add_panda_four_table_segments(PlanInstance &instance, const std::string &prefix)
    {
        const double z = 0.05;
        const double thickness = 0.1;
        const std::array<std::array<double, 4>, 6> segments = {{{0.0, 0.0, 2.0, 0.5},
                                                                {0.0, 0.0, 0.5, 2.0},
                                                                {0.77, 0.0, 0.46, 2.0},
                                                                {-0.77, 0.0, 0.46, 2.0},
                                                                {0.0, 0.77, 2.0, 0.46},
                                                                {0.0, -0.77, 2.0, 0.46}}};

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const auto &segment = segments[i];
            auto table = make_box_object(
                prefix + "_segment_" + std::to_string(i),
                segment[0],
                segment[1],
                z,
                segment[2],
                segment[3],
                thickness);
            instance.addMoveableObject(table);
        }
    }

    void add_hollow_box_obstacles(PlanInstance &instance,
                                  const std::string &prefix,
                                  const Eigen::Vector3d &center,
                                  const Eigen::Vector3d &size,
                                  double thickness = 0.02)
    {
        const double cx = center.x();
        const double cy = center.y();
        const double cz = center.z();
        const double length = size.x();
        const double width = size.y();
        const double height = size.z();

        auto bottom = make_box_object(
            prefix + "_bottom",
            cx,
            cy,
            cz,
            length,
            width,
            thickness);
        instance.addMoveableObject(bottom);

        const double side_z = cz + thickness * 0.5 + height * 0.5;

        auto front = make_box_object(
            prefix + "_front",
            cx,
            cy + width / 2.0,
            side_z,
            length,
            thickness,
            height);
        auto back = front;
        back.name = prefix + "_back";
        back.y = cy - width / 2.0;

        instance.addMoveableObject(front);
        instance.addMoveableObject(back);

        auto left = make_box_object(
            prefix + "_left",
            cx - length / 2.0,
            cy,
            side_z,
            thickness,
            width,
            height);
        auto right = left;
        right.name = prefix + "_right";
        right.x = cx + length / 2.0;

        instance.addMoveableObject(left);
        instance.addMoveableObject(right);
    }

    void add_panda_three_table_segments(PlanInstance &instance, const std::string &prefix)
    {
        struct Segment
        {
            double x;
            double y;
            double z;
            double length;
            double width;
            double height;
        };

        const std::array<Segment, 12> segments = {{{-0.9, 0.0, 0.05, 0.25, 2.0, 0.1},
                                                   {-0.65, 0.0, 0.05, 0.2, 2.0, 0.1},
                                                   {-0.2, 0.0, 0.05, 0.16, 2.0, 0.1},
                                                   {0.2, 0.0, 0.05, 0.16, 2.0, 0.1},
                                                   {0.65, 0.0, 0.05, 0.2, 2.0, 0.1},
                                                   {0.9, 0.0, 0.05, 0.25, 2.0, 0.1},
                                                   {-0.4, 0.55, 0.05, 0.2, 0.9, 0.1},
                                                   {-0.4, -0.55, 0.05, 0.2, 0.9, 0.1},
                                                   {0.0, 0.55, 0.05, 0.2, 0.9, 0.1},
                                                   {0.0, -0.55, 0.05, 0.2, 0.9, 0.1},
                                                   {0.4, 0.55, 0.05, 0.2, 0.9, 0.1},
                                                   {0.4, -0.55, 0.05, 0.2, 0.9, 0.1}}};

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const auto &segment = segments[i];
            auto table = make_box_object(
                prefix + "_segment_" + std::to_string(i),
                segment.x,
                segment.y,
                segment.z,
                segment.length,
                segment.width,
                segment.height);
            instance.addMoveableObject(table);
        }
    }

    void add_panda_three_bookshelf(PlanInstance &instance, const std::string &prefix)
    {
        auto back_plate = make_box_object(prefix + "_back_plate", 0.0, -1.0, 0.5, 0.8, 0.02, 1.0);
        instance.addMoveableObject(back_plate);

        const double side_y = -0.82;
        const double side_z = 0.5;
        const double side_length = 0.02;
        const double side_width = 0.36;
        const double side_height = 1.0;

        auto left_side = make_box_object(
            prefix + "_left_side_plate",
            -0.4,
            side_y,
            side_z,
            side_length,
            side_width,
            side_height);
        auto right_side = make_box_object(
            prefix + "_right_side_plate",
            0.4,
            side_y,
            side_z,
            side_length,
            side_width,
            side_height);
        instance.addMoveableObject(left_side);
        instance.addMoveableObject(right_side);

        const std::array<double, 5> shelf_offsets = {{-0.5, -0.25, 0.0, 0.25, 0.5}};
        for (std::size_t i = 0; i < shelf_offsets.size(); ++i)
        {
            const double z = 0.5 + shelf_offsets[i];
            auto shelf = make_box_object(
                prefix + "_shelf_" + std::to_string(i),
                0.0,
                side_y,
                z,
                0.8,
                0.36,
                0.01);
            instance.addMoveableObject(shelf);
        }
    }

    void add_panda_three_environment(PlanInstance &instance)
    {
        add_panda_three_table_segments(instance, "panda_three_table");

        auto wall = make_box_object("panda_three_wall", 0.0, 1.0, 1.0, 3.0, 0.01, 2.0);
        instance.addMoveableObject(wall);

        add_panda_three_bookshelf(instance, "panda_three_bookshelf");
    }

    void add_panda_six_table_segments(PlanInstance &instance, const std::string &prefix)
    {
        struct Segment
        {
            double x;
            double y;
            double z;
            double length;
            double width;
            double height;
        };

        const std::array<Segment, 12> segments = {{
            {0.0, 0.85, 0.05, 2.0, 0.3, 0.1},
            {0.0, -0.85, 0.05, 2.0, 0.3, 0.1},
            {-0.85, 0.0, 0.05, 0.3, 2.0, 0.1},
            {0.85, 0.0, 0.05, 0.3, 2.0, 0.1},
            {0.0, 0.0, 0.05, 1.2, 0.3, 0.1},
            {0.0, 0.0, 0.05, 0.3, 1.2, 0.1},
            {0.45, 0.26, 0.05, 0.5, 0.3, 0.1},
            {-0.45, 0.26, 0.05, 0.5, 0.3, 0.1},
            {-0.45, -0.26, 0.05, 0.5, 0.3, 0.1},
            {0.45, -0.26, 0.05, 0.5, 0.3, 0.1},
            {0.0, 0.7, 0.05, 1.2, 0.25, 0.1},
            {0.0, -0.7, 0.05, 1.2, 0.25, 0.1},
        }};

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const auto &segment = segments[i];
            auto table = make_box_object(
                prefix + "_segment_" + std::to_string(i),
                segment.x,
                segment.y,
                segment.z,
                segment.length,
                segment.width,
                segment.height);
            instance.addMoveableObject(table);
        }
    }

    void add_panda_six_environment(PlanInstance &instance)
    {
        add_panda_six_table_segments(instance, "panda_six_table");

        auto plank = make_box_object("panda_six_plank", 0.0, 0.0, 0.7, 0.2, 0.6, 0.02);
        instance.addMoveableObject(plank);
    }

    void add_cylinder_attachment(PlanInstance &instance,
                                 int robot_id,
                                 const std::string &name,
                                 double radius,
                                 double length,
                                 const Eigen::Vector3d &translation,
                                 const Eigen::Quaterniond &orientation)
    {
        Object obj;
        obj.name = name;
        obj.state = Object::State::Attached;
        obj.parent_link.clear();
        obj.robot_id = -1;
        obj.x = 0.0;
        obj.y = 0.0;
        obj.z = 0.0;
        obj.qx = 0.0;
        obj.qy = 0.0;
        obj.qz = 0.0;
        obj.qw = 1.0;
        obj.shape = Object::Shape::Cylinder;
        obj.radius = radius;
        obj.length = length;
        obj.width = 0.0;
        obj.height = 0.0;

        obj.x_attach = translation.x();
        obj.y_attach = translation.y();
        obj.z_attach = translation.z();

        Eigen::Quaterniond norm_q = orientation.normalized();
        obj.qx_attach = norm_q.x();
        obj.qy_attach = norm_q.y();
        obj.qz_attach = norm_q.z();
        obj.qw_attach = norm_q.w();

        instance.moveObject(obj);

        RobotPose placeholder = instance.initRobotPose(robot_id);
        instance.attachObjectToRobot(name, robot_id, "", placeholder);
    }

    std::optional<std::string> getenv_string(const char *key)
    {
        const char *val = std::getenv(key);
        if (!val || std::string(val).empty())
        {
            return std::nullopt;
        }
        return std::string(val);
    }

    std::vector<std::string> split_paths(const std::string &input)
    {
        std::vector<std::string> out;
        std::stringstream ss(input);
        std::string item;
        while (std::getline(ss, item, ':'))
        {
            if (!item.empty())
            {
                out.push_back(item);
            }
        }
        return out;
    }

    bool parse_vec3(const Json::Value &v, Eigen::Vector3d *out)
    {
        if (!out || !v.isArray() || v.size() != 3)
        {
            return false;
        }
        (*out)(0) = v[0].asDouble();
        (*out)(1) = v[1].asDouble();
        (*out)(2) = v[2].asDouble();
        return true;
    }

    bool parse_vec4(const Json::Value &v, Eigen::Quaterniond *out)
    {
        if (!out || !v.isArray() || v.size() != 4)
        {
            return false;
        }
        out->x() = v[0].asDouble();
        out->y() = v[1].asDouble();
        out->z() = v[2].asDouble();
        out->w() = v[3].asDouble();
        return true;
    }

    bool parse_transform(const Json::Value &v, Eigen::Isometry3d *out)
    {
        if (!out || !v.isObject())
        {
            return false;
        }
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

        if (v.isMember("xyz") && parse_vec3(v["xyz"], &xyz))
        {
            // ok
        }
        else if (v.isMember("translation") && parse_vec3(v["translation"], &xyz))
        {
            // ok
        }

        if (v.isMember("quaternion") && parse_vec4(v["quaternion"], &q))
        {
            q.normalize();
        }
        else if (v.isMember("rpy") && v["rpy"].isArray() && v["rpy"].size() == 3)
        {
            q = quaternion_from_rpy(v["rpy"][0].asDouble(), v["rpy"][1].asDouble(), v["rpy"][2].asDouble());
        }

        out->setIdentity();
        out->translation() = xyz;
        out->linear() = q.toRotationMatrix();
        return true;
    }

    std::optional<EnvironmentConfig> parse_environment_json(const std::string &path, const Json::Value &root)
    {
        if (!root.isObject())
        {
            return std::nullopt;
        }

        EnvironmentConfig cfg;
        cfg.environment_name = root.get("environment_name", root.get("name", "")).asString();
        cfg.move_group = root.get("move_group", "").asString();

        const auto robots = root["robot_groups"];
        if (!robots.isArray())
        {
            return std::nullopt;
        }
        for (Json::ArrayIndex i = 0; i < robots.size(); ++i)
        {
            if (robots[i].isString())
            {
                cfg.robot_groups.push_back(robots[i].asString());
            }
        }

        const auto hands = root["hand_groups"];
        if (hands.isArray())
        {
            for (Json::ArrayIndex i = 0; i < hands.size(); ++i)
            {
                if (hands[i].isString())
                {
                    cfg.hand_groups.push_back(hands[i].asString());
                }
            }
        }

        cfg.vamp_plugin = root.get("vamp_plugin", "").asString();
        cfg.vamp_variant = std::nullopt;
        if (!cfg.vamp_plugin.empty())
        {
            std::filesystem::path plugin_path(cfg.vamp_plugin);
            if (plugin_path.is_relative())
            {
                plugin_path = std::filesystem::path(path).parent_path() / plugin_path;
            }
            cfg.vamp_plugin = plugin_path.lexically_normal().string();
        }

        const auto base_tf = root["base_transforms"];
        if (base_tf.isArray() && !base_tf.empty())
        {
            std::vector<Eigen::Isometry3d> transforms;
            transforms.reserve(base_tf.size());
            for (Json::ArrayIndex i = 0; i < base_tf.size(); ++i)
            {
                Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
                if (!parse_transform(base_tf[i], &tf))
                {
                    log("Invalid base_transforms[" + std::to_string(i) + "] in " + path, LogLevel::WARN);
                    continue;
                }
                transforms.push_back(tf);
            }
            if (!transforms.empty())
            {
                cfg.base_transforms = std::move(transforms);
            }
        }

        if (cfg.environment_name.empty())
        {
            cfg.environment_name = std::filesystem::path(path).stem().string();
        }
        if (cfg.move_group.empty() && !cfg.robot_groups.empty())
        {
            cfg.move_group = cfg.robot_groups.front();
        }
        if (cfg.robot_groups.empty() || cfg.vamp_plugin.empty())
        {
            return std::nullopt;
        }
        return cfg;
    }

    std::optional<Json::Value> read_json_file(const std::string &path, std::string *err)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            if (err)
            {
                *err = "Failed to open file: " + path;
            }
            return std::nullopt;
        }
        Json::CharReaderBuilder builder;
        Json::Value root;
        std::string errs;
        if (!Json::parseFromStream(builder, file, &root, &errs))
        {
            if (err)
            {
                *err = errs;
            }
            return std::nullopt;
        }
        return root;
    }

    std::optional<EnvironmentConfig> load_environment_from_path(const std::string &path, std::string *err)
    {
        auto root = read_json_file(path, err);
        if (!root)
        {
            return std::nullopt;
        }
        auto cfg = parse_environment_json(path, *root);
        if (!cfg && err)
        {
            *err = "Invalid environment JSON schema: " + path;
        }
        return cfg;
    }

    std::optional<std::string> resolve_environment_json(const std::string &name)
    {
        if (name.empty())
        {
            return std::nullopt;
        }
        if (std::filesystem::exists(name))
        {
            return name;
        }

        auto search = getenv_string("MR_PLANNER_VAMP_ENV_PATH");
        if (!search)
        {
            return std::nullopt;
        }

        for (const auto &dir : split_paths(*search))
        {
            const auto p1 = std::filesystem::path(dir) / (name + ".json");
            if (std::filesystem::exists(p1))
            {
                return p1.string();
            }
            const auto p2 = std::filesystem::path(dir) / name / "environment.json";
            if (std::filesystem::exists(p2))
            {
                return p2.string();
            }
        }
        return std::nullopt;
    }
}  // namespace

std::optional<EnvironmentConfig> make_environment_config(const std::string &name)
{
    if (auto path = resolve_environment_json(name))
    {
        std::string err;
        auto cfg = load_environment_from_path(*path, &err);
        if (!cfg)
        {
            log("Failed to load VAMP environment config '" + *path + "': " + err, LogLevel::ERROR);
            return std::nullopt;
        }
        return cfg;
    }

    if (name == "dual_gp4")
    {
        return EnvironmentConfig{
            name,
            "dual_arms",
            {"left_arm", "right_arm"},
            {},
            VampVariant::DualGp4,
            {},
            std::nullopt};
    }
    if (name == "quad_gp4")
    {
        return EnvironmentConfig{
            name,
            "quad_arms",
            {"left_arm", "right_arm", "top_arm", "bottom_arm"},
            {},
            VampVariant::QuadGp4,
            {},
            std::nullopt};
    }
    if (name == "panda_two")
    {
        return EnvironmentConfig{
            name,
            "panda_multi_arm",
            {"panda0_arm", "panda1_arm"},
            {"panda0_hand", "panda1_hand"},
            VampVariant::PandaPair,
            {},
            std::nullopt};
    }
    if (name == "panda_two_rod")
    {
        return EnvironmentConfig{
            name,
            "panda_multi_arm",
            {"panda0_arm", "panda1_arm"},
            {"panda0_hand", "panda1_hand"},
            VampVariant::PandaPair,
            {},
            std::nullopt};
    }
    if (name == "panda_three")
    {
        return EnvironmentConfig{
            name,
            "panda_multi_arm",
            {"panda0_arm", "panda1_arm", "panda2_arm"},
            {"panda0_hand", "panda1_hand", "panda2_hand"},
            VampVariant::PandaTriple,
            {},
            std::nullopt};
    }
    if (name == "panda_six")
    {
        return EnvironmentConfig{
            name,
            "panda_multi_arm",
            {"panda0_arm",
             "panda1_arm",
             "panda2_arm",
             "panda3_arm",
             "panda4_arm",
             "panda5_arm"},
            {"panda0_hand",
             "panda1_hand",
             "panda2_hand",
             "panda3_hand",
             "panda4_hand",
             "panda5_hand"},
            VampVariant::PandaHex,
            {},
            std::nullopt};
    }
    if (name == "panda_four")
    {
        return EnvironmentConfig{
            name,
            "panda_multi_arm",
            {"panda0_arm", "panda1_arm", "panda2_arm", "panda3_arm"},
            {"panda0_hand", "panda1_hand", "panda2_hand", "panda3_hand"},
            VampVariant::PandaQuad,
            {},
            std::nullopt};
    }
    if (name == "panda_four_bins")
    {
        return EnvironmentConfig{
            name,
            "panda_multi_arm",
            {"panda0_arm", "panda1_arm", "panda2_arm", "panda3_arm"},
            {"panda0_hand", "panda1_hand", "panda2_hand", "panda3_hand"},
            VampVariant::PandaQuad,
            {},
            std::nullopt};
    }
    return std::nullopt;
}

std::shared_ptr<PlanInstance> make_vamp_instance(VampVariant variant)
{
    switch (variant)
    {
    case VampVariant::DualGp4:
        return std::make_shared<VampDualGp4Instance>();
    case VampVariant::QuadGp4:
        return std::make_shared<VampQuadGp4Instance>();
    case VampVariant::PandaPair:
        return std::make_shared<VampPandaPairInstance>();
    case VampVariant::PandaTriple:
        return std::make_shared<VampPandaTripleInstance>();
    case VampVariant::PandaQuad:
        return std::make_shared<VampPandaQuadInstance>();
    case VampVariant::PandaHex:
        return std::make_shared<VampPandaHexInstance>();
    case VampVariant::PandaOct:
        return std::make_shared<VampPandaOctInstance>();
    case VampVariant::PandaDec:
        return std::make_shared<VampPandaDecInstance>();
    }
    throw std::runtime_error("Unsupported VampVariant");
}

std::shared_ptr<PlanInstance> make_vamp_instance(const EnvironmentConfig &config)
{
    if (config.vamp_variant)
    {
        return make_vamp_instance(*config.vamp_variant);
    }
    if (!config.vamp_plugin.empty())
    {
        return mr_planner::vamp_plugin::load_instance_from_library(config.vamp_plugin);
    }
    throw std::runtime_error("EnvironmentConfig has neither vamp_variant nor vamp_plugin");
}

void add_environment_obstacles(const EnvironmentConfig &config, PlanInstance &instance)
{
    if (config.environment_name == "panda_two" || config.environment_name == "panda_two_rod")
    {
        add_panda_table_segments(instance, "panda_two_table");
    }
    else if (config.environment_name == "panda_four")
    {
        add_panda_four_table_segments(instance, "panda_four_table");
    }
    else if (config.environment_name == "panda_four_bins")
    {
        add_panda_four_table_segments(instance, "panda_four_table");
        const std::array<Eigen::Vector3d, 5> centers = {
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.5, 0.0, 0.0),
            Eigen::Vector3d(-0.5, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.5, 0.0),
            Eigen::Vector3d(0.0, -0.5, 0.0)};
        const Eigen::Vector3d size(0.5, 0.5, 0.5);
        for (std::size_t i = 0; i < centers.size(); ++i)
        {
            add_hollow_box_obstacles(
                instance,
                "panda_four_bins_box_" + std::to_string(i),
                centers[i],
                size);
        }
    }
    else if (config.environment_name == "panda_three")
    {
        add_panda_three_environment(instance);
    }
    else if (config.environment_name == "panda_six")
    {
        add_panda_six_environment(instance);
    }
}

void add_environment_attachments(const EnvironmentConfig &config, PlanInstance &instance)
{
    if (config.environment_name == "panda_two_rod")
    {
        const Eigen::Quaterniond rod_orientation = quaternion_from_rpy(0.0, M_PI_2, 0.0);
        add_cylinder_attachment(
            instance,
            0,
            "panda0_rod",
            0.015,
            0.5,
            Eigen::Vector3d(0.0, 0.0, -0.005),
            rod_orientation);
        instance.setObjectColor(
            "panda0_rod",
            kPandaGripperRodColor[0],
            kPandaGripperRodColor[1],
            kPandaGripperRodColor[2],
            kPandaGripperRodColor[3]);
        add_cylinder_attachment(
            instance,
            1,
            "panda1_rod",
            0.015,
            0.5,
            Eigen::Vector3d(0.0, 0.0, -0.005),
            rod_orientation);
        instance.setObjectColor(
            "panda1_rod",
            kPandaGripperRodColor[0],
            kPandaGripperRodColor[1],
            kPandaGripperRodColor[2],
            kPandaGripperRodColor[3]);
    }
    else if (config.environment_name == "panda_four_bins")
    {
        const Eigen::Quaterniond rod_orientation = quaternion_from_rpy(0.0, M_PI_2, 0.0);
        for (int robot_id = 0; robot_id < 4; ++robot_id)
        {
            const std::string rod_name = "panda" + std::to_string(robot_id) + "_gripper_rod";
            add_cylinder_attachment(
                instance,
                robot_id,
                rod_name,
                0.01,
                0.3,
                Eigen::Vector3d(0.0, 0.0, -0.005),
                rod_orientation);
            instance.setObjectColor(
                rod_name,
                kPandaGripperRodColor[0],
                kPandaGripperRodColor[1],
                kPandaGripperRodColor[2],
                kPandaGripperRodColor[3]);
        }
    }
}

std::optional<std::vector<Eigen::Isometry3d>> default_base_transforms(const EnvironmentConfig &config)
{
    if (config.base_transforms)
    {
        return config.base_transforms;
    }
    if (!config.vamp_variant)
    {
        return std::nullopt;
    }
    std::vector<Eigen::Isometry3d> transforms(config.robot_groups.size(), Eigen::Isometry3d::Identity());
    switch (*config.vamp_variant)
    {
    case VampVariant::DualGp4:
    {
        if (transforms.size() < 2)
        {
            return std::nullopt;
        }
        // Match the dual_gp4 URDF: right arm faces the left arm with ~180deg yaw.
        const Eigen::AngleAxisd right_rotation(3.13792336, Eigen::Vector3d::UnitZ());
        transforms[0].translation() = Eigen::Vector3d(0.0, 0.0, 0.9246);
        transforms[1].linear() = right_rotation.toRotationMatrix();
        transforms[1].translation() = Eigen::Vector3d(0.88128092, -0.01226491, 0.9246);
        return transforms;
    }
    case VampVariant::QuadGp4:
    {
        if (transforms.size() < 4)
        {
            return std::nullopt;
        }
        // Layout mirrors the calibrated base files r1_base.txt - r4_base.txt
        // and the quad_gp4 URDF xacro: two robots facing each other along X,
        // two robots facing each other along Y. All share the same height.
        const std::array<Eigen::Vector3d, 4> translations = {
            Eigen::Vector3d(0.0, 0.0, 0.9246),
            Eigen::Vector3d(0.881, -0.012, 0.9246),
            Eigen::Vector3d(0.44, -0.44, 0.9246),
            Eigen::Vector3d(0.44, 0.44, 0.9246)};
        const std::array<double, 4> yaws = {
            0.0,
            3.13792336,          // ~180 deg, calibrated
            M_PI_2,
            -M_PI_2};

        for (std::size_t i = 0; i < 4; ++i)
        {
            const Eigen::AngleAxisd rotation(yaws[i], Eigen::Vector3d::UnitZ());
            transforms[i].linear() = rotation.toRotationMatrix();
            transforms[i].translation() = translations[i];
        }
        return transforms;
    }
    case VampVariant::PandaPair:
    {
        if (transforms.size() < 2)
        {
            return std::nullopt;
        }
        Eigen::AngleAxisd left_rotation(M_PI, Eigen::Vector3d::UnitZ());
        transforms[0].linear() = left_rotation.toRotationMatrix();
        transforms[0].translation() = Eigen::Vector3d(0.4, 0.0, 0.1);
        transforms[1].translation() = Eigen::Vector3d(-0.4, 0.0, 0.1);
        return transforms;
    }
    case VampVariant::PandaTriple:
    {
        if (transforms.size() < 3)
        {
            return std::nullopt;
        }
        const Eigen::AngleAxisd yaw(M_PI_2, Eigen::Vector3d::UnitZ());
        const Eigen::Matrix3d rotation = yaw.toRotationMatrix();
        transforms[0].linear() = rotation;
        transforms[1].linear() = rotation;
        transforms[2].linear() = rotation;
        transforms[0].translation() = Eigen::Vector3d(-0.4, 0.0, 0.1);
        transforms[1].translation() = Eigen::Vector3d(0.0, 0.0, 0.1);
        transforms[2].translation() = Eigen::Vector3d(0.4, 0.0, 0.1);
        return transforms;
    }
    case VampVariant::PandaQuad:
    {
        if (transforms.size() < 4)
        {
            return std::nullopt;
        }
        if (config.environment_name == "panda_four")
        {
            // Match the panda_four URDF: four arms on a square table, yawed about Z.
            const std::array<double, 4> yaws = {2.28, 0.66, -0.66, -2.28};
            const std::array<Eigen::Vector3d, 4> translations = {
                Eigen::Vector3d(0.36, -0.36, 0.1),
                Eigen::Vector3d(-0.36, -0.36, 0.1),
                Eigen::Vector3d(-0.36, 0.36, 0.1),
                Eigen::Vector3d(0.36, 0.36, 0.1)};

            for (std::size_t i = 0; i < yaws.size(); ++i)
            {
                const Eigen::AngleAxisd rotation(yaws[i], Eigen::Vector3d::UnitZ());
                transforms[i].linear() = rotation.toRotationMatrix();
                transforms[i].translation() = translations[i];
            }
            return transforms;
        }
        else if (config.environment_name == "panda_four_bins")
        {
            const std::array<double, 4> yaws = {2.28, 0.66, -0.66, -2.28};
            const std::array<Eigen::Vector3d, 4> translations = {
                Eigen::Vector3d(0.4, -0.4, 0.1),
                Eigen::Vector3d(-0.4, -0.4, 0.1),
                Eigen::Vector3d(-0.4, 0.4, 0.1),
                Eigen::Vector3d(0.4, 0.4, 0.1)};

            for (std::size_t i = 0; i < yaws.size(); ++i)
            {
                const Eigen::AngleAxisd rotation(yaws[i], Eigen::Vector3d::UnitZ());
                transforms[i].linear() = rotation.toRotationMatrix();
                transforms[i].translation() = translations[i];
            }
            return transforms;
        }
        return std::nullopt;
    }
    case VampVariant::PandaHex:
    {
        constexpr std::size_t count = 6;
        if (transforms.size() < count)
        {
            return std::nullopt;
        }
        const std::array<Eigen::Vector3d, count> translations = {
            Eigen::Vector3d(0.6, 0.0, 0.1),
            Eigen::Vector3d(0.3, 0.52, 0.1),
            Eigen::Vector3d(-0.3, 0.52, 0.1),
            Eigen::Vector3d(-0.6, 0.0, 0.1),
            Eigen::Vector3d(-0.3, -0.52, 0.1),
            Eigen::Vector3d(0.3, -0.52, 0.1)};

        const std::array<double, count> yaws = {
            M_PI,
            -2.0 * M_PI / 3.0,
            -M_PI / 3.0,
            0.0,
            M_PI / 3.0,
            2.0 * M_PI / 3.0};

        for (std::size_t idx = 0; idx < count; ++idx)
        {
            const Eigen::AngleAxisd rotation(yaws[idx], Eigen::Vector3d::UnitZ());
            transforms[idx].linear() = rotation.toRotationMatrix();
            transforms[idx].translation() = translations[idx];
        }

        return transforms;
    }
    case VampVariant::PandaOct:
    {
        constexpr std::size_t count = 8;
        if (transforms.size() < count)
        {
            return std::nullopt;
        }
        const double radius = 0.65;
        const double height = 0.1;
        const double offset = 0.0;
        for (std::size_t idx = 0; idx < count; ++idx)
        {
            const double angle = offset + (2.0 * M_PI * static_cast<double>(idx)) / static_cast<double>(count);
            const Eigen::AngleAxisd rotation(angle + M_PI, Eigen::Vector3d::UnitZ());
            transforms[idx].linear() = rotation.toRotationMatrix();
            transforms[idx].translation() = Eigen::Vector3d(
                radius * std::cos(angle),
                radius * std::sin(angle),
                height);
        }
        return transforms;
    }
    case VampVariant::PandaDec:
    {
        constexpr std::size_t count = 10;
        if (transforms.size() < count)
        {
            return std::nullopt;
        }
        const double radius = 0.7;
        const double height = 0.1;
        const double offset = 0.0;
        for (std::size_t idx = 0; idx < count; ++idx)
        {
            const double angle = offset + (2.0 * M_PI * static_cast<double>(idx)) / static_cast<double>(count);
            const Eigen::AngleAxisd rotation(angle + M_PI, Eigen::Vector3d::UnitZ());
            transforms[idx].linear() = rotation.toRotationMatrix();
            transforms[idx].translation() = Eigen::Vector3d(
                radius * std::cos(angle),
                radius * std::sin(angle),
                height);
        }
        return transforms;
    }
    }
    return std::nullopt;
}

}  // namespace vamp_env
