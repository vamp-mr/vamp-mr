#include <mr_planner/io/skillplan.h>

#include <mr_planner/core/task.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>

namespace mr_planner::skillplan
{
namespace
{
Json::Value vec_to_json(const std::vector<double> &v)
{
    Json::Value out(Json::arrayValue);
    for (double x : v)
    {
        out.append(x);
    }
    return out;
}

bool as_int(const Json::Value &v, int *out)
{
    if (!out)
    {
        return false;
    }
    if (!v.isInt())
    {
        return false;
    }
    *out = v.asInt();
    return true;
}

bool as_string(const Json::Value &v, std::string *out)
{
    if (!out)
    {
        return false;
    }
    if (!v.isString())
    {
        return false;
    }
    *out = v.asString();
    return true;
}

bool parse_joint_positions(const Json::Value &v, int dof, std::vector<double> *out)
{
    if (!out)
    {
        return false;
    }
    if (!v.isArray())
    {
        return false;
    }
    if (dof > 0 && static_cast<int>(v.size()) != dof)
    {
        return false;
    }
    out->clear();
    out->reserve(v.size());
    for (const auto &val : v)
    {
        if (!val.isNumeric())
        {
            return false;
        }
        out->push_back(val.asDouble());
    }
    return true;
}

std::string format_error(const std::string &ctx, const std::string &msg)
{
    if (ctx.empty())
    {
        return msg;
    }
    return ctx + ": " + msg;
}

Json::Value pose_xyzquat_to_json(double x, double y, double z, double qx, double qy, double qz, double qw)
{
    Json::Value pose(Json::objectValue);

    Json::Value pos(Json::objectValue);
    pos["x"] = x;
    pos["y"] = y;
    pos["z"] = z;
    pose["position"] = pos;

    Json::Value ori(Json::objectValue);
    ori["w"] = qw;
    ori["x"] = qx;
    ori["y"] = qy;
    ori["z"] = qz;
    pose["orientation"] = ori;

    return pose;
}

std::string object_shape_to_string(Object::Shape shape)
{
    switch (shape)
    {
        case Object::Shape::Box:
            return "Box";
        case Object::Shape::Sphere:
            return "Sphere";
        case Object::Shape::Cylinder:
            return "Cylinder";
        case Object::Shape::Mesh:
            return "Mesh";
        default:
            return "Box";
    }
}

std::string object_state_to_string(Object::State state)
{
    switch (state)
    {
        case Object::State::Static:
            return "Static";
        case Object::State::Attached:
            return "Attached";
        case Object::State::Supported:
            return "Supported";
        case Object::State::Handover:
            return "Handover";
        default:
            return "Static";
    }
}

Json::Value object_to_json(const Object &obj,
                           bool vanish = false,
                           bool fixed = false,
                           bool handover = false)
{
    Json::Value out(Json::objectValue);
    out["name"] = obj.name;
    out["type"] = "Object";
    out["shape"] = object_shape_to_string(obj.shape);
    out["state"] = object_state_to_string(obj.state);
    out["parent_link"] = obj.parent_link;
    out["robot_id"] = obj.robot_id;

    out["pose"] = pose_xyzquat_to_json(obj.x, obj.y, obj.z, obj.qx, obj.qy, obj.qz, obj.qw);
    out["attach_pose"] =
        pose_xyzquat_to_json(obj.x_attach, obj.y_attach, obj.z_attach, obj.qx_attach, obj.qy_attach, obj.qz_attach, obj.qw_attach);

    Json::Value dims(Json::objectValue);
    dims["x"] = obj.length;
    dims["y"] = obj.width;
    dims["z"] = obj.height;
    if (obj.radius > 0.0)
    {
        dims["radius"] = obj.radius;
    }
    if (!obj.mesh_path.empty())
    {
        out["mesh_path"] = obj.mesh_path;
    }
    out["dimensions"] = dims;

    Json::Value meta(Json::objectValue);
    meta["vanish"] = vanish;
    meta["fixed"] = fixed;
    meta["handover"] = handover;
    out["mr_planner"] = meta;

    return out;
}

bool poses_close_maxabs(const RobotPose &a, const RobotPose &b, double eps)
{
    const auto &qa = a.joint_values;
    const auto &qb = b.joint_values;
    if (qa.size() != qb.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < qa.size(); ++i)
    {
        if (std::abs(qa[i] - qb[i]) > eps)
        {
            return false;
        }
    }
    const auto &ha = a.hand_values;
    const auto &hb = b.hand_values;
    if (ha.size() != hb.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < ha.size(); ++i)
    {
        if (std::abs(ha[i] - hb[i]) > eps)
        {
            return false;
        }
    }
    return true;
}

std::string format_action_id(int idx)
{
    std::ostringstream oss;
    oss << "a" << std::setw(4) << std::setfill('0') << idx;
    return oss.str();
}

std::string format_action_key(int robot_id, int act_id, const std::string &type)
{
    std::ostringstream oss;
    oss << "r" << robot_id << "_a" << act_id << "_" << type;
    return oss.str();
}

}  // namespace

Json::Value make_simple_plan(const std::vector<RobotSpec> &robots,
                             const MRTrajectory &trajectories,
                             const ExportOptions &options)
{
    Json::Value root(Json::objectValue);
    root["schema"] = "aidf.skillplan";
    root["schema_version"] = 1;
    if (!options.plan_name.empty())
    {
        root["plan_name"] = options.plan_name;
    }

    // robots[]
    Json::Value robots_json(Json::arrayValue);
    for (const auto &robot : robots)
    {
        Json::Value r(Json::objectValue);
        r["id"] = robot.id;
        r["name"] = robot.name;
        r["dof"] = robot.dof;
        if (!robot.end_effector_link.empty())
        {
            r["end_effector_link"] = robot.end_effector_link;
        }
        robots_json.append(r);
    }
    root["robots"] = robots_json;

    // initial_scene
    Json::Value init_scene(Json::objectValue);
    Json::Value init_robots(Json::arrayValue);
    for (const auto &robot : robots)
    {
        Json::Value rs(Json::objectValue);
        rs["robot_id"] = robot.id;
        rs["robot_name"] = robot.name;

        std::vector<double> start(robot.dof, 0.0);
        if (robot.id >= 0 && robot.id < static_cast<int>(trajectories.size()))
        {
            const auto &traj = trajectories[robot.id];
            if (!traj.trajectory.empty())
            {
                start = traj.trajectory.front().joint_values;
            }
        }
        rs["joint_positions"] = vec_to_json(start);
        init_robots.append(rs);
    }
    init_scene["robots"] = init_robots;
    init_scene["objects"] = Json::Value(Json::arrayValue);
    root["initial_scene"] = init_scene;

    // environment (optional)
    if (!options.environment_name.empty() || !options.backend_type.empty() || !options.moveit_group_name.empty())
    {
        Json::Value env(Json::objectValue);
        if (!options.environment_name.empty())
        {
            env["name"] = options.environment_name;
        }
        if (!options.backend_type.empty() || !options.moveit_group_name.empty() || !options.moveit_config_pkg.empty())
        {
            Json::Value backend(Json::objectValue);
            if (!options.backend_type.empty())
            {
                backend["type"] = options.backend_type;
            }
            if (!options.moveit_group_name.empty())
            {
                backend["moveit_group_name"] = options.moveit_group_name;
            }
            if (!options.moveit_config_pkg.empty())
            {
                backend["moveitConfigPkg"] = options.moveit_config_pkg;
            }
            backend["l1_vmax"] = options.l1_vmax;
            env["backend"] = backend;
        }
        root["environment"] = env;
    }

    // actions[] (one per robot)
    Json::Value actions(Json::arrayValue);
    int action_index = 0;
    for (const auto &robot : robots)
    {
        Json::Value action(Json::objectValue);
        std::ostringstream aid;
        aid << "a" << std::setw(4) << std::setfill('0') << action_index++;
        action["id"] = aid.str();
        action["depends_on"] = Json::Value(Json::arrayValue);
        Json::Value robot_json(Json::objectValue);
        robot_json["id"] = robot.id;
        robot_json["name"] = robot.name;
        if (!robot.end_effector_link.empty())
        {
            robot_json["end_effector_link"] = robot.end_effector_link;
        }
        action["robot"] = robot_json;

        // Trajectory payload
        Json::Value traj_json(Json::objectValue);
        traj_json["robot_id"] = robot.id;
        traj_json["robot_name"] = robot.name;

        Json::Value points(Json::arrayValue);
        Json::Value times_json(Json::arrayValue);
        std::vector<double> goal(robot.dof, 0.0);
        double t0 = 0.0;
        if (robot.id >= 0 && robot.id < static_cast<int>(trajectories.size()))
        {
            const auto &traj = trajectories[robot.id];
            if (!traj.times.empty())
            {
                t0 = traj.times.front();
            }
            for (std::size_t i = 0; i < traj.trajectory.size(); ++i)
            {
                Json::Value p(Json::objectValue);
                const double t = (i < traj.times.size()) ? (traj.times[i] - t0) : (static_cast<double>(i) * 0.01);
                p["joint_positions"] = vec_to_json(traj.trajectory[i].joint_values);
                points.append(p);
                times_json.append(std::max(0.0, t));
            }
            if (!traj.trajectory.empty())
            {
                goal = traj.trajectory.back().joint_values;
            }
        }
        traj_json["points"] = points;
        traj_json["times"] = times_json;
        action["trajectory"] = traj_json;

        Json::Value goal_json(Json::objectValue);
        goal_json["joint_positions"] = vec_to_json(goal);
        action["goal"] = goal_json;

        actions.append(action);
    }
    root["actions"] = actions;

    return root;
}

Json::Value make_activity_graph_plan(const std::vector<RobotSpec> &robots,
                                     const std::shared_ptr<ActivityGraph> &act_graph,
                                     const MRTrajectory &trajectories,
                                     const ExportOptions &options,
                                     const ActivityGraphExportOptions &export_options)
{
    Json::Value root(Json::objectValue);
    root["schema"] = "aidf.skillplan";
    root["schema_version"] = 1;
    if (!options.plan_name.empty())
    {
        root["plan_name"] = options.plan_name;
    }
    if (export_options.pad_to_next_activity)
    {
        Json::Value meta(Json::objectValue);
        meta["padded_action_times"] = true;
        root["mr_planner"] = meta;
    }

    // robots[]
    Json::Value robots_json(Json::arrayValue);
    for (const auto &robot : robots)
    {
        Json::Value r(Json::objectValue);
        r["id"] = robot.id;
        r["name"] = robot.name;
        r["dof"] = robot.dof;
        if (!robot.end_effector_link.empty())
        {
            r["end_effector_link"] = robot.end_effector_link;
        }
        robots_json.append(r);
    }
    root["robots"] = robots_json;

    // initial_scene
    Json::Value init_scene(Json::objectValue);
    Json::Value init_robots(Json::arrayValue);
    for (const auto &robot : robots)
    {
        Json::Value rs(Json::objectValue);
        rs["robot_id"] = robot.id;
        rs["robot_name"] = robot.name;

        std::vector<double> start(robot.dof, 0.0);
        std::vector<double> start_hand;
        if (robot.id >= 0 && robot.id < static_cast<int>(trajectories.size()))
        {
            const auto &traj = trajectories[static_cast<std::size_t>(robot.id)];
            if (!traj.trajectory.empty())
            {
                start = traj.trajectory.front().joint_values;
                start_hand = traj.trajectory.front().hand_values;
            }
        }
        rs["joint_positions"] = vec_to_json(start);
        if (!start_hand.empty())
        {
            rs["hand_positions"] = vec_to_json(start_hand);
        }
        init_robots.append(rs);
    }
    init_scene["robots"] = init_robots;

    Json::Value init_objects(Json::arrayValue);
    if (export_options.include_initial_scene_objects && act_graph)
    {
        for (const auto &obj_ptr : act_graph->get_obj_nodes())
        {
            if (!obj_ptr)
            {
                continue;
            }
            if (obj_ptr->prev_detach != nullptr)
            {
                continue;  // not part of the initial scene snapshot
            }
            init_objects.append(object_to_json(obj_ptr->obj, obj_ptr->vanish, obj_ptr->fixed, obj_ptr->handover));
        }
    }
    init_scene["objects"] = init_objects;
    root["initial_scene"] = init_scene;

    // environment (optional)
    if (!options.environment_name.empty() || !options.backend_type.empty() || !options.moveit_group_name.empty())
    {
        Json::Value env(Json::objectValue);
        if (!options.environment_name.empty())
        {
            env["name"] = options.environment_name;
        }
        if (!options.backend_type.empty() || !options.moveit_group_name.empty() || !options.moveit_config_pkg.empty())
        {
            Json::Value backend(Json::objectValue);
            if (!options.backend_type.empty())
            {
                backend["type"] = options.backend_type;
            }
            if (!options.moveit_group_name.empty())
            {
                backend["moveit_group_name"] = options.moveit_group_name;
            }
            if (!options.moveit_config_pkg.empty())
            {
                backend["moveitConfigPkg"] = options.moveit_config_pkg;
            }
            backend["l1_vmax"] = options.l1_vmax;
            env["backend"] = backend;
        }
        root["environment"] = env;
    }

    // actions[]
    Json::Value actions_json(Json::arrayValue);
    if (!act_graph)
    {
        root["actions"] = actions_json;
        return root;
    }

    const int num_robots = act_graph->num_robots();
    std::vector<int> robot_dofs(static_cast<std::size_t>(num_robots), 0);
    std::vector<std::string> robot_names(static_cast<std::size_t>(num_robots));
    std::vector<std::string> ee_links(static_cast<std::size_t>(num_robots));
    for (const auto &r : robots)
    {
        if (r.id >= 0 && r.id < num_robots)
        {
            robot_dofs[static_cast<std::size_t>(r.id)] = r.dof;
            robot_names[static_cast<std::size_t>(r.id)] = r.name;
            ee_links[static_cast<std::size_t>(r.id)] = r.end_effector_link;
        }
    }

    // Precompute start and "next activity start" timestamps for each act_id from the provided trajectories.
    // This is used to preserve inter-activity time gaps (e.g., removed wait segments) when exporting per-action
    // relative times.
    const double kNaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<std::vector<double>> first_time(static_cast<std::size_t>(num_robots));
    std::vector<std::vector<double>> next_start_time(static_cast<std::size_t>(num_robots));
    for (int rid = 0; rid < num_robots; ++rid)
    {
        const int nacts = act_graph->num_activities(rid);
        first_time[static_cast<std::size_t>(rid)].assign(static_cast<std::size_t>(nacts), kNaN);
        next_start_time[static_cast<std::size_t>(rid)].assign(static_cast<std::size_t>(nacts), kNaN);
        if (rid < 0 || rid >= static_cast<int>(trajectories.size()))
        {
            continue;
        }
        const auto &rt = trajectories[static_cast<std::size_t>(rid)];
        const std::size_t n = std::min(rt.act_ids.size(), rt.times.size());
        for (std::size_t k = 0; k < n; ++k)
        {
            const int act_id = rt.act_ids[k];
            if (act_id < 0 || act_id >= nacts)
            {
                continue;
            }
            auto &ft = first_time[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)];
            if (std::isnan(ft))
            {
                ft = rt.times[k];
            }
            if (k == 0)
            {
                continue;
            }
            const int prev_act = rt.act_ids[k - 1];
            if (prev_act == act_id)
            {
                continue;
            }
            if (prev_act < 0 || prev_act >= nacts)
            {
                continue;
            }
            auto &nt = next_start_time[static_cast<std::size_t>(rid)][static_cast<std::size_t>(prev_act)];
            if (std::isnan(nt))
            {
                nt = rt.times[k];
            }
        }
    }

    // Decide which activities to keep.
    std::vector<std::vector<bool>> keep(static_cast<std::size_t>(num_robots));
    int max_acts = 0;
    for (int rid = 0; rid < num_robots; ++rid)
    {
        const int n = act_graph->num_activities(rid);
        max_acts = std::max(max_acts, n);
        keep[static_cast<std::size_t>(rid)].assign(static_cast<std::size_t>(n), true);
        if (!export_options.prune_stationary)
        {
            continue;
        }
        for (int act_id = 0; act_id < n; ++act_id)
        {
            if (act_id == 0)
            {
                keep[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)] = true;
                continue;
            }
            auto act = act_graph->get(rid, act_id);
            if (!act)
            {
                keep[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)] = false;
                continue;
            }
            const bool moved = !poses_close_maxabs(act->start_pose, act->end_pose, export_options.motion_epsilon);
            const bool has_scene_updates =
                !act->obj_attached.empty() || !act->obj_detached.empty() || !act->collision_nodes.empty();
            const bool has_type2 = !act->type2_prev.empty() || !act->type2_next.empty();
            const bool type_changed =
                (act->type1_prev != nullptr && act->type1_prev->type != act->type);
            bool has_time_span = false;
            if (export_options.pad_to_next_activity)
            {
                const auto &ft = first_time[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)];
                const auto &nt = next_start_time[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)];
                if (!std::isnan(ft) && !std::isnan(nt))
                {
                    has_time_span = (nt - ft) > 1e-9;
                }
            }
            keep[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)] =
                moved || has_scene_updates || has_type2 || type_changed || has_time_span;
        }
    }

    struct ActKey
    {
        int robot_id{-1};
        int act_id{-1};
    };
    std::map<std::pair<int, int>, std::string> action_id_by_act;
    struct PendingAction
    {
        int robot_id{-1};
        int act_id{-1};
        ActPtr act;
        Json::Value json;
    };
    std::vector<PendingAction> pending;
    pending.reserve(1024);

    int action_index = 0;
    for (int act_id = 0; act_id < max_acts; ++act_id)
    {
        for (int rid = 0; rid < num_robots; ++rid)
        {
            if (act_id < 0 || act_id >= act_graph->num_activities(rid))
            {
                continue;
            }
            if (!keep[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)])
            {
                continue;
            }

            auto act = act_graph->get(rid, act_id);
            if (!act)
            {
                continue;
            }

            Json::Value action(Json::objectValue);
            const std::string aid = format_action_id(action_index++);
            action["id"] = aid;
            action["depends_on"] = Json::Value(Json::arrayValue);  // filled later
            action["key"] = format_action_key(rid, act_id, act->type_string());

            Json::Value activity(Json::objectValue);
            activity["type"] = act->type_string();
            activity["robot_id"] = rid;
            activity["act_id"] = act_id;
            action["activity"] = activity;
            action["activity_type"] = act->type_string();

            // AIDF-style fields for skill sequence playback/compatibility.
            {
                Json::Value meta_skill(Json::objectValue);
                meta_skill["index"] = 0;
                meta_skill["name"] = options.plan_name.empty() ? "mr_planner" : options.plan_name;
                action["meta_skill"] = meta_skill;
            }
            {
                Json::Value skill(Json::objectValue);
                skill["index_in_meta"] = act_id;
                skill["name"] = act->type_string();
                action["skill"] = skill;
            }

            Json::Value robot_json(Json::objectValue);
            robot_json["id"] = rid;
            robot_json["name"] = robot_names[static_cast<std::size_t>(rid)].empty()
                                     ? ("robot_" + std::to_string(rid))
                                     : robot_names[static_cast<std::size_t>(rid)];
            if (!ee_links[static_cast<std::size_t>(rid)].empty())
            {
                robot_json["end_effector_link"] = ee_links[static_cast<std::size_t>(rid)];
            }
            action["robot"] = robot_json;

            // Trajectory payload (segment for this activity).
            Json::Value traj_json(Json::objectValue);
            traj_json["robot_id"] = rid;
            traj_json["robot_name"] = robot_json["name"];

            Json::Value points(Json::arrayValue);
            Json::Value times_json(Json::arrayValue);
            std::vector<double> goal(robot_dofs[static_cast<std::size_t>(rid)], 0.0);
            std::vector<double> goal_hand;
            double t0 = 0.0;
            bool have_t0 = false;
            bool padded_end_marker = false;
            double action_duration_rel = 0.0;

            if (rid >= 0 && rid < static_cast<int>(trajectories.size()))
            {
                const auto &rt = trajectories[static_cast<std::size_t>(rid)];
                for (std::size_t k = 0; k < rt.trajectory.size() && k < rt.act_ids.size(); ++k)
                {
                    if (rt.act_ids[k] != act_id)
                    {
                        continue;
                    }
                    const double tk = (k < rt.times.size()) ? rt.times[k] : static_cast<double>(points.size()) * 0.01;
                    if (!have_t0)
                    {
                        t0 = tk;
                        have_t0 = true;
                    }

                    Json::Value p(Json::objectValue);
                    const double t_rel = std::max(0.0, tk - t0);
                    p["joint_positions"] = vec_to_json(rt.trajectory[k].joint_values);
                    if (!rt.trajectory[k].hand_values.empty())
                    {
                        p["hand_positions"] = vec_to_json(rt.trajectory[k].hand_values);
                        goal_hand = rt.trajectory[k].hand_values;
                    }
                    points.append(p);
                    times_json.append(t_rel);
                    goal = rt.trajectory[k].joint_values;
                }
            }

            if (export_options.pad_to_next_activity && have_t0 && rid >= 0 && rid < num_robots)
            {
                const auto &nt = next_start_time[static_cast<std::size_t>(rid)];
                if (act_id >= 0 && act_id < static_cast<int>(nt.size()))
                {
                    const double next_t = nt[static_cast<std::size_t>(act_id)];
                    if (!std::isnan(next_t))
                    {
                        const double pad_rel = std::max(0.0, next_t - t0);
                        action_duration_rel = pad_rel;
                        double last_rel = 0.0;
                        if (times_json.isArray() && times_json.size() > 0 && times_json[times_json.size() - 1].isNumeric())
                        {
                            last_rel = times_json[times_json.size() - 1].asDouble();
                        }
                        if (pad_rel > last_rel + 1e-9)
                        {
                            Json::Value p(Json::objectValue);
                            p["joint_positions"] = vec_to_json(goal);
                            if (!goal_hand.empty())
                            {
                                p["hand_positions"] = vec_to_json(goal_hand);
                            }
                            points.append(p);
                            times_json.append(pad_rel);
                            padded_end_marker = true;
                        }
                    }
                }
            }

            if (points.empty())
            {
                // Fallback to start/end pose snapshots.
                Json::Value p0(Json::objectValue);
                p0["joint_positions"] = vec_to_json(act->start_pose.joint_values);
                if (!act->start_pose.hand_values.empty())
                {
                    p0["hand_positions"] = vec_to_json(act->start_pose.hand_values);
                }
                points.append(p0);
                times_json.append(0.0);

                Json::Value p1(Json::objectValue);
                p1["joint_positions"] = vec_to_json(act->end_pose.joint_values);
                if (!act->end_pose.hand_values.empty())
                {
                    p1["hand_positions"] = vec_to_json(act->end_pose.hand_values);
                }
                points.append(p1);
                times_json.append(0.1);

                goal = act->end_pose.joint_values;
                goal_hand = act->end_pose.hand_values;
            }

            if (export_options.pad_to_next_activity)
            {
                if (action_duration_rel <= 0.0)
                {
                    if (times_json.isArray() && times_json.size() > 0 && times_json[times_json.size() - 1].isNumeric())
                    {
                        action_duration_rel = times_json[times_json.size() - 1].asDouble();
                    }
                }
                Json::Value meta(Json::objectValue);
                meta["duration"] = action_duration_rel;
                meta["padded_end_marker"] = padded_end_marker;
                action["mr_planner"] = meta;
            }

            traj_json["points"] = points;
            traj_json["times"] = times_json;
            action["trajectory"] = traj_json;

            Json::Value goal_json(Json::objectValue);
            goal_json["joint_positions"] = vec_to_json(goal);
            if (!goal_hand.empty())
            {
                goal_json["hand_positions"] = vec_to_json(goal_hand);
            }
            action["goal"] = goal_json;

            if (export_options.include_scene_updates)
            {
                Json::Value scene_updates(Json::objectValue);

                Json::Value attachments(Json::arrayValue);
                for (const auto &obj_ptr : act->obj_attached)
                {
                    if (!obj_ptr)
                    {
                        continue;
                    }
                    Json::Value upd(Json::objectValue);
                    upd["action"] = "attach";
                    upd["robot_id"] = rid;
                    upd["link"] = obj_ptr->next_attach_link;
                    upd["object"] = obj_ptr->obj.name;
                    upd["relative_pose"] =
                        pose_xyzquat_to_json(obj_ptr->obj.x_attach,
                                             obj_ptr->obj.y_attach,
                                             obj_ptr->obj.z_attach,
                                             obj_ptr->obj.qx_attach,
                                             obj_ptr->obj.qy_attach,
                                             obj_ptr->obj.qz_attach,
                                             obj_ptr->obj.qw_attach);
                    attachments.append(upd);
                }
                for (const auto &obj_ptr : act->obj_detached)
                {
                    if (!obj_ptr)
                    {
                        continue;
                    }
                    Json::Value upd(Json::objectValue);
                    upd["action"] = "detach";
                    upd["robot_id"] = rid;
                    upd["link"] = obj_ptr->obj.parent_link;
                    upd["object"] = obj_ptr->obj.name;
                    upd["relative_pose"] =
                        pose_xyzquat_to_json(obj_ptr->obj.x_attach,
                                             obj_ptr->obj.y_attach,
                                             obj_ptr->obj.z_attach,
                                             obj_ptr->obj.qx_attach,
                                             obj_ptr->obj.qy_attach,
                                             obj_ptr->obj.qz_attach,
                                             obj_ptr->obj.qw_attach);
                    attachments.append(upd);
                }
                scene_updates["attachments"] = attachments;

                Json::Value col_allow(Json::arrayValue);
                Json::Value col_disallow(Json::arrayValue);
                for (const auto &col : act->collision_nodes)
                {
                    Json::Value entry(Json::objectValue);
                    entry["link"] = col.link_name;
                    entry["object"] = col.obj_name;
                    if (col.allow)
                    {
                        col_allow.append(entry);
                    }
                    else
                    {
                        col_disallow.append(entry);
                    }
                }
                scene_updates["collisions_allow"] = col_allow;
                scene_updates["collisions_disallow"] = col_disallow;

                Json::Value obj_updates(Json::objectValue);
                obj_updates["add"] = Json::Value(Json::arrayValue);
                obj_updates["remove"] = Json::Value(Json::arrayValue);
                Json::Value update(Json::arrayValue);
                for (const auto &obj_ptr : act->obj_detached)
                {
                    if (!obj_ptr)
                    {
                        continue;
                    }
                    update.append(object_to_json(obj_ptr->obj, obj_ptr->vanish, obj_ptr->fixed, obj_ptr->handover));
                }
                obj_updates["update"] = update;
                scene_updates["objects"] = obj_updates;

                action["scene_updates"] = scene_updates;
            }

            action_id_by_act[{rid, act_id}] = aid;
            PendingAction entry;
            entry.robot_id = rid;
            entry.act_id = act_id;
            entry.act = act;
            entry.json = std::move(action);
            pending.push_back(std::move(entry));
        }
    }

    // Fill depends_on from type2_prev edges.
    for (auto &entry : pending)
    {
        Json::Value deps(Json::arrayValue);
        if (entry.act)
        {
            for (const auto &dep : entry.act->type2_prev)
            {
                if (!dep)
                {
                    continue;
                }
                const auto it = action_id_by_act.find({dep->robot_id, dep->act_id});
                if (it != action_id_by_act.end())
                {
                    deps.append(it->second);
                }
            }
        }
        entry.json["depends_on"] = deps;
        actions_json.append(entry.json);
    }

    root["actions"] = actions_json;
    return root;
}

bool write_json_to_file(const Json::Value &json, const std::string &path, std::string *error)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        if (error)
        {
            *error = format_error(path, "failed to open for writing");
        }
        return false;
    }

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(json, &file);
    file << "\n";
    if (!file.good())
    {
        if (error)
        {
            *error = format_error(path, "failed while writing");
        }
        return false;
    }
    return true;
}

bool read_json_from_file(const std::string &path, Json::Value *json, std::string *error)
{
    if (!json)
    {
        if (error)
        {
            *error = "null output parameter";
        }
        return false;
    }

    std::ifstream file(path);
    if (!file.is_open())
    {
        if (error)
        {
            *error = format_error(path, "failed to open for reading");
        }
        return false;
    }

    Json::CharReaderBuilder builder;
    std::string errs;
    if (!Json::parseFromStream(builder, file, json, &errs))
    {
        if (error)
        {
            *error = format_error(path, errs);
        }
        return false;
    }
    return true;
}

bool extract_robot_trajectories(const Json::Value &plan, MRTrajectory *out, std::string *error)
{
    if (!out)
    {
        if (error)
        {
            *error = "null output parameter";
        }
        return false;
    }
    out->clear();

    const auto robots_json = plan["robots"];
    if (!robots_json.isArray())
    {
        if (error)
        {
            *error = "robots must be an array";
        }
        return false;
    }

    struct RobotInfo
    {
        std::string name;
        int dof{0};
    };
    std::map<int, RobotInfo> robots;
    int max_id = -1;
    for (Json::ArrayIndex i = 0; i < robots_json.size(); ++i)
    {
        const auto &r = robots_json[i];
        int id = -1;
        int dof = 0;
        std::string name;
        if (!as_int(r["id"], &id) || !as_int(r["dof"], &dof) || !as_string(r["name"], &name))
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "robots[" << i << "] missing required fields";
                *error = oss.str();
            }
            return false;
        }
        robots[id] = RobotInfo{name, dof};
        max_id = std::max(max_id, id);
    }

    if (max_id < 0)
    {
        return true;
    }

    out->resize(static_cast<std::size_t>(max_id + 1));
    for (const auto &[rid, info] : robots)
    {
        if (rid < 0)
        {
            continue;
        }
        (*out)[rid].robot_id = rid;
    }
    std::vector<int> next_act_id(out->size(), 0);

    const auto actions_json = plan["actions"];
    if (!actions_json.isArray())
    {
        if (error)
        {
            *error = "actions must be an array";
        }
        return false;
    }

    // Infer a nominal timestep from trajectory.times arrays. This helps reconstruct the original
    // per-robot timeline when skillplan actions store times relative to each action.
    double dt_hint = 0.0;
    for (Json::ArrayIndex i = 0; i < actions_json.size(); ++i)
    {
        const auto &action = actions_json[i];
        if (!action.isObject())
        {
            continue;
        }
        const auto traj = action["trajectory"];
        const auto times = traj["times"];
        if (!times.isArray() || times.size() < 2)
        {
            continue;
        }
        for (Json::ArrayIndex k = 1; k < times.size(); ++k)
        {
            if (!times[k - 1].isNumeric() || !times[k].isNumeric())
            {
                continue;
            }
            const double dt = times[k].asDouble() - times[k - 1].asDouble();
            if (dt > 1e-9 && (dt_hint <= 0.0 || dt < dt_hint))
            {
                dt_hint = dt;
            }
        }
    }
    if (dt_hint <= 0.0)
    {
        dt_hint = 0.01;
    }

    const bool padded_action_times =
        plan.isMember("mr_planner") && plan["mr_planner"].isObject() && plan["mr_planner"].isMember("padded_action_times") &&
        plan["mr_planner"]["padded_action_times"].isBool() && plan["mr_planner"]["padded_action_times"].asBool();

    // Pre-count actions per robot so we can treat padded end markers (added to preserve inter-action timing)
    // differently for the last action.
    std::vector<int> action_counts(out->size(), 0);
    for (Json::ArrayIndex i = 0; i < actions_json.size(); ++i)
    {
        const auto &action = actions_json[i];
        int rid = -1;
        if (action.isObject() && as_int(action["robot"]["id"], &rid) && rid >= 0 && rid < static_cast<int>(out->size()))
        {
            action_counts[static_cast<std::size_t>(rid)]++;
        }
    }

    // Maintain per-robot time cursors so action sequencing can preserve gaps even when
    // trajectory samples omit intermediate wait waypoints.
    std::vector<double> time_cursor(out->size(), 0.0);

    for (Json::ArrayIndex i = 0; i < actions_json.size(); ++i)
    {
        const auto &action = actions_json[i];
        int rid = -1;
        if (!as_int(action["robot"]["id"], &rid))
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << i << "].robot.id missing or not an int";
                *error = oss.str();
            }
            return false;
        }
        auto rit = robots.find(rid);
        if (rit == robots.end())
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << i << "].robot.id=" << rid << " not declared in robots[]";
                *error = oss.str();
            }
            return false;
        }
        if (rid < 0 || rid >= static_cast<int>(out->size()))
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << i << "].robot.id=" << rid << " out of range";
                *error = oss.str();
            }
            return false;
        }

        const int dof = rit->second.dof;
        const std::string &robot_name = rit->second.name;
        const int act_id = next_act_id[static_cast<std::size_t>(rid)]++;
        const bool is_last_action_for_robot =
            (rid >= 0 && rid < static_cast<int>(action_counts.size())) &&
            (act_id >= (action_counts[static_cast<std::size_t>(rid)] - 1));

        struct TrajPoint
        {
            double t{0.0};
            std::vector<double> joints;
            std::vector<double> hands;
        };
        std::vector<TrajPoint> points;

        const auto traj = action["trajectory"];
        const auto traj_points = traj["points"];
        if (traj.isObject() && traj_points.isArray() && !traj_points.empty())
        {
            double last_t = 0.0;
            bool have_last_t = false;
            for (Json::ArrayIndex pidx = 0; pidx < traj_points.size(); ++pidx)
            {
                const auto &p = traj_points[pidx];
                std::vector<double> joints;
                if (!parse_joint_positions(p["joint_positions"], dof, &joints))
                {
                    if (error)
                    {
                        std::ostringstream oss;
                        oss << "actions[" << i << "].trajectory.points[" << pidx << "].joint_positions invalid";
                        *error = oss.str();
                    }
                    return false;
                }
                std::vector<double> hands;
                if (p.isMember("hand_positions") && p["hand_positions"].isArray())
                {
                    if (!parse_joint_positions(p["hand_positions"], 0, &hands))
                    {
                        if (error)
                        {
                            std::ostringstream oss;
                            oss << "actions[" << i << "].trajectory.points[" << pidx << "].hand_positions invalid";
                            *error = oss.str();
                        }
                        return false;
                    }
                }

                double t = 0.0;
                const auto times_array = traj["times"];
                if (times_array.isArray() && pidx < times_array.size() && times_array[pidx].isNumeric())
                {
                    t = times_array[pidx].asDouble();
                }
                else if (p.isMember("t") && p["t"].isNumeric())
                {
                    t = p["t"].asDouble();
                }
                else
                {
                    t = have_last_t ? (last_t + 0.01) : 0.0;
                }
                if (have_last_t && t < last_t)
                {
                    t = last_t;
                }
                last_t = t;
                have_last_t = true;
                TrajPoint pt;
                pt.t = t;
                pt.joints = std::move(joints);
                pt.hands = std::move(hands);
                points.push_back(std::move(pt));
            }
        }
        else if (action.isMember("goal") && action["goal"].isObject())
        {
            std::vector<double> joints;
            if (parse_joint_positions(action["goal"]["joint_positions"], dof, &joints))
            {
                TrajPoint pt;
                pt.t = 0.0;
                pt.joints = std::move(joints);
                if (action["goal"].isMember("hand_positions") && action["goal"]["hand_positions"].isArray())
                {
                    parse_joint_positions(action["goal"]["hand_positions"], 0, &pt.hands);
                }
                points.push_back(std::move(pt));
            }
        }

        if (points.empty())
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << i << "] has no usable trajectory.points or goal.joint_positions";
                *error = oss.str();
            }
            return false;
        }

        // Normalize times within this action so the first point starts at t=0.
        const double action_t0 = points.front().t;
        for (auto &pt : points)
        {
            pt.t = std::max(0.0, pt.t - action_t0);
        }

        double action_duration = points.back().t;
        bool padded_end_marker = false;
        bool have_padded_end_marker = false;
        if (padded_action_times && action.isObject() && action.isMember("mr_planner") && action["mr_planner"].isObject())
        {
            const auto &meta = action["mr_planner"];
            if (meta.isMember("duration") && meta["duration"].isNumeric())
            {
                action_duration = std::max(0.0, meta["duration"].asDouble());
            }
            if (meta.isMember("padded_end_marker") && meta["padded_end_marker"].isBool())
            {
                padded_end_marker = meta["padded_end_marker"].asBool();
                have_padded_end_marker = true;
            }
        }
        if (padded_action_times && !is_last_action_for_robot && points.size() >= 2)
        {
            // Skillplans exported with padded_action_times append a stationary end marker so the last timestamp equals
            // the next action's absolute start time. Drop the marker when constructing per-robot trajectories to avoid
            // duplicating boundary samples; keep action_duration to advance the time cursor correctly.
            bool should_drop = padded_end_marker;
            if (!have_padded_end_marker)
            {
                const auto &last = points.back();
                const auto &prev = points[points.size() - 2];
                should_drop = (last.joints == prev.joints && last.hands == prev.hands && last.t > prev.t + 1e-9);
            }
            if (should_drop)
            {
                points.pop_back();
            }
        }

        auto &traj_out = (*out)[rid];
        const double offset = time_cursor[static_cast<std::size_t>(rid)];
        const std::size_t act_start_samples = traj_out.act_ids.size();

        for (std::size_t k = 0; k < points.size(); ++k)
        {
            double t_abs = offset + points[k].t;
            if (!traj_out.times.empty() && t_abs <= traj_out.times.back() + 1e-12)
            {
                // Avoid duplicate timestamps which can break downstream interpolation.
                t_abs = traj_out.times.back() + 1e-6;
            }

            RobotPose pose;
            pose.robot_id = rid;
            pose.robot_name = robot_name;
            pose.joint_values = points[k].joints;
            pose.hand_values = points[k].hands;
            traj_out.trajectory.push_back(std::move(pose));
            traj_out.times.push_back(t_abs);
            traj_out.act_ids.push_back(act_id);
        }

        // Ensure every action contributes at least one sample so downstream ADG reconstruction
        // can locate a start/end pose for each act_id.
        if (traj_out.act_ids.size() == act_start_samples)
        {
            RobotPose pose;
            pose.robot_id = rid;
            pose.robot_name = robot_name;
            pose.joint_values = points.back().joints;
            pose.hand_values = points.back().hands;

            double t_abs = traj_out.times.empty() ? 0.0 : (traj_out.times.back() + 1e-6);
            traj_out.trajectory.push_back(std::move(pose));
            traj_out.times.push_back(t_abs);
            traj_out.act_ids.push_back(act_id);
        }

        // Advance the absolute time cursor for this robot to the next action's start time.
        time_cursor[static_cast<std::size_t>(rid)] =
            offset + action_duration + (padded_action_times ? 0.0 : dt_hint);
    }

    for (auto &traj : *out)
    {
        if (!traj.times.empty())
        {
            const double t0 = traj.times.front();
            for (auto &t : traj.times)
            {
                t = std::max(0.0, t - t0);
            }
            traj.cost = traj.times.back();
        }
    }

    return true;
}

bool extract_synchronized_trajectory(const Json::Value &plan, MRTrajectory *out, std::string *error)
{
    if (!out)
    {
        if (error)
        {
            *error = "null output parameter";
        }
        return false;
    }
    out->clear();

    const auto robots_json = plan["robots"];
    if (!robots_json.isArray())
    {
        if (error)
        {
            *error = "robots must be an array";
        }
        return false;
    }

    struct RobotInfo
    {
        std::string name;
        int dof{0};
    };
    std::map<int, RobotInfo> robots;
    int max_id = -1;
    for (Json::ArrayIndex i = 0; i < robots_json.size(); ++i)
    {
        const auto &r = robots_json[i];
        int id = -1;
        int dof = 0;
        std::string name;
        if (!as_int(r["id"], &id) || !as_int(r["dof"], &dof) || !as_string(r["name"], &name))
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "robots[" << i << "] missing required fields";
                *error = oss.str();
            }
            return false;
        }
        robots[id] = RobotInfo{name, dof};
        max_id = std::max(max_id, id);
    }

    if (max_id < 0)
    {
        return true;
    }

    out->resize(static_cast<std::size_t>(max_id + 1));
    std::vector<RobotPose> current_pose(out->size());
    for (const auto &[rid, info] : robots)
    {
        if (rid < 0 || rid >= static_cast<int>(out->size()))
        {
            continue;
        }
        (*out)[rid].robot_id = rid;
        RobotPose pose;
        pose.robot_id = rid;
        pose.robot_name = info.name;
        pose.joint_values.assign(static_cast<std::size_t>(std::max(0, info.dof)), 0.0);
        current_pose[static_cast<std::size_t>(rid)] = std::move(pose);
    }

    // Seed initial robot poses (best effort).
    if (plan.isMember("initial_scene") && plan["initial_scene"].isObject() && plan["initial_scene"].isMember("robots") &&
        plan["initial_scene"]["robots"].isArray())
    {
        const auto init = plan["initial_scene"]["robots"];
        for (Json::ArrayIndex i = 0; i < init.size(); ++i)
        {
            const auto &r = init[i];
            int rid = -1;
            if (!as_int(r["robot_id"], &rid))
            {
                continue;
            }
            auto it = robots.find(rid);
            if (it == robots.end())
            {
                continue;
            }
            if (rid < 0 || rid >= static_cast<int>(current_pose.size()))
            {
                continue;
            }
            std::vector<double> joints;
            if (parse_joint_positions(r["joint_positions"], it->second.dof, &joints))
            {
                current_pose[static_cast<std::size_t>(rid)].joint_values = std::move(joints);
            }
            if (r.isMember("hand_positions") && r["hand_positions"].isArray())
            {
                std::vector<double> hands;
                if (parse_joint_positions(r["hand_positions"], 0, &hands))
                {
                    current_pose[static_cast<std::size_t>(rid)].hand_values = std::move(hands);
                }
            }
        }
    }

    // Emit a shared t=0 sample for all robots.
    const double kEps = 1e-6;
    for (std::size_t rid = 0; rid < out->size(); ++rid)
    {
        (*out)[rid].trajectory.push_back(current_pose[rid]);
        (*out)[rid].times.push_back(0.0);
        (*out)[rid].act_ids.push_back(0);
    }
    double global_time = kEps;
    int global_act_id = 0;

    const auto actions_json = plan["actions"];
    if (!actions_json.isArray())
    {
        if (error)
        {
            *error = "actions must be an array";
        }
        return false;
    }

    struct TrajPoint
    {
        double t{0.0};
        std::vector<double> joints;
        std::vector<double> hands;
    };
    struct ParsedAction
    {
        int robot_id{-1};
        int step_id{-1};
        std::vector<TrajPoint> points;
        double duration{0.0};
        Json::ArrayIndex index{0};
    };

    auto parse_action_points = [&](const Json::Value &action,
                                   Json::ArrayIndex action_index,
                                   ParsedAction *out_action) -> bool {
        if (!out_action)
        {
            return false;
        }
        out_action->index = action_index;
        out_action->step_id = -1;
        if (action.isMember("activity") && action["activity"].isObject() && action["activity"]["act_id"].isInt())
        {
            out_action->step_id = action["activity"]["act_id"].asInt();
        }

        int rid = -1;
        if (!as_int(action["robot"]["id"], &rid))
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << action_index << "].robot.id missing or not an int";
                *error = oss.str();
            }
            return false;
        }
        out_action->robot_id = rid;
        const auto rit = robots.find(rid);
        if (rit == robots.end())
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << action_index << "].robot.id=" << rid << " not declared in robots[]";
                *error = oss.str();
            }
            return false;
        }
        if (rid < 0 || rid >= static_cast<int>(out->size()))
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << action_index << "].robot.id=" << rid << " out of range";
                *error = oss.str();
            }
            return false;
        }

        const int dof = rit->second.dof;
        std::vector<TrajPoint> points;

        const auto traj = action["trajectory"];
        const auto traj_points = traj["points"];
        if (traj.isObject() && traj_points.isArray() && !traj_points.empty())
        {
            double last_t = 0.0;
            bool have_last_t = false;
            for (Json::ArrayIndex pidx = 0; pidx < traj_points.size(); ++pidx)
            {
                const auto &p = traj_points[pidx];
                std::vector<double> joints;
                if (!parse_joint_positions(p["joint_positions"], dof, &joints))
                {
                    if (error)
                    {
                        std::ostringstream oss;
                        oss << "actions[" << action_index << "].trajectory.points[" << pidx << "].joint_positions invalid";
                        *error = oss.str();
                    }
                    return false;
                }
                std::vector<double> hands;
                if (p.isMember("hand_positions") && p["hand_positions"].isArray())
                {
                    if (!parse_joint_positions(p["hand_positions"], 0, &hands))
                    {
                        if (error)
                        {
                            std::ostringstream oss;
                            oss << "actions[" << action_index << "].trajectory.points[" << pidx << "].hand_positions invalid";
                            *error = oss.str();
                        }
                        return false;
                    }
                }

                double t = 0.0;
                const auto times_array = traj["times"];
                if (times_array.isArray() && pidx < times_array.size() && times_array[pidx].isNumeric())
                {
                    t = times_array[pidx].asDouble();
                }
                else if (p.isMember("t") && p["t"].isNumeric())
                {
                    t = p["t"].asDouble();
                }
                else
                {
                    t = have_last_t ? (last_t + 0.01) : 0.0;
                }
                if (have_last_t && t < last_t)
                {
                    t = last_t;
                }
                last_t = t;
                have_last_t = true;
                points.push_back(TrajPoint{t, std::move(joints), std::move(hands)});
            }
        }
        else if (action.isMember("goal") && action["goal"].isObject())
        {
            std::vector<double> joints;
            if (parse_joint_positions(action["goal"]["joint_positions"], dof, &joints))
            {
                TrajPoint pt;
                pt.t = 0.0;
                pt.joints = std::move(joints);
                if (action["goal"].isMember("hand_positions") && action["goal"]["hand_positions"].isArray())
                {
                    parse_joint_positions(action["goal"]["hand_positions"], 0, &pt.hands);
                }
                points.push_back(std::move(pt));
            }
        }

        if (points.empty())
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "actions[" << action_index << "] has no usable trajectory.points or goal.joint_positions";
                *error = oss.str();
            }
            return false;
        }

        // Normalize times within this action so the first point starts at t=0.
        const double action_t0 = points.front().t;
        for (auto &pt : points)
        {
            pt.t = std::max(0.0, pt.t - action_t0);
        }
        double duration = 0.0;
        for (const auto &pt : points)
        {
            duration = std::max(duration, pt.t);
        }

        out_action->points = std::move(points);
        out_action->duration = duration;
        return true;
    };

    Json::ArrayIndex i = 0;
    while (i < actions_json.size())
    {
        int step_id = -1;
        const auto &a0 = actions_json[i];
        if (a0.isObject() && a0.isMember("activity") && a0["activity"].isObject() && a0["activity"]["act_id"].isInt())
        {
            step_id = a0["activity"]["act_id"].asInt();
        }

        std::vector<ParsedAction> group;
        Json::ArrayIndex j = i;
        while (j < actions_json.size())
        {
            const auto &aj = actions_json[j];
            int aj_step = -1;
            if (aj.isObject() && aj.isMember("activity") && aj["activity"].isObject() && aj["activity"]["act_id"].isInt())
            {
                aj_step = aj["activity"]["act_id"].asInt();
            }
            if (step_id < 0 || aj_step != step_id)
            {
                break;
            }
            ParsedAction pa;
            if (!parse_action_points(aj, j, &pa))
            {
                return false;
            }
            group.push_back(std::move(pa));
            ++j;
        }
        if (step_id < 0)
        {
            group.clear();
            ParsedAction pa;
            if (!parse_action_points(actions_json[i], i, &pa))
            {
                return false;
            }
            group.push_back(std::move(pa));
            j = i + 1;
        }

        std::sort(group.begin(), group.end(), [](const ParsedAction &a, const ParsedAction &b) {
            if (a.duration != b.duration)
            {
                return a.duration < b.duration;
            }
            return a.index < b.index;
        });

        for (const auto &pa : group)
        {
            const int rid = pa.robot_id;
            if (rid < 0 || rid >= static_cast<int>(out->size()))
            {
                continue;
            }

            ++global_act_id;
            for (const auto &pt : pa.points)
            {
                const double t_abs = global_time + pt.t;

                RobotPose active = current_pose[static_cast<std::size_t>(rid)];
                active.joint_values = pt.joints;
                active.hand_values = pt.hands;

                for (std::size_t rj = 0; rj < out->size(); ++rj)
                {
                    RobotPose pose = current_pose[rj];
                    if (static_cast<int>(rj) == rid)
                    {
                        pose = active;
                    }
                    (*out)[rj].trajectory.push_back(pose);
                    (*out)[rj].times.push_back(t_abs);
                    (*out)[rj].act_ids.push_back(global_act_id);
                }
                current_pose[static_cast<std::size_t>(rid)] = active;
            }
            global_time = global_time + pa.duration + kEps;
        }

        i = j;
    }

    for (auto &traj : *out)
    {
        if (!traj.times.empty())
        {
            const double t0 = traj.times.front();
            for (auto &t : traj.times)
            {
                t = std::max(0.0, t - t0);
            }
            traj.cost = traj.times.back();
        }
    }

    return true;
}

}  // namespace mr_planner::skillplan
