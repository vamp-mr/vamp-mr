#include <mr_planner/io/graph_proto.h>

#include <mr_planner/core/instance.h>
#include <mr_planner/execution/adg.h>
#include <mr_planner/execution/tpg.h>

#include "mr_planner_graph.pb.h"

#include <google/protobuf/message.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mr_planner::graph_proto
{
namespace
{

std::string format_error(const std::string &ctx, const std::string &msg)
{
    if (ctx.empty())
    {
        return msg;
    }
    return ctx + ": " + msg;
}

bool write_message_to_file(const google::protobuf::Message &msg, const std::string &path, std::string *error)
{
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open())
    {
        if (error)
        {
            *error = format_error(path, "failed to open for writing");
        }
        return false;
    }
    if (!msg.SerializeToOstream(&ofs))
    {
        if (error)
        {
            *error = format_error(path, "protobuf SerializeToOstream failed");
        }
        return false;
    }
    return true;
}

void fill_robot_pose(const RobotPose &pose, mr_planner::proto::RobotPose *out)
{
    if (!out)
    {
        return;
    }
    out->set_robot_id(pose.robot_id);
    out->set_robot_name(pose.robot_name);
    out->clear_joint_values();
    out->clear_hand_values();
    for (double v : pose.joint_values)
    {
        out->add_joint_values(v);
    }
    for (double v : pose.hand_values)
    {
        out->add_hand_values(v);
    }
}

void fill_object(const Object &obj, mr_planner::proto::Object *out)
{
    if (!out)
    {
        return;
    }
    out->set_name(obj.name);
    out->set_state(static_cast<std::int32_t>(obj.state));
    out->set_parent_link(obj.parent_link);
    out->set_robot_id(obj.robot_id);

    out->set_x(obj.x);
    out->set_y(obj.y);
    out->set_z(obj.z);
    out->set_qx(obj.qx);
    out->set_qy(obj.qy);
    out->set_qz(obj.qz);
    out->set_qw(obj.qw);

    out->set_x_attach(obj.x_attach);
    out->set_y_attach(obj.y_attach);
    out->set_z_attach(obj.z_attach);
    out->set_qx_attach(obj.qx_attach);
    out->set_qy_attach(obj.qy_attach);
    out->set_qz_attach(obj.qz_attach);
    out->set_qw_attach(obj.qw_attach);

    out->set_shape(static_cast<std::int32_t>(obj.shape));
    out->set_radius(obj.radius);
    out->set_length(obj.length);
    out->set_width(obj.width);
    out->set_height(obj.height);
    out->set_mesh_path(obj.mesh_path);
}

struct EdgeKey
{
    int edge_id{-1};
    int from_robot{-1};
    int from_step{-1};
    int to_robot{-1};
    int to_step{-1};

    bool operator==(const EdgeKey &o) const
    {
        return edge_id == o.edge_id && from_robot == o.from_robot && from_step == o.from_step && to_robot == o.to_robot &&
               to_step == o.to_step;
    }
};

struct EdgeKeyHash
{
    std::size_t operator()(const EdgeKey &k) const noexcept
    {
        std::size_t h = 0;
        auto mix = [&h](std::size_t x) {
            h ^= x + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        };
        mix(std::hash<int>()(k.edge_id));
        mix(std::hash<int>()(k.from_robot));
        mix(std::hash<int>()(k.from_step));
        mix(std::hash<int>()(k.to_robot));
        mix(std::hash<int>()(k.to_step));
        return h;
    }
};

struct EdgeInfo
{
    EdgeKey key;
    bool switchable{true};
    bool tight{false};
};

bool fill_tpg_graph(const tpg::TPG &tpg, mr_planner::proto::TPGGraph *out, std::string *error)
{
    if (!out)
    {
        if (error)
        {
            *error = "null output parameter";
        }
        return false;
    }
    out->Clear();
    out->set_schema("mr_planner.tpg");
    out->set_schema_version(1);

    const auto cfg = tpg.getConfig();
    const double dt = cfg.dt;
    out->set_dt(dt);

    MRTrajectory solution;
    tpg.getSolution(solution);
    const int num_robots = static_cast<int>(solution.size());
    if (num_robots <= 0)
    {
        if (error)
        {
            *error = "TPG has empty solution (uninitialized?)";
        }
        return false;
    }

    std::unordered_set<EdgeKey, EdgeKeyHash> seen_edges;
    std::vector<EdgeInfo> edges;
    std::vector<int> last_time_step(static_cast<std::size_t>(num_robots), -1);

    out->clear_robots();
    for (int rid = 0; rid < num_robots; ++rid)
    {
        auto *robot = out->add_robots();
        robot->set_robot_id(rid);

        std::string robot_name;
        if (rid < static_cast<int>(solution.size()) && !solution[rid].trajectory.empty())
        {
            robot_name = solution[rid].trajectory.front().robot_name;
        }

        tpg::NodePtr node = tpg.getStartNode(rid);
        if (!node)
        {
            if (error)
            {
                std::ostringstream oss;
                oss << "TPG start node missing for robot_id=" << rid;
                *error = oss.str();
            }
            return false;
        }

        if (robot_name.empty())
        {
            robot_name = node->pose.robot_name;
        }
        if (robot_name.empty())
        {
            robot_name = "robot_" + std::to_string(rid);
        }
        robot->set_robot_name(robot_name);

        robot->clear_nodes();
        int safety = 0;
        while (node)
        {
            auto *n = robot->add_nodes();
            n->set_time_step(node->timeStep);
            n->set_act_id(node->actId);
            n->clear_joint_values();
            for (double v : node->pose.joint_values)
            {
                n->add_joint_values(v);
            }
            n->clear_hand_values();
            for (double v : node->pose.hand_values)
            {
                n->add_hand_values(v);
            }
            last_time_step[static_cast<std::size_t>(rid)] = node->timeStep;

            for (const auto &edge : node->Type2Next)
            {
                if (!edge || !edge->nodeFrom || !edge->nodeTo)
                {
                    continue;
                }
                EdgeInfo info;
                info.key.edge_id = edge->edgeId;
                info.key.from_robot = edge->nodeFrom->robotId;
                info.key.from_step = edge->nodeFrom->timeStep;
                info.key.to_robot = edge->nodeTo->robotId;
                info.key.to_step = edge->nodeTo->timeStep;
                info.switchable = edge->switchable;
                info.tight = edge->tight;

                if (seen_edges.insert(info.key).second)
                {
                    edges.push_back(info);
                }
            }

            node = node->Type1Next;
            if (++safety > 2000000)
            {
                if (error)
                {
                    *error = "TPG Type1Next chain appears to be cyclic/too long";
                }
                return false;
            }
        }
    }

    std::sort(edges.begin(), edges.end(), [](const EdgeInfo &a, const EdgeInfo &b) {
        if (a.key.edge_id != b.key.edge_id)
        {
            return a.key.edge_id < b.key.edge_id;
        }
        if (a.key.from_robot != b.key.from_robot)
        {
            return a.key.from_robot < b.key.from_robot;
        }
        if (a.key.from_step != b.key.from_step)
        {
            return a.key.from_step < b.key.from_step;
        }
        if (a.key.to_robot != b.key.to_robot)
        {
            return a.key.to_robot < b.key.to_robot;
        }
        return a.key.to_step < b.key.to_step;
    });

    out->clear_type2_edges();
    for (const auto &edge : edges)
    {
        auto *e = out->add_type2_edges();
        e->set_edge_id(edge.key.edge_id);
        e->set_switchable(edge.switchable);
        e->set_tight(edge.tight);
        e->mutable_from()->set_robot_id(edge.key.from_robot);
        e->mutable_from()->set_time_step(edge.key.from_step);
        e->mutable_to()->set_robot_id(edge.key.to_robot);
        e->mutable_to()->set_time_step(edge.key.to_step);
    }

    double makespan = 0.0;
    double flowtime = 0.0;
    for (int rid = 0; rid < num_robots; ++rid)
    {
        double end_t = 0.0;
        const auto &traj = solution[static_cast<std::size_t>(rid)];
        if (!traj.times.empty())
        {
            end_t = traj.times.back();
        }
        else if (!traj.trajectory.empty())
        {
            end_t = (static_cast<double>(traj.trajectory.size() - 1) * dt);
        }
        else if (last_time_step[static_cast<std::size_t>(rid)] >= 0)
        {
            end_t = static_cast<double>(last_time_step[static_cast<std::size_t>(rid)]) * dt;
        }
        makespan = std::max(makespan, end_t);
        flowtime += end_t;
    }

    out->set_pre_shortcut_flowtime(flowtime);
    out->set_pre_shortcut_makespan(makespan);
    out->set_post_shortcut_flowtime(flowtime);
    out->set_post_shortcut_makespan(makespan);

    return true;
}

bool fill_adg_graph(const tpg::ADG &adg, mr_planner::proto::ADGGraph *out, std::string *error)
{
    if (!out)
    {
        if (error)
        {
            *error = "null output parameter";
        }
        return false;
    }
    out->Clear();

    if (!fill_tpg_graph(adg, out->mutable_base(), error))
    {
        return false;
    }

    const auto act_graph = adg.getActGraph();
    if (!act_graph)
    {
        if (error)
        {
            *error = "ADG missing ActivityGraph";
        }
        return false;
    }

    const int num_robots = act_graph->num_robots();
    std::vector<std::vector<std::pair<int, int>>> act_ranges(static_cast<std::size_t>(num_robots));
    for (int rid = 0; rid < num_robots; ++rid)
    {
        act_ranges[static_cast<std::size_t>(rid)].assign(static_cast<std::size_t>(act_graph->num_activities(rid)),
                                                         std::make_pair(-1, -1));
    }

    // Compute per-activity time step ranges from base robot timelines.
    for (const auto &robot : out->base().robots())
    {
        const int rid = robot.robot_id();
        if (rid < 0 || rid >= num_robots)
        {
            continue;
        }
        auto &ranges = act_ranges[static_cast<std::size_t>(rid)];
        for (const auto &node : robot.nodes())
        {
            const int act_id = node.act_id();
            if (act_id < 0 || act_id >= static_cast<int>(ranges.size()))
            {
                continue;
            }
            auto &r = ranges[static_cast<std::size_t>(act_id)];
            if (r.first < 0)
            {
                r.first = node.time_step();
                r.second = node.time_step();
            }
            else
            {
                r.first = std::min(r.first, node.time_step());
                r.second = std::max(r.second, node.time_step());
            }
        }
    }

    auto *ag = out->mutable_activity_graph();
    ag->Clear();
    ag->set_num_robots(num_robots);

    // Activities
    for (int rid = 0; rid < num_robots; ++rid)
    {
        const int num_act = act_graph->num_activities(rid);
        for (int act_id = 0; act_id < num_act; ++act_id)
        {
            ActPtr act = act_graph->get(rid, act_id);
            if (!act)
            {
                continue;
            }

            auto *a = ag->add_activities();
            a->mutable_key()->set_robot_id(rid);
            a->mutable_key()->set_act_id(act_id);

            a->set_type(static_cast<std::int32_t>(act->type));
            a->set_type_name(act->type_string());

            a->clear_type2_prev();
            for (const auto &dep : act->type2_prev)
            {
                if (!dep)
                {
                    continue;
                }
                auto *dk = a->add_type2_prev();
                dk->set_robot_id(dep->robot_id);
                dk->set_act_id(dep->act_id);
            }

            fill_robot_pose(act->start_pose, a->mutable_start_pose());
            fill_robot_pose(act->end_pose, a->mutable_end_pose());

            a->clear_obj_attached();
            for (const auto &obj : act->obj_attached)
            {
                if (obj)
                {
                    a->add_obj_attached(obj->name());
                }
            }
            a->clear_obj_detached();
            for (const auto &obj : act->obj_detached)
            {
                if (obj)
                {
                    a->add_obj_detached(obj->name());
                }
            }

            a->clear_collision_nodes();
            for (const auto &col : act->collision_nodes)
            {
                auto *cn = a->add_collision_nodes();
                cn->set_obj_name(col.obj_name);
                cn->set_link_name(col.link_name);
                cn->set_allow(col.allow);
            }

            if (rid >= 0 && rid < static_cast<int>(act_ranges.size()) && act_id >= 0 &&
                act_id < static_cast<int>(act_ranges[static_cast<std::size_t>(rid)].size()))
            {
                const auto &r = act_ranges[static_cast<std::size_t>(rid)][static_cast<std::size_t>(act_id)];
                a->set_start_time_step(r.first);
                a->set_end_time_step(r.second);
            }
        }
    }

    // Objects
    for (const auto &obj : act_graph->get_obj_nodes())
    {
        if (!obj)
        {
            continue;
        }
        auto *o = ag->add_objects();
        o->set_obj_node_id(obj->obj_node_id);
        fill_object(obj->obj, o->mutable_object());
        o->set_next_attach_link(obj->next_attach_link);
        o->set_vanish(obj->vanish);
        o->set_handover(obj->handover);
        o->set_fixed(obj->fixed);

        if (obj->prev_detach)
        {
            o->mutable_prev_detach()->set_robot_id(obj->prev_detach->robot_id);
            o->mutable_prev_detach()->set_act_id(obj->prev_detach->act_id);
        }
        if (obj->next_attach)
        {
            o->mutable_next_attach()->set_robot_id(obj->next_attach->robot_id);
            o->mutable_next_attach()->set_act_id(obj->next_attach->act_id);
        }
    }

    // Execution start activity IDs (optional; defaults to empty/0 in consumers).
    out->clear_exec_start_act();
    for (int v : adg.getExecStartActs())
    {
        out->add_exec_start_act(v);
    }

    return true;
}

}  // namespace

bool write_tpg(const tpg::TPG &tpg, const std::string &path, std::string *error)
{
    mr_planner::proto::GraphFile file;
    file.set_schema("mr_planner.graph");
    file.set_schema_version(1);

    if (!fill_tpg_graph(tpg, file.mutable_tpg(), error))
    {
        return false;
    }

    return write_message_to_file(file, path, error);
}

bool write_adg(const tpg::ADG &adg, const std::string &path, std::string *error)
{
    mr_planner::proto::GraphFile file;
    file.set_schema("mr_planner.graph");
    file.set_schema_version(1);

    if (!fill_adg_graph(adg, file.mutable_adg(), error))
    {
        return false;
    }

    return write_message_to_file(file, path, error);
}

}  // namespace mr_planner::graph_proto
