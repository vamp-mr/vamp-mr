#pragma once

#include <jsoncpp/json/json.h>

#include <mr_planner/core/instance.h>

#include <memory>
#include <string>
#include <vector>

class ActivityGraph;

namespace mr_planner::skillplan
{

struct RobotSpec
{
    int id{0};
    std::string name;
    int dof{0};
    std::string end_effector_link;
};

struct ExportOptions
{
    std::string plan_name;
    std::string environment_name;
    std::string backend_type;
    std::string moveit_group_name;
    std::string moveit_config_pkg;
    double l1_vmax{1.0};
};

struct ActivityGraphExportOptions
{
    // Drop activities that have no motion, no scene updates, and no type2 edges.
    // The first activity for each robot is always kept to ensure every robot has at least one action.
    bool prune_stationary{true};

    // Per-joint max-abs threshold for deciding whether an activity moved.
    double motion_epsilon{1e-6};

    // Export ActivityGraph objects with prev_detach==nullptr into initial_scene.objects.
    bool include_initial_scene_objects{true};

    // Export act->obj_attached/obj_detached/collision_nodes into action.scene_updates.
    bool include_scene_updates{true};

    // If true, pad each exported action's trajectory so its last timestamp matches the start time of the next
    // activity for that robot (derived from the provided per-robot trajectory times). This preserves inter-activity
    // time gaps (e.g., removed waits) when reconstructing absolute timelines from per-action relative times.
    bool pad_to_next_activity{false};
};

Json::Value make_simple_plan(const std::vector<RobotSpec> &robots,
                             const MRTrajectory &trajectories,
                             const ExportOptions &options = ExportOptions());

// Export a per-activity AIDF-style skillplan from an ActivityGraph + segmented trajectories (act_ids).
// This is intended for LEGO planning, where the ActivityGraph also encodes attach/detach and collision overrides.
Json::Value make_activity_graph_plan(const std::vector<RobotSpec> &robots,
                                     const std::shared_ptr<ActivityGraph> &act_graph,
                                     const MRTrajectory &trajectories,
                                     const ExportOptions &options = ExportOptions(),
                                     const ActivityGraphExportOptions &export_options = ActivityGraphExportOptions());

bool write_json_to_file(const Json::Value &json, const std::string &path, std::string *error = nullptr);

bool read_json_from_file(const std::string &path, Json::Value *json, std::string *error = nullptr);

// Extract per-robot trajectories by concatenating all actions for each robot in file order.
// This is sufficient for smoke-testing and for skillplans where each robot has a single action.
bool extract_robot_trajectories(const Json::Value &plan, MRTrajectory *out, std::string *error = nullptr);

// Build a synchronized, sequential multi-robot trajectory by replaying actions[] in file order and
// holding non-active robots stationary for each action's duration.
// This is useful for TPG construction from AIDF-style (skill sequence) plans.
bool extract_synchronized_trajectory(const Json::Value &plan, MRTrajectory *out, std::string *error = nullptr);

}  // namespace mr_planner::skillplan
