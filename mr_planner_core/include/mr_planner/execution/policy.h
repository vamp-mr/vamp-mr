// Execution policies for non-motion ADG nodes (e.g. gripper open/close) and
// hooks for failure recovery: selecting a recovery skill, finding a recovery
// goal, and planning recovery trajectories. GripperPolicy is a trivial
// always-succeed implementation for simulation/playback.
#ifndef MR_PLANNER_POLICY_H
#define MR_PLANNER_POLICY_H

#include "mr_planner/core/task.h"
#include "mr_planner/execution/tpg.h"

/// Abstract interface used by the ADG to execute policy (non-motion) nodes
/// such as gripper actions, and to select/plan failure-recovery skills.
/// Recovery hooks have throwing defaults, so subclasses only override what
/// they support; execute() and update_joint_states() are mandatory.
class Policy {
public:
    /// Recovery skills that can be attempted after an execution failure.
    enum RecoverySkillType {
       RepickOther = 0,
       PickAgain = 1,
       DropAgain = 2,
       Inspect = 3,
       SupportTop = 4,
       None = -1
    };

    Policy() = default;
    /// Execute the policy action between two ADG nodes; returns success.
    virtual bool execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type) = 0;
    
    virtual RecoverySkillType select_recover_skill_type(const tpg::NodePtr &start_node, int robot_id)
        {throw std::runtime_error("Not implemented");};

    virtual bool find_recover_goal(int robot_id, const tpg::NodePtr &start_node, tpg::NodePtr &end_node,
            std::vector<ActPtr> &skipped_acts, RecoverySkillType &skill_type) { throw std::runtime_error("Not implemented"); };

    virtual bool plan_recover(int robot_id, const tpg::NodePtr &start_node, tpg::NodePtr &recover_goal,
            const RecoverySkillType &skill_type,
            std::vector<ActPtr> &recovery_acts, std::vector<RobotTrajectory> &recovery_trajs,
            const std::vector<tpg::NodePtr> &cut_nodes = {})
            {throw std::runtime_error("Not implemented");};

    virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id) = 0;
};


/// No-op policy: reports every gripper action as successful and ignores joint
/// state updates. Suitable for simulation and visualization playback.
class GripperPolicy: public Policy {
public:
    GripperPolicy();
    virtual bool execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        Activity::Type type) override;
    virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id) override;

};

#endif // MR_PLANNER_POLICY_H
