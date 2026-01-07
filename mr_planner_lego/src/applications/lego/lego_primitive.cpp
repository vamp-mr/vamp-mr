#include <mr_planner/applications/lego/lego_primitive.h>
#include <algorithm>
#include <cmath>

namespace
{
using lego_manipulation::lego::ToolMode;

ToolMode fkTypeToToolMode(int fk_type)
{
    switch (fk_type)
    {
    case 1:
        return ToolMode::ToolAssemble;
    case 2:
        return ToolMode::ToolDisassemble;
    case 3:
        return ToolMode::ToolAlt;
    case 4:
        return ToolMode::ToolAltAssemble;
    default:
        return ToolMode::Tool;
    }
}
}  // namespace

LegoPrimitive::LegoPrimitive(const std::shared_ptr<lego_manipulation::lego::Lego>& lego_ptr,
                             const std::shared_ptr<PlanInstance>& instance,
                             const std::vector<std::string>& eof_names,
                             bool optimize_poses,
                             bool check_stability,
                             bool print_debug,
                             double twist_rad,
                             double handover_twist_rad,
                             bool use_gen3_camera,
                             const std::string& output_dir)
  : lego_ptr_(lego_ptr), instance_(instance), eof_names_(eof_names),
    optimize_poses_(optimize_poses), check_stability_(false), print_debug_(print_debug),
    use_gen3_camera_(use_gen3_camera), output_dir_(output_dir) {
  y_n90_ << 0, 0, -1, 0,
            0, 1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;
  y_s90_ << 0, 0, 1, 0,
            0, 1, 0, 0,
            -1, 0, 0, 0,
            0, 0, 0, 1;
  z_180_ << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

  if (check_stability) {
    log("LegoPrimitive stability checking is currently disabled (TODO: integrate python-based stability client).",
        LogLevel::WARN);
  }

  // Initialize home and special poses (replicates lego_assignment setup)
  home_q_ = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
  idle_q_ = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
  receive_q_ = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
  home_receive_q_ = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
  home_handover_q_ = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
  home_q_.col(0) << 0, 0, 0, 0, -90, 0;
  idle_q_.col(0) << 0, 0, 0, 0, -90, 0;
  home_receive_q_.col(0) << 0, 0, 0, 0, 0, 180;
  receive_q_.col(0) << 0, 0, 0, 0, 0, 180;
  home_handover_q_.col(0) << 0, 0, 0, 0, -90, 0;

  // home_q
  Eigen::Matrix4d home_T = lego_manipulation::math::FK(home_q_, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
  const bool multi_robot_standby = instance_ && instance_->getNumberOfRobots() > 2;
  // Keep the default "home" pose retracted enough to avoid blocking other robots in multi-robot (e.g., quad_gp4) setups.
  // Preserve the legacy 2-robot home pose for dual_gp4.
  const double home_x = multi_robot_standby ? 0.15 : 0.2;
  home_T.col(3) << home_x, 0, 0.4, 1; // Home X, Y, Z in base frame of the Flange
  home_T = lego_ptr_->world_base_frame() * home_T;
  home_q_ = lego_ptr_->IK(home_q_, home_T, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_ee_inv_r1(), 0, IK_status_);

  // idle_q: a retracted "parking" pose used to keep non-active robots out of the workspace.
  // For multi-robot (e.g., quad_gp4) setups this is critical for feasibility when one robot
  // needs to execute a support motion near the assembly.
  idle_q_ = home_q_;
  const bool use_idle_pose = multi_robot_standby;
  if (use_idle_pose)
  {
    bool idle_found = false;
    const std::vector<Eigen::Vector3d> idle_candidates = {
        Eigen::Vector3d(-0.30, 0.0, 0.5),
        Eigen::Vector3d(-0.25, 0.0, 0.5),
        Eigen::Vector3d(-0.20, 0.0, 0.5),
        Eigen::Vector3d(-0.15, 0.0, 0.4),
        Eigen::Vector3d(-0.10, 0.0, 0.4),
        Eigen::Vector3d(-0.05, 0.0, 0.4),
        Eigen::Vector3d(0.00, 0.0, 0.4),
        Eigen::Vector3d(0.05, 0.0, 0.4),
        Eigen::Vector3d(0.05, 0.0, 0.5),
    };
    for (const auto &p : idle_candidates)
    {
      Eigen::Matrix4d idle_T = lego_manipulation::math::FK(home_q_, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
      idle_T.col(3) << p.x(), p.y(), p.z(), 1.0;
      idle_T = lego_ptr_->world_base_frame() * idle_T;
      const Eigen::MatrixXd candidate =
          lego_ptr_->IK(home_q_, idle_T, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_ee_inv_r1(), 0, IK_status_);
      if (!IK_status_)
      {
        continue;
      }

      bool collision = false;
      if (instance_)
      {
        std::vector<RobotPose> poses;
        poses.reserve(static_cast<std::size_t>(instance_->getNumberOfRobots()));
        for (int rid = 0; rid < instance_->getNumberOfRobots(); ++rid)
        {
          RobotPose pose = instance_->initRobotPose(rid);
          for (int j = 0; j < static_cast<int>(pose.joint_values.size()) && j < candidate.rows(); ++j)
          {
            pose.joint_values[static_cast<std::size_t>(j)] = candidate(j, 0) * M_PI / 180.0;
          }
          poses.push_back(std::move(pose));
        }
        collision = instance_->checkCollision(poses, false, print_debug_);
      }

      if (collision)
      {
        continue;
      }

      idle_q_ = candidate;
      idle_found = true;
      break;
    }
    if (!idle_found)
    {
      log("Failed to compute a collision-free idle (parking) pose; falling back to home pose.", LogLevel::WARN);
      idle_q_ = home_q_;
    }
  }

  // Home to receive
  Eigen::Matrix4d home_receive_T = lego_manipulation::math::FK(home_receive_q_, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
  const double receive_y = multi_robot_standby ? 0.12 : 0.0;
  home_receive_T.col(3) << 0.375, receive_y, 0.35, 1;
  home_receive_T = lego_ptr_->world_base_frame() * home_receive_T;
  home_receive_q_ = lego_ptr_->IK(home_receive_q_, home_receive_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);

  // Receive
  Eigen::Matrix4d receive_T = lego_manipulation::math::FK(receive_q_, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
  receive_T.col(3) << 0.45, receive_y, 0.35, 1;
  receive_T = lego_ptr_->world_base_frame() * receive_T;
  receive_q_ = lego_ptr_->IK(receive_q_, receive_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);

  // Home to handover
  Eigen::Matrix4d home_handover_T = lego_manipulation::math::FK(home_handover_q_, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
  home_handover_T.col(3) << 0.2, 0, 0.5, 1;
  home_handover_T = lego_ptr_->world_base_frame() * home_handover_T;
  home_handover_q_ = lego_ptr_->IK(home_handover_q_, home_handover_T, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_ee_inv_r1(), 0, IK_status_);

  // Attack angle and offsets
  pick_offset_ = Eigen::MatrixXd::Zero(7, 1);
  pick_offset_ << -0.005, 0.005, -0.005,  // place brick offset
                  -0.005, 0.005, -0.0028, // grab brick offset
                  -0.0028; // place up offset
  if (use_gen3_camera_) { pick_offset_(5) = -0.0170; }

  // Twist pose
  twist_R_pick_ = Eigen::MatrixXd::Identity(3, 3);
  twist_R_place_ = Eigen::MatrixXd::Identity(3, 3);
  twist_R_pick_ << std::cos(twist_rad), 0, std::sin(twist_rad),
                   0, 1, 0,
                   -std::sin(twist_rad), 0, std::cos(twist_rad);
  twist_R_place_ << std::cos(-twist_rad), 0, std::sin(-twist_rad),
                    0, 1, 0,
                    -std::sin(-twist_rad), 0, std::cos(-twist_rad);
  twist_R_handover_ = Eigen::MatrixXd::Identity(3, 3);
  twist_R_handover_ << std::cos(-handover_twist_rad), 0, std::sin(-handover_twist_rad), 
                      0, 1, 0, 
                      -std::sin(-handover_twist_rad), 0, std::cos(-handover_twist_rad);
}

void LegoPrimitive::setWorldGrid(const vec3d<std::string>& world_grid) { world_grid_ = world_grid; }

bool LegoPrimitive::setCollision(const std::string& object_id, const std::string& link_name, bool allow) {
  instance_->setCollision(object_id, link_name, allow);
  if (instance_ && instance_->instanceType() == "VampInstance" && object_id != link_name) {
    // Mirror the collision toggle for VAMP so attached objects (which become robot-local attachments) can be
    // whitelisted against other robot links via the attachment override path.
    instance_->setCollision(link_name, object_id, allow);
  }
  instance_->updateScene();
  if (allow) {
    log("Allow collision between " + object_id + " and " + link_name, LogLevel::DEBUG);
  } else {
    log("Disallow collision between " + object_id + " and " + link_name, LogLevel::DEBUG);
  }
  return true;
}

bool LegoPrimitive::allowToolObjectCollision(const std::string &object_id, int robot_id, bool allow) {
  if (robot_id < 0 || robot_id >= (int)eof_names_.size()) {
    log("Invalid robot_id in allowToolObjectCollision: " + std::to_string(robot_id), LogLevel::ERROR);
    return false;
  }
  return setCollision(object_id, eof_names_[robot_id], allow);
}

void LegoPrimitive::attachObjectToRobot(const std::string &name, int robot_id, const RobotPose &pose) {
  if (!instance_) return;
  instance_->attachObjectToRobot(name, robot_id, eof_names_[robot_id], pose);
  instance_->updateScene();
  log("Attached collision object " + name + " to " + eof_names_[robot_id], LogLevel::INFO);
}

void LegoPrimitive::detachObjectFromRobot(const std::string &name, const RobotPose &pose) {
  if (!instance_) return;
  instance_->detachObjectFromRobot(name, pose);
  instance_->updateScene();
  log("Detached collision object " + name, LogLevel::INFO);
}

void LegoPrimitive::computeBottomBricks(const Json::Value &cur_graph_node,
                                        const std::string &brick_name,
                                        std::vector<std::string> &bottom_bricks) {
  bottom_bricks.clear();
  if (world_grid_.empty()) {
    log("World grid not set in computeBottomBricks", LogLevel::WARN);
    return;
  }
  int brick_x = cur_graph_node["x"].asInt();
  int brick_y = cur_graph_node["y"].asInt();
  int brick_z = cur_graph_node["z"].asInt();
  int brick_ori = cur_graph_node["ori"].asInt();
  int brick_type = cur_graph_node["brick_id"].asInt();
  lego_ptr_->get_lego_below(brick_x, brick_y, brick_z, brick_ori, brick_type, world_grid_, bottom_bricks);
  if (bottom_bricks.empty()) {
    bottom_bricks.push_back("table");
  }
}

void LegoPrimitive::computeTopBricks(const Json::Value &cur_graph_node,
                                     const std::string &brick_name,
                                     std::vector<std::string> &top_bricks) {
  top_bricks.clear();
  if (world_grid_.empty()) {
    log("World grid not set in computeTopBricks", LogLevel::WARN);
    return;
  }
  int brick_x = cur_graph_node["x"].asInt();
  int brick_y = cur_graph_node["y"].asInt();
  int brick_z = cur_graph_node["z"].asInt();
  int brick_ori = cur_graph_node["ori"].asInt();
  int brick_type = cur_graph_node["brick_id"].asInt();
  lego_ptr_->get_lego_above(brick_x, brick_y, brick_z + 2, brick_ori, brick_type, world_grid_, top_bricks);
}

void LegoPrimitive::computeTwistSideBricks(const Json::Value &cur_graph_node,
                                           const std::string &brick_name,
                                           std::vector<std::string> &side_bricks) {
  side_bricks.clear();
  if (world_grid_.empty()) {
    log("World grid not set in computeTwistSideBricks", LogLevel::WARN);
    return;
  }
  int press_side = cur_graph_node["press_side"].asInt();
  int press_offset = cur_graph_node["press_offset"].asInt();
  int brick_type = cur_graph_node["brick_id"].asInt();
  int brick_ori = cur_graph_node["ori"].asInt();
  int x = cur_graph_node.isMember("press_x") ? cur_graph_node["press_x"].asInt() : cur_graph_node["x"].asInt();
  int y = cur_graph_node.isMember("press_y") ? cur_graph_node["press_y"].asInt() : cur_graph_node["y"].asInt();
  int z = cur_graph_node["z"].asInt();
  int press_x, press_y, press_ori;
  lego_ptr_->get_press_pt(x, y, brick_type, brick_ori, press_side, press_offset, press_x, press_y, press_ori);
  lego_ptr_->get_lego_next(press_x, press_y, z, press_side, brick_ori, brick_type, brick_name, world_grid_, side_bricks);
}

void LegoPrimitive::applyPickDownCollisionSetup(const Json::Value &cur_graph_node,
                                                const std::string &brick_name,
                                                int robot_id,
                                                bool enable) {
  // Allow tool touching the target brick during pick stages
  allowToolObjectCollision(brick_name, robot_id, enable);

  // Allow tool and brick to collide with the bricks underneath
  std::vector<std::string> bottom_bricks;
  computeBottomBricks(cur_graph_node, brick_name, bottom_bricks);
  for (const auto &bb : bottom_bricks) {
    setCollision(bb, eof_names_[robot_id], enable);
    setCollision(bb, brick_name, enable);
  }
}

void LegoPrimitive::applyDropDownCollisionSetup(const Json::Value &cur_graph_node,
                                                const std::string &brick_name,
                                                int robot_id,
                                                bool enable) {
  // Allow the target brick to collide with bricks underneath during insertion
  std::vector<std::string> bottom_bricks;
  computeBottomBricks(cur_graph_node, brick_name, bottom_bricks);
  for (const auto &bb : bottom_bricks) {
    setCollision(bb, brick_name, enable);
  }

  // Allow tool to collide with twist-side neighboring bricks during twist
  std::vector<std::string> side_bricks;
  computeTwistSideBricks(cur_graph_node, brick_name, side_bricks);
  for (const auto &sb : side_bricks) {
    setCollision(sb, eof_names_[robot_id], enable);
  }
}

void LegoPrimitive::applyPressDownCollisionSetup(const Json::Value &cur_graph_node,
                                                 const std::string &brick_name,
                                                 int support_robot_id,
                                                 bool enable) {
  // During pressing support, allow the support robot tool to collide with bricks on top of the assembly region
  std::vector<std::string> top_bricks;
  computeTopBricks(cur_graph_node, brick_name, top_bricks);
  for (const auto &tb : top_bricks) {
    setCollision(tb, eof_names_[support_robot_id], enable);
  }
}

void LegoPrimitive::calculateIKforLego(const Eigen::MatrixXd& T, const Eigen::MatrixXd & home_q,
                                       int robot_id, int fk_type, bool check_collision,
                                        lego_manipulation::math::VectorJd& joint_q, bool &reachable) {
  if (!reachable) return;

  ToolMode mode = fkTypeToToolMode(fk_type);
#ifdef OLD_IK_METHOD
  joint_q = lego_manipulation::math::IK(
      home_q,
      T.block(0, 3, 3, 1),
      T.block(0, 0, 3, 3),
      lego_ptr_->robot_DH(robot_id, mode),
      lego_ptr_->robot_base(robot_id),
      0,
      10e4,
      10e-4 * 5);
  reachable &= (joint_q - home_q).norm() > 1e-6;
#else
  joint_q = lego_ptr_->IK(home_q,
                          T,
                          lego_ptr_->robot_DH(robot_id, mode),
                          lego_ptr_->robot_base(robot_id),
                          lego_ptr_->robot_base_inv(robot_id),
                          lego_ptr_->robot_tool_inv(robot_id, mode),
                          0,
                          IK_status_);

  reachable = IK_status_;
#endif

  if (check_collision && reachable) {
    RobotPose pose = instance_->initRobotPose(robot_id);
    for (int i = 0; i < 6; i++) {
      pose.joint_values[i] = joint_q(i, 0) / 180.0 * M_PI;
    }
    bool hasCollision = instance_->checkCollision({pose}, false, print_debug_);
    reachable &= !hasCollision;
  }
}

bool LegoPrimitive::validateInterpolation(const lego_manipulation::math::VectorJd &q1,
                                          const lego_manipulation::math::VectorJd &q2,
                                          int robot_id) {
  RobotPose pose1 = instance_->initRobotPose(robot_id);
  RobotPose pose2 = instance_->initRobotPose(robot_id);
  for (int i = 0; i < 6; i++) {
    pose1.joint_values[i] = q1(i, 0) / 180.0 * M_PI;
    pose2.joint_values[i] = q2(i, 0) / 180.0 * M_PI;
  }
  bool valid = instance_->connect(pose1, pose2, 0.1, print_debug_);
  return valid;
}

void LegoPrimitive::calculatePickGoalsMulti(const std::string &brick_name,
                                            int press_side,
                                            int press_offset,
                                            std::vector<bool> &reachables,
                                            std::vector<vecgoal> &robot_goals)
{
  int robot_count = std::max(1, instance_->getNumberOfRobots());
  reachables.assign(robot_count, true);
  robot_goals.assign(robot_count, vecgoal());

  Eigen::MatrixXd grab_T = Eigen::MatrixXd::Identity(4, 4);
  lego_ptr_->calc_brick_grab_pose(brick_name, true, 1, -1, -1, -1, -1, press_side, press_offset, grab_T);

  Eigen::MatrixXd offset_delta = Eigen::MatrixXd::Identity(4, 4);
  offset_delta.col(3) << pick_offset_(3), pick_offset_(4), pick_offset_(5) - std::abs(pick_offset_(5)), 1;
  Eigen::MatrixXd up_delta = Eigen::MatrixXd::Identity(4, 4);
  up_delta.col(3) << 0, 0, pick_offset_(5), 1;

  Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
  twist_T.block(0, 0, 3, 3) = twist_R_pick_;

  for (int robot_id = 0; robot_id < robot_count; ++robot_id)
  {
    bool reachable = true;
    lego_manipulation::math::VectorJd pick_tilt_up, pick_up, pick_pose, pick_twist, pick_twist_up;

    auto fk_with_tool = [&](const lego_manipulation::math::VectorJd &q, ToolMode mode)
    {
      return lego_manipulation::math::FK(q,
                                         lego_ptr_->robot_DH(robot_id, mode),
                                         lego_ptr_->robot_base(robot_id),
                                         false);
    };

    Eigen::Matrix4d offset_T = grab_T * offset_delta;
    calculateIKforLego(offset_T, home_q_, robot_id, 0, false, pick_tilt_up, reachable);

    Eigen::Matrix4d up_T = grab_T * up_delta;
    if (reachable)
    {
      calculateIKforLego(up_T, pick_tilt_up, robot_id, 0, false, pick_up, reachable);
    }

    if (reachable)
    {
      calculateIKforLego(grab_T, pick_up, robot_id, 0, false, pick_pose, reachable);
    }

    if (reachable)
    {
      Eigen::Matrix4d cart_T = fk_with_tool(pick_pose, ToolMode::ToolDisassemble);
      cart_T = cart_T * twist_T;
      calculateIKforLego(cart_T, pick_pose, robot_id, 2, false, pick_twist, reachable);
    }

    if (reachable)
    {
      Eigen::Matrix4d cart_T = fk_with_tool(pick_twist, ToolMode::ToolAssemble);
      cart_T(2, 3) = cart_T(2, 3) + 0.015;
      calculateIKforLego(cart_T, pick_twist, robot_id, 1, false, pick_twist_up, reachable);
    }

    reachables[robot_id] = reachable;
    if (reachable)
    {
      robot_goals[robot_id] = {pick_tilt_up, pick_up, pick_pose, pick_twist, pick_twist_up};
    }
    else
    {
      robot_goals[robot_id].clear();
    }
  }
}

void LegoPrimitive::calculatePickGoals(const std::string &brick_name, int press_side, int press_offset,
                                       bool &r1_reachable, bool &r2_reachable,
                                       vecgoal &r1_goals, vecgoal &r2_goals) {
  std::vector<bool> reachables;
  std::vector<vecgoal> goals;
  calculatePickGoalsMulti(brick_name, press_side, press_offset, reachables, goals);

  r1_reachable = (!reachables.empty()) ? reachables[0] : false;
  r2_reachable = (reachables.size() > 1) ? reachables[1] : false;
  if (!goals.empty())
  {
    r1_goals = goals[0];
  }
  if (goals.size() > 1)
  {
    r2_goals = goals[1];
  }
}

bool LegoPrimitive::calculateDropPoses(const std::string &brick_name, const Json::Value &cur_graph_node,
                                       int press_side, int press_offset, int attack_dir, int task_idx,
                                       int robot_id, vecgoal &drop_goal) {
  lego_manipulation::math::VectorJd r_offset_goal, r_drop_up_goal, r_drop_goal, r_drop_twist_goal, r_drop_twist_up_goal;
  bool reachable = true;

  int press_x, press_y, press_ori;
  int brick_x = cur_graph_node["x"].asInt();
  int brick_y = cur_graph_node["y"].asInt();
  int brick_z = cur_graph_node["z"].asInt();
  int brick_ori = cur_graph_node["ori"].asInt();
  int brick_type = cur_graph_node["brick_id"].asInt();
  lego_ptr_->get_press_pt(brick_x, brick_y, brick_type, brick_ori, press_side, press_offset, press_x, press_y, press_ori);

  Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);
  lego_ptr_->calc_brick_grab_pose(brick_name, 1, 0, brick_x, brick_y, brick_z, brick_ori, press_side, press_offset, cart_T);

  // offset pose
  Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
  offset_T.col(3) << pick_offset_(0), pick_offset_(1) * attack_dir, pick_offset_(2) - std::abs(-0.0028), 1;
  offset_T = cart_T * offset_T;
  calculateIKforLego(offset_T, home_q_, robot_id, 0, true, r_offset_goal, reachable);

  if (reachable) {
    // drop up
    Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
    up_T.col(3) << 0, 0, pick_offset_(2), 1;
    up_T = cart_T * up_T;
    calculateIKforLego(up_T, r_offset_goal, robot_id, 0, true, r_drop_up_goal, reachable);
  }

  // twist release path must be collision free
  std::vector<std::string> side_bricks;

  log("press_x: " + std::to_string(press_x) + ", press_y: " + std::to_string(press_y)
      + ", brick_z: " + std::to_string(brick_z) + ", press_side: " + std::to_string(press_side)
      + ", brick_ori: " + std::to_string(brick_ori) + ", brick_type: " + std::to_string(brick_type)
      + ", robot_id: " + std::to_string(robot_id), LogLevel::DEBUG);
  lego_ptr_->get_lego_next(press_x, press_y, brick_z, press_side, brick_ori, brick_type, brick_name, world_grid_, side_bricks);
  for (auto & side_brick : side_bricks) {
    log("Setting collision between " + side_brick + " and " + eof_names_[robot_id], LogLevel::DEBUG);
    instance_->setCollision(side_brick, eof_names_[robot_id], true);
  }

  if (reachable) {
    // drop pose
    calculateIKforLego(cart_T, r_drop_up_goal, robot_id, 0, true, r_drop_goal, reachable);
  }

  // drop twist
  if (reachable) {
    Eigen::Matrix4d twist_T = Eigen::MatrixXd::Identity(4, 4);
    twist_T.block(0, 0, 3, 3) << twist_R_place_;
    cart_T = lego_manipulation::math::FK(r_drop_goal,
                                         lego_ptr_->robot_DH(robot_id, ToolMode::ToolAssemble),
                                         lego_ptr_->robot_base(robot_id),
                                         false);
    cart_T = cart_T * twist_T;
    calculateIKforLego(cart_T, r_drop_goal, robot_id, 1, true, r_drop_twist_goal, reachable);
  }

  if (reachable) {
    reachable &= validateInterpolation(r_drop_goal, r_drop_twist_goal, robot_id);
  }

  // drop twist up
  if (reachable) {
    cart_T = lego_manipulation::math::FK(r_drop_twist_goal,
                                         lego_ptr_->robot_DH(robot_id, ToolMode::ToolAssemble),
                                         lego_ptr_->robot_base(robot_id),
                                         false);
    cart_T(2, 3) = cart_T(2, 3) + 0.015;
    calculateIKforLego(cart_T, r_drop_twist_goal, robot_id, 1, true, r_drop_twist_up_goal, reachable);
  }

  for (auto & side_brick : side_bricks) {
    instance_->setCollision(side_brick, eof_names_[robot_id], false);
  }

  drop_goal = {r_offset_goal, r_drop_up_goal, r_drop_goal, r_drop_twist_goal, r_drop_twist_up_goal};

  return reachable;
}

bool LegoPrimitive::calculateHandoverPoses(int robot_id, vecgoal &handover_goal) {
    int robot_count = instance_->getNumberOfRobots();
    if (robot_count < 2) {
        log("Not enough robots to compute handover poses", LogLevel::ERROR);
        return false;
    }
    int receiver_robot_id = (robot_id + 1) % robot_count;
    if (receiver_robot_id == robot_id) {
        receiver_robot_id = (robot_id + 1) % robot_count;
    }
    return calculateHandoverPoses(robot_id, receiver_robot_id, handover_goal);
}

bool LegoPrimitive::calculateHandoverPoses(int robot_id, int receiver_robot_id, vecgoal &handover_goal) {
    if (robot_id < 0 || robot_id >= instance_->getNumberOfRobots() ||
        receiver_robot_id < 0 || receiver_robot_id >= instance_->getNumberOfRobots() ||
        receiver_robot_id == robot_id) {
        log("Invalid robot ids for handover", LogLevel::ERROR);
        return false;
    }

    lego_manipulation::math::VectorJd r_transfer_up_goal, r_transfer_down_goal, r_transfer_twist_goal, r_transfer_twist_up_goal;
    Eigen::MatrixXd cart_T = lego_manipulation::math::FK(
        receive_q_,
        lego_ptr_->robot_DH(receiver_robot_id, ToolMode::ToolAlt),
        lego_ptr_->robot_base(receiver_robot_id),
        false);
    cart_T = cart_T * y_s90_ * z_180_;
    
    Eigen::Matrix4d up_T = cart_T;
    up_T(2, 3) = up_T(2, 3) + 0.015;

    bool reachable = true;

    // transfer up
    calculateIKforLego(up_T, home_handover_q_, robot_id, 0, true, r_transfer_up_goal, reachable);

    if (reachable) {
        // transfer down
        calculateIKforLego(cart_T, r_transfer_up_goal, robot_id, 0, true, r_transfer_down_goal, reachable);
    }
    if (reachable) {
        // transfer twist
        cart_T = lego_manipulation::math::FK(
            r_transfer_down_goal,
            lego_ptr_->robot_DH(robot_id, ToolMode::ToolHandoverAssemble),
            lego_ptr_->robot_base(robot_id),
            false);
        Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
        twist_T.block(0, 0, 3, 3) << twist_R_handover_;
        cart_T = cart_T * twist_T;
        calculateIKforLego(cart_T, r_transfer_down_goal, robot_id, 1, true, r_transfer_twist_goal, reachable);
    }
    if (reachable) {
        // transfer twist up
        cart_T = lego_manipulation::math::FK(
            r_transfer_twist_goal,
            lego_ptr_->robot_DH(robot_id, ToolMode::ToolHandoverAssemble),
            lego_ptr_->robot_base(robot_id),
            false);
        Eigen::Matrix4d up_T_final = cart_T;
        up_T_final(2, 3) = up_T_final(2, 3) + 0.015;
        calculateIKforLego(up_T_final, r_transfer_twist_goal, robot_id, 1, true, r_transfer_twist_up_goal, reachable);
    }
    handover_goal = {r_transfer_up_goal, r_transfer_down_goal, r_transfer_twist_goal, r_transfer_twist_up_goal};

    log("Handover poses for robot " + std::to_string(robot_id+1) + " is " + std::to_string(reachable), LogLevel::DEBUG);
    return reachable;
}

bool LegoPrimitive::calculatePlacePoses(const std::string &brick_name, const Json::Value &cur_graph_node,
                                        int press_side, int press_offset, int attack_dir, int task_idx,
                                        int robot_id, vecgoal &place_goal) {
  lego_manipulation::math::VectorJd r_place_tilt_down_pre, r_place_tilt_down, r_place_down, r_place_up, r_twist, r_twist_down;
  bool reachable = true;
  Eigen::Matrix4d cart_T = Eigen::Matrix4d::Identity(4, 4);
  int press_x, press_y, press_z, press_ori;
  press_x = cur_graph_node["press_x"].asInt();
  press_y = cur_graph_node["press_y"].asInt();
  press_z = cur_graph_node["press_z"].asInt();
  press_ori = cur_graph_node["press_ori"].asInt();
  lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z+2, press_ori, press_side, cart_T);

  // place tilt down pre
  cart_T = cart_T * y_s90_ * z_180_;
  Eigen::Matrix4d pre_T = Eigen::MatrixXd::Identity(4, 4);
  pre_T.col(3) << -(pick_offset_(6) - std::abs(pick_offset_(6))), attack_dir * (-pick_offset_(4)), pick_offset_(3) - 0.02, 1;
  pre_T = cart_T * pre_T;
  calculateIKforLego(pre_T, home_receive_q_, robot_id, 3, true, r_place_tilt_down_pre, reachable);

  if (reachable) {
    // place tilt down
    Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
    offset_T.col(3) << -(pick_offset_(6) - std::abs(pick_offset_(6))), attack_dir * (-pick_offset_(4)), pick_offset_(3), 1;
    offset_T = cart_T * offset_T;
    calculateIKforLego(offset_T, r_place_tilt_down_pre, robot_id, 3, false, r_place_tilt_down, reachable);
  }

  if (reachable) {
    // place down
    Eigen::Matrix4d down_T = Eigen::MatrixXd::Identity(4, 4);
    down_T.col(3) << -pick_offset_(6), 0, 0, 1;
    down_T = cart_T * down_T;
    calculateIKforLego(down_T, r_place_tilt_down, robot_id, 3, false, r_place_down, reachable);
  }

  // twist release path must be collision free
  std::vector<std::string> side_bricks;

  int brick_z = cur_graph_node["z"].asInt();
  int brick_ori = cur_graph_node["ori"].asInt();
  int brick_type = cur_graph_node["brick_id"].asInt();
  log("press_x: " + std::to_string(press_x) + ", press_y: " + std::to_string(press_y)
      + ", brick_z: " + std::to_string(brick_z) + ", press_side: " + std::to_string(press_side)
      + ", brick_ori: " + std::to_string(brick_ori) + ", brick_type: " + std::to_string(brick_type)
      + ", robot_id: " + std::to_string(robot_id), LogLevel::DEBUG);
  lego_ptr_->get_lego_next(press_x, press_y, brick_z, press_side, brick_ori, brick_type, brick_name, world_grid_, side_bricks);
  for (auto & side_brick : side_bricks) {
    log("Setting collision between " + side_brick + " and " + eof_names_[robot_id], LogLevel::DEBUG);
    instance_->setCollision(side_brick, eof_names_[robot_id], true);
  }

  if (reachable) {
    // place up
    calculateIKforLego(cart_T, r_place_down, robot_id, 3, true, r_place_up, reachable);
  }

  if (reachable) {
    // place twist
    Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
    twist_T.block(0, 0, 3, 3) << twist_R_pick_;

    cart_T = lego_manipulation::math::FK(
        r_place_up,
        lego_ptr_->robot_DH(robot_id, ToolMode::ToolAltAssemble),
        lego_ptr_->robot_base(robot_id),
        false);
    cart_T = cart_T * twist_T;
    calculateIKforLego(cart_T, r_place_up, robot_id, 4, true, r_twist, reachable);
  }

  for (auto & side_brick : side_bricks) {
    instance_->setCollision(side_brick, eof_names_[robot_id], false);
  }

  if (reachable) {
    // place twist down
    cart_T = lego_manipulation::math::FK(
        r_twist,
        lego_ptr_->robot_DH(robot_id, ToolMode::ToolAltAssemble),
        lego_ptr_->robot_base(robot_id),
        false);
    Eigen::Matrix4d down_T = Eigen::MatrixXd::Identity(4, 4);
    down_T << 1, 0, 0, 0.015,
              0, 1, 0, 0,
              0, 0, 1, -0.015,
              0, 0, 0, 1;
    down_T = cart_T * down_T;
    calculateIKforLego(down_T, r_twist, robot_id, 4, true, r_twist_down, reachable);
  }
  place_goal = {r_place_tilt_down_pre, r_place_tilt_down, r_place_down, r_place_up, r_twist, r_twist_down};

  return reachable;
}

bool LegoPrimitive::isPressPtInBound(const std::string &brick_name, int press_side, int press_offset) {
  return lego_ptr_->is_press_pt_in_bound(brick_name, press_side, press_offset);
}

int LegoPrimitive::checkPressSideStability(const Json::Value &cur_graph_node, int press_side, int task_idx) {
  int brick_id = cur_graph_node["brick_id"].asInt();
  int ori = cur_graph_node["ori"].asInt();
  int brick_x = cur_graph_node["x"].asInt();
  int brick_y = cur_graph_node["y"].asInt();
  int brick_z = cur_graph_node["z"].asInt();
  int h, w;
  lego_ptr_->get_brick_sizes_by_type(brick_id, h, w);

  if (brick_z <= 1) {
    return 2;
  }

  int stability = 0;
  if (ori == 0) {
    if (press_side == 1) {
      for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
        if (!world_grid_[brick_x][j][brick_z-2].empty()) { stability++; }
      }
    } else if (press_side == 2) {
      for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
        if (!world_grid_[i][brick_y+w-1][brick_z-2].empty()) { stability++; }
      }
    } else if (press_side == 3) {
      for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
        if (!world_grid_[i][brick_y][brick_z-2].empty()) { stability++; }
      }
    } else if (press_side == 4) {
      for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
        if (!world_grid_[brick_x+h-1][j][ brick_z-2].empty()) { stability++; }
      }
    }
  } else if (ori == 1) {
    std::swap(h, w);
    if (press_side == 1) {
      for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
        if (!world_grid_[i][brick_y][brick_z-2].empty()) { stability++; }
      }
    } else if (press_side == 2) {
      for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
        if (!world_grid_[brick_x][j][brick_z-2].empty()) { stability++; }
      }
    } else if (press_side == 3) {
      for (int j = brick_y+w/2-1; j < brick_y+w/2+1; j++) {
        if (!world_grid_[brick_x+h-1][j][brick_z-2].empty()) { stability++; }
      }
    } else if (press_side == 4) {
      for (int i = brick_x+h/2-1; i < brick_x+h/2+1; i++) {
        if (!world_grid_[i][brick_y+w-1][brick_z-2].empty()) { stability++; }
      }
    }
  }
  return stability;
}

bool LegoPrimitive::checkSupportNeeded(const Json::Value &cur_graph_node, int press_side, int press_offset, int task_idx) {
  (void)cur_graph_node;
  (void)press_side;
  (void)press_offset;
  (void)task_idx;
  // TODO: Integrate a ROS-free stability checker (e.g., python RPC client) for deciding when support is required.
  return false;
}

bool LegoPrimitive::findStableSupportPose(int press_side, int press_offset, const Json::Value &cur_graph_node,
                                          int robot_id, int task_idx,
                                          Eigen::MatrixXd &support_T, Eigen::MatrixXd &support_pre_T,
                                          vecgoal &r_sup_goal) {
  r_sup_goal = {Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1)};

  int brick_x = cur_graph_node["x"].asInt();
  int brick_y = cur_graph_node["y"].asInt();
  int press_pt_z = cur_graph_node["z"].asInt() + 1;
  int ori = cur_graph_node["ori"].asInt();
  int brick_id = cur_graph_node["brick_id"].asInt();

  int press_pt_x, press_pt_y, press_ori;
  lego_ptr_->get_press_pt(brick_x, brick_y, brick_id, ori, press_side, press_offset, press_pt_x, press_pt_y, press_ori);

  Eigen::MatrixXd init_q = home_q_;
#ifdef OLD_IK_METHOD
  init_q(4) = 30;
#endif

  if (!optimize_poses_) {
    if (cur_graph_node.isMember("support_x")) {
      int support_x = cur_graph_node["support_x"].asInt();
      int support_y = cur_graph_node["support_y"].asInt();
      int support_z = cur_graph_node["support_z"].asInt();
      int support_ori = cur_graph_node["support_ori"].asInt();
      int manip_type = 0;
      if (cur_graph_node.isMember("manipulate_type")) {
        manip_type = cur_graph_node["manipulate_type"].asInt();
      }

      bool reachable = true;

      if (manip_type == 0) {
        Eigen::Matrix4d sup_T = Eigen::MatrixXd::Identity(4, 4);
        lego_ptr_->support_pose(support_x, support_y, support_z, support_ori, sup_T);

        // disable collision with nearby bricks
        std::vector<std::string> side_bricks, side_low_bricks, above_bricks;
        int sup_press_side, sup_brick_ori;
        lego_ptr_->get_sup_side_ori(support_ori, sup_press_side, sup_brick_ori);
        lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid_, side_bricks);
        lego_ptr_->get_lego_next(support_x, support_y, support_z-1, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid_, side_low_bricks);
        side_bricks.insert(side_bricks.end(), side_low_bricks.begin(), side_low_bricks.end());
        lego_ptr_->get_lego_above(support_x, support_y, support_z, sup_brick_ori, 9, world_grid_, above_bricks);

        for (auto & above_brick : above_bricks) { setCollision(above_brick, eof_names_[robot_id], true); }
        for (auto & side_brick : side_bricks) { setCollision(side_brick, eof_names_[robot_id], true); }

        calculateIKforLego(sup_T, init_q, robot_id, 0, true, r_sup_goal[1], reachable);

        // reenable collision
        for (auto & above_brick : above_bricks) { setCollision(above_brick, eof_names_[robot_id], false); }
        for (auto & side_brick : side_bricks) { setCollision(side_brick, eof_names_[robot_id], false); }

        if (reachable) {
          visualize_robot_pose(r_sup_goal[1], robot_id);
          // find nearby free pose
          vec<double> dxs = {0, 0.001, -0.001};
          vec<double> dys = {0, 0.001, -0.001};
          for (double dx : dxs) {
            for (double dy : dys) {
              Eigen::Matrix4d sup_T_down = Eigen::MatrixXd::Identity(4, 4);
              sup_T_down = sup_T;
              sup_T_down(0, 3) += dx;
              sup_T_down(1, 3) += dy;
              sup_T_down(2, 3) += sup_down_offset_;
              reachable = true;
              calculateIKforLego(sup_T_down, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);
              if (reachable) {
                log("Found stable support pose for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::DEBUG);
                support_T = sup_T; support_pre_T = sup_T_down;
                return true;
              }
            }
          }
          log("No nearby pre-support pose found for  task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::WARN);
          return false;
        }
      } else {
        // press from the top as support for placing from the bottom
        Eigen::Matrix4d press_T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d press_up_T = press_T;

        // press up
        if(support_ori == 0){
          lego_ptr_->assemble_pose_from_top(support_x+1, support_y, support_z+1, 0, support_ori+1, press_T);
          press_T(0, 3) = press_T(0, 3) + 0.002;
        }
        else if(support_ori == 1){
          lego_ptr_->assemble_pose_from_top(support_x, support_y-1, support_z+1, 1, support_ori+1, press_T);
          press_T(1, 3) = press_T(1, 3) - 0.002;
        }
        else if (support_ori == 2) {
          lego_ptr_->assemble_pose_from_top(support_x, support_y+1, support_z+1, 1, support_ori+1, press_T);
          press_T(1, 3) = press_T(1, 3) + 0.002;
        }
        else if (support_ori == 3) {
          lego_ptr_->assemble_pose_from_top(support_x-1, support_y, support_z+1, 0, support_ori+1, press_T);
          press_T(0, 3) = press_T(0, 3) - 0.002;
        }
        else {
          log("Invalid support orientation " + std::to_string(support_ori), LogLevel::ERROR);
          return false;
        }
        press_T(2, 3) = press_T(2, 3) - lego_ptr_->brick_height() + lego_ptr_->lever_wall_height() + lego_ptr_->knob_height();
        press_up_T = press_T;
        press_up_T(2, 3) = press_up_T(2, 3) + 0.005;

        std::vector<std::string> side_bricks, below_bricks;
        int sup_press_side, sup_brick_ori;
        lego_ptr_->get_sup_side_ori(support_ori, sup_press_side, sup_brick_ori);
        lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid_, side_bricks);
        lego_ptr_->get_lego_below(support_x, support_y, support_z, sup_brick_ori, 9, world_grid_, below_bricks);
        for (auto & side_brick : side_bricks) { setCollision(side_brick, eof_names_[robot_id], true); }
        for (auto & below_brick : below_bricks) { setCollision(below_brick, eof_names_[robot_id], true); }
        calculateIKforLego(press_T, home_q_, robot_id, 0, true, r_sup_goal[1], reachable);
        for (auto & side_brick : side_bricks) { setCollision(side_brick, eof_names_[robot_id], false); }
        for (auto & below_brick : below_bricks) { setCollision(below_brick, eof_names_[robot_id], false); }

        if (reachable) {
          calculateIKforLego(press_up_T, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);
          if (reachable) {
            log("Found stable press support pose for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::DEBUG);
            support_T = press_T; support_pre_T = press_up_T;
            return true;
          }
        }
      }
    }
  } else {
    vec<int> dxs = {0, 1, -1, 2, -2};
    vec<int> dys = {0, 1, -1, 2, -2};
    vec<int> support_oris = {0, 1, 2, 3};
    for (int dx : dxs) {
      for (int dy : dys) {
        for (int support_ori : support_oris) {
          for (int dz = 2; dz <= 4; dz++) {
            int support_x = press_pt_x + dx;
            int support_y = press_pt_y + dy;
            int support_z = press_pt_z - 1 - dz;
            if (support_z < 0 || support_z >= (int)world_grid_[0][0].size() ||
                support_x < 0 || support_x >= (int)world_grid_.size() ||
                support_y < 0 || support_y >= (int)world_grid_[0].size()) {
              continue;
            }
            if ((!world_grid_[support_x][support_y][support_z-1].empty()) || (world_grid_[support_x][support_y][support_z].empty())) {
              continue;
            }

            // disable collision with nearby bricks
            std::vector<std::string> side_bricks, above_bricks;
            int sup_press_side, sup_brick_ori;
            lego_ptr_->get_sup_side_ori(support_ori, sup_press_side, sup_brick_ori);
            lego_ptr_->get_lego_next(support_x, support_y, support_z, sup_press_side, sup_brick_ori, 9, "support_brick", world_grid_, side_bricks);
            lego_ptr_->get_lego_above(support_x, support_y, support_z, sup_brick_ori, 9, world_grid_, above_bricks);

            for (auto & above_brick : above_bricks) { setCollision(above_brick, eof_names_[robot_id], true); }
            for (auto & side_brick : side_bricks) { setCollision(side_brick, eof_names_[robot_id], true); }

            // calculate pose
            Eigen::Matrix4d sup_T = Eigen::MatrixXd::Identity(4, 4);
            lego_ptr_->support_pose(support_x, support_y, support_z, support_ori, sup_T);
            log("Checking reachability for task: " + std::to_string(task_idx) + " at sup_x "
                + std::to_string(support_x) + " sup_y " + std::to_string(support_y) + " sup_z " + std::to_string(support_z)
                + " sup_ori " + std::to_string(support_ori), LogLevel::DEBUG);

            bool reachable = true;
            calculateIKforLego(sup_T, init_q, robot_id, 0, true, r_sup_goal[1], reachable);

            for (auto & above_brick : above_bricks) { setCollision(above_brick, eof_names_[robot_id], false); }
            for (auto & side_brick : side_bricks) { setCollision(side_brick, eof_names_[robot_id], false); }

            // check stability
            if (reachable) {
              // TODO: Add stability validation once we have a ROS-free stability checker.
              {
                log("Stability check disabled; accepting reachable support pose for task: " + std::to_string(task_idx) +
                        " at sup_x " + std::to_string(support_x) + " sup_y " + std::to_string(support_y) +
                        " sup_z " + std::to_string(support_z) + " sup_ori " + std::to_string(support_ori),
                    LogLevel::INFO);
                // find nearby free pose
                Eigen::Matrix4d sup_T_down = Eigen::MatrixXd::Identity(4, 4);
                sup_T_down = sup_T;
                sup_T_down(2, 3) += sup_down_offset_;
                calculateIKforLego(sup_T_down, r_sup_goal[1], robot_id, 0, true, r_sup_goal[0], reachable);
                if (reachable) {
                  log("Found stable support pre pose for task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::DEBUG);
                  support_T = sup_T; support_pre_T = sup_T_down;
                  return true;
                } else {
                  log("No nearby pre-support pose found for  task " + std::to_string(task_idx) + " sup robot " + std::to_string(robot_id+1), LogLevel::WARN);
                  return false;
                }
              }
            }
          }
        }
      }
    }
  }
  return false;
}

bool LegoPrimitive::findBestPlacePoses(int task_idx, int robot_id, const vec<std::string> &brick_names,
                                       const Json::Value &cur_graph_node, int manip_type,
                                       vec<int> &press_poses, vec<vecgoal> &goals) {
  vec<int> fes_press_poses;
  vec<int> stability;
  vec<vecgoal> fes_goals;

  int brick_id = cur_graph_node["brick_id"].asInt();
  int brick_width, brick_height;
  lego_ptr_->get_brick_sizes_by_type(brick_id, brick_width, brick_height);
  int attack_dir = 1;
  if (cur_graph_node.isMember("attack_dir")) {
    attack_dir = cur_graph_node["attack_dir"].asInt();
  }

  std::string brick_name;
  for (auto & name : brick_names) {
    if (name.find("b" + std::to_string(brick_id) + "_") == 0) { brick_name = name; break; }
  }

  press_poses.clear(); goals.clear();

  if (!optimize_poses_) {
    if (cur_graph_node.isMember("press_side") && cur_graph_node.isMember("press_x")) {
      int press_side = cur_graph_node["press_side"].asInt();
      int press_offset = cur_graph_node["press_offset"].asInt();

      vecgoal goal; bool reachable;
      if (manip_type == 0) {
        reachable = calculateDropPoses(brick_name, cur_graph_node, press_side, press_offset, attack_dir, task_idx, robot_id, goal);
      } else {
        reachable = calculatePlacePoses(brick_name, cur_graph_node, press_side, press_offset, attack_dir, task_idx, robot_id, goal);
      }
      if (reachable) {
        goals.push_back(goal);
        press_poses.push_back((press_side - 1) * num_offsets_ + press_offset);
        bool stable = false;
        if (cur_graph_node.isMember("support_x") && cur_graph_node["support_x"].asInt() == -1) { stable = true; }
        return stable;
      } else {
        log("Task " + std::to_string(task_idx) + " robot " + std::to_string(robot_id + 1) + " given press side "
            + std::to_string(press_side) + " offset " + std::to_string(press_offset) + " is not reachable.", LogLevel::WARN);
        return false;
      }
    } else {
      log("Task " + std::to_string(task_idx) + " robot " + std::to_string(robot_id + 1) + " does not have press side and offset", LogLevel::ERROR);
      return false;
    }
  } else {
    for (int press_side = 1; press_side <= 4; press_side++) {
      for (int press_offset = 0; press_offset < num_offsets_; press_offset ++) {
        if (!isPressPtInBound(brick_name, press_side, press_offset)) { continue; }
        int press_pt_x, press_pt_y, press_ori, height, width;
        lego_ptr_->get_press_pt(cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), brick_id,
                                cur_graph_node["ori"].asInt(), press_side, press_offset, press_pt_x, press_pt_y, press_ori);
        int press_pt_z = cur_graph_node["z"].asInt();
        log("Press pt " + std::to_string(press_pt_x) + " " + std::to_string(press_pt_y) + " " + std::to_string(press_pt_z) + " " + std::to_string(press_ori), LogLevel::DEBUG);

        if (press_pt_x < (int)world_grid_.size() && press_pt_y < (int)world_grid_[0].size() && !world_grid_[press_pt_x][press_pt_y][press_pt_z-1].empty()) { continue; }
        int press_pt_x2 = (press_ori == 0) ? press_pt_x : press_pt_x + 1;
        int press_pt_y2 = (press_ori == 0) ? press_pt_y + 1 : press_pt_y;
        if (press_pt_x2 < (int)world_grid_.size() && press_pt_y2 < (int)world_grid_[0].size() && !world_grid_[press_pt_x2][press_pt_y2][press_pt_z-1].empty()) { continue; }

        vecgoal goal;
        bool reachable = calculateDropPoses(brick_name, cur_graph_node, press_side, press_offset, attack_dir, task_idx, robot_id, goal);
        if (reachable) {
          bool sup_needed = checkSupportNeeded(cur_graph_node, press_side, press_offset, task_idx);
          int press_side_score = (sup_needed) ? 0 : 100;
          stability.push_back(press_side_score);

          fes_goals.push_back(goal);
          fes_press_poses.push_back((press_side-1) * num_offsets_ + press_offset);
        }
      }
    }

    if (fes_press_poses.empty()) {
      log("No feasible press sides found for task " + std::to_string(task_idx) + " robot " + std::to_string(robot_id + 1), LogLevel::WARN);
      return false;
    }

    int max_stability = *std::max_element(stability.begin(), stability.end());
    for (int i = 0; i < (int)stability.size(); i++) {
      if (stability[i] == max_stability) {
        press_poses.push_back(fes_press_poses[i]);
        goals.push_back(fes_goals[i]);
      }
    }

    bool stable = max_stability >= 100;
    log("robot " + std::to_string(robot_id + 1) + " task " + std::to_string(task_idx) + " have " + std::to_string(press_poses.size())
        +  (stable ? " stable" : " unstable" ) + " press sides ", LogLevel::INFO);
    return stable;
  }
}

void LegoPrimitive::visualize_robot_pose(const lego_manipulation::math::VectorJd &joint_q, int robot_id) {
    RobotPose pose = instance_->initRobotPose(robot_id);
    for (int i = 0; i < 6; i++) {
        pose.joint_values[i] = joint_q(i, 0) / 180.0 * M_PI;
    }
    instance_->moveRobot(robot_id, pose);
    instance_->updateScene();
}
