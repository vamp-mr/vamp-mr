#ifndef LEGO_PRIMITIVE_H
#define LEGO_PRIMITIVE_H

#include "mr_planner/core/instance.h"
#include "mr_planner/core/logger.h"
#include "mr_planner/applications/lego/lego/Lego.hpp"

template<typename T>
using vec = std::vector<T>;

template<typename T>
using vec2d = std::vector<std::vector<T>>;

template<typename T>
using vec3d = std::vector<std::vector<std::vector<T>>>;

using vecgoal = std::vector<lego_manipulation::math::VectorJd>;

class LegoPrimitive {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegoPrimitive(const std::shared_ptr<lego_manipulation::lego::Lego>& lego_ptr,
                const std::shared_ptr<PlanInstance>& instance,
                const std::vector<std::string>& eof_names,
                bool optimize_poses,
                bool check_stability,
                bool print_debug,
                double twist_rad,
                double handover_twist_rad,
                bool use_gen3_camera,
                const std::string& output_dir);

  // Dynamic input that can change per task iteration
  void setWorldGrid(const vec3d<std::string>& world_grid);

  // Getters for values initialized within the primitive
  const Eigen::MatrixXd& getHomeQ() const { return home_q_; }
  const Eigen::MatrixXd& getIdleQ() const { return idle_q_; }
  const Eigen::MatrixXd& getHomeReceiveQ() const { return home_receive_q_; }
  const Eigen::MatrixXd& getReceiveQ() const { return receive_q_; }
  const Eigen::MatrixXd& getHomeHandoverQ() const { return home_handover_q_; }
  const Eigen::MatrixXd& getPickOffset() const { return pick_offset_; }
  const Eigen::MatrixXd& getTwistRPick() const { return twist_R_pick_; }
  const Eigen::MatrixXd& getTwistRPlace() const { return twist_R_place_; }
  int getNumOffsets() const { return num_offsets_; }
  void setSupDownOffset(double offset) { sup_down_offset_ = offset; }
  double getSupDownOffset() const { return sup_down_offset_; }

  // Public reusable primitives
  void calculatePickGoalsMulti(const std::string &brick_name, int press_side, int press_offset,
                               std::vector<bool> &reachables, std::vector<vecgoal> &robot_goals);
  void calculatePickGoals(const std::string &brick_name, int press_side, int press_offset,
                          bool &r1_reachable, bool &r2_reachable,
                          vecgoal &r1_goals, vecgoal &r2_goals);

  bool findBestPlacePoses(int task_idx, int robot_id, const vec<std::string> &brick_names,
                          const Json::Value &cur_graph_node, int manip_type,
                          vec<int> &press_poses, vec<vecgoal> &goals);

  bool findStableSupportPose(int press_side, int press_offset, const Json::Value &cur_graph_node,
                             int robot_id, int task_idx,
                             Eigen::MatrixXd &support_T, Eigen::MatrixXd &support_pre_T,
                          vecgoal &r_sup_goal);

  bool calculateHandoverPoses(int robot_id, vecgoal &handover_goal);
  bool calculateHandoverPoses(int robot_id, int receiver_robot_id, vecgoal &handover_goal);

  bool isPressPtInBound(const std::string &brick_name, int press_side, int press_offset);

  // Collision and scene helpers for reuse in replanning
  // - Simple toggle between a target object and the active robot end-effector
  bool allowToolObjectCollision(const std::string &object_id, int robot_id, bool allow);

  // - Attach/detach an object to/from a robot end-effector
  void attachObjectToRobot(const std::string &name, int robot_id, const RobotPose &pose);
  void detachObjectFromRobot(const std::string &name, const RobotPose &pose);

  // - Compute neighboring bricks used in collision setup
  //   Requires that world grid is provided via setWorldGrid()
  void computeBottomBricks(const Json::Value &cur_graph_node,
                           const std::string &brick_name,
                           std::vector<std::string> &bottom_bricks);

  void computeTopBricks(const Json::Value &cur_graph_node,
                        const std::string &brick_name,
                        std::vector<std::string> &top_bricks);

  void computeTwistSideBricks(const Json::Value &cur_graph_node,
                              const std::string &brick_name,
                              std::vector<std::string> &side_bricks);

  // - Convenience methods to mirror task-phase collision setups from lego_node
  void applyPickDownCollisionSetup(const Json::Value &cur_graph_node,
                                   const std::string &brick_name,
                                   int robot_id,
                                   bool enable);

  void applyDropDownCollisionSetup(const Json::Value &cur_graph_node,
                                   const std::string &brick_name,
                                   int robot_id,
                                   bool enable);

  void applyPressDownCollisionSetup(const Json::Value &cur_graph_node,
                                    const std::string &brick_name,
                                    int support_robot_id,
                                    bool enable);

private:
  // Helpers used by the primitives
  void calculateIKforLego(const Eigen::MatrixXd& T, const Eigen::MatrixXd & home_q,
                          int robot_id, int fk_type, bool check_collision,
                          lego_manipulation::math::VectorJd& joint_q, bool &reachable);

  bool validateInterpolation(const lego_manipulation::math::VectorJd &q1,
                             const lego_manipulation::math::VectorJd &q2,
                             int robot_id);

  bool calculateDropPoses(const std::string &brick_name, const Json::Value &cur_graph_node,
                           int press_side, int press_offset, int attack_dir, int task_idx,
                           int robot_id, vecgoal &drop_goal);

  bool calculatePlacePoses(const std::string &brick_name, const Json::Value &cur_graph_node,
                            int press_side, int press_offset, int attack_dir, int task_idx,
                            int robot_id, vecgoal &place_goal);

  int checkPressSideStability(const Json::Value &cur_graph_node, int press_side, int task_idx);

  bool checkSupportNeeded(const Json::Value &cur_graph_node, int press_side, int press_offset, int task_idx);

  bool setCollision(const std::string& object_id, const std::string& link_name, bool allow);

  void visualize_robot_pose(const lego_manipulation::math::VectorJd &joint_q, int robot_id);

private:
  std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
  std::shared_ptr<PlanInstance> instance_;
  std::vector<std::string> eof_names_;

  std::string output_dir_;
  bool optimize_poses_ = false;
  bool check_stability_ = false;
  bool print_debug_ = false;
  bool use_gen3_camera_ = false;

  // Shared state/config
  Eigen::MatrixXd home_q_, idle_q_, home_receive_q_, receive_q_, home_handover_q_;
  Eigen::MatrixXd pick_offset_;
  Eigen::MatrixXd twist_R_pick_, twist_R_place_, twist_R_handover_;
  Eigen::Matrix4d y_n90_, y_s90_, z_180_;
  vec3d<std::string> world_grid_;
  bool IK_status_ = false;
  double sup_down_offset_ = -0.004;
  int num_offsets_ = 7;
};

#endif // LEGO_PRIMITIVE_H
