#ifndef LEGO_HPP
#define LEGO_HPP

#include "mr_planner/applications/lego/lego/Utils/Math.hpp"
#include "mr_planner/applications/lego/lego/Utils/FileIO.hpp"
#include <Eigen/StdVector>
#include <array>
#include <vector>
// #include "ros/ros.h"
// #include "gazebo_msgs/SetModelState.h"

namespace lego_manipulation
{
namespace lego
{
enum class ToolMode
{
    Flange = 0,
    Tool,
    ToolAssemble,
    ToolDisassemble,
    ToolAlt,
    ToolAltAssemble,
    ToolHandoverAssemble,
    COUNT
};

struct RobotCalibration
{
    std::string base_fname;
    std::string dh_fname;
    std::string dh_tool_fname;
    std::string dh_tool_assemble_fname;
    std::string dh_tool_disassemble_fname;
    std::string dh_tool_alt_fname;
    std::string dh_tool_alt_assemble_fname;
    std::string dh_tool_handover_assemble_fname;
};

struct lego_brick{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quaterniond cur_quat;
    std::string brick_name;
    int height;
    int width;
    double x;
    double y;
    double z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
    double cur_x;
    double cur_y;
    double cur_z;
    int press_side;
    int press_offset;
    bool fixed;
    bool in_stock;
    std::map<std::string, std::string> top_connect;
    std::map<std::string, std::string> bottom_connect;
};

struct lego_plate{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Matrix4d pose;
    int height;
    int width;
};

struct RobotKinematics
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Matrix4d base_frame = Matrix4d::Identity();
    Matrix4d base_inv = Matrix4d::Identity();
    Eigen::MatrixXd dh;
    Matrix4d ee_inv = Matrix4d::Identity();
    std::array<Eigen::MatrixXd, static_cast<std::size_t>(ToolMode::COUNT)> tool_dh;
    std::array<Matrix4d, static_cast<std::size_t>(ToolMode::COUNT)> tool_inv;
    int dof = 0;
};

class Lego
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<Lego> Ptr;
        typedef std::shared_ptr<Lego const> ConstPtr;
        using BrickMap = std::map<
            std::string,
            lego_brick,
            std::less<std::string>,
            Eigen::aligned_allocator<std::pair<const std::string, lego_brick>>>;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        lego_plate assemble_plate_;
        lego_plate storage_plate_;

        Matrix4d world_base_frame_;
        Matrix4d world_T_base_inv_;

        std::vector<RobotKinematics, Eigen::aligned_allocator<RobotKinematics>> robots_;

        BrickMap brick_map_;

        Eigen::MatrixXd thetamax_; 
        Eigen::MatrixXd thetamax_rad_;


        Json::Value lego_library_;
        Json::Value config_;
        // ros::ServiceClient client_; 
        // gazebo_msgs::SetModelState setmodelstate_;
        double brick_height_m_ = 0.0096;
        double lever_wall_height_ = 0.0032;
        double brick_len_offset_ = 0.0000;
        double P_len_ = 0.008;
        double EPS_ = 0.00001;
        double knob_height_ = 0.0017;
        double table_width_ = 0.6985;
        double table_length_ = 0.6985;
        int default_robot_dof_ = 6;

        void update_all_top_bricks(const std::string& brick_name, Matrix4dConstRef dT);
        void update(const std::string& brick_name, Matrix4dConstRef T);
        void calc_brick_loc(const lego_brick& brick, const lego_plate& plate, const int& orientation,
                            const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z, 
                            Matrix4dRef out_pose);
        bool is_top_connect(const lego_brick& b1, const lego_brick& b2);
        bool is_bottom_connect(const lego_brick& b1, const lego_brick& b2);
        bool bricks_overlap(const lego_brick& b1, const lego_brick& b2);
        void get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry);


    public:
        Lego();
        ~Lego(){}
                
        // Operations
        void setup(const std::string& env_setup_fname,
                  const std::string& lego_lib_fname,
                  const std::string &plate_calib_fname,
                  const bool& assemble,
                  const Json::Value& task_json,
                  const std::string& world_base_fname,
                  const std::vector<RobotCalibration>& robot_files);
        void set_robot_base(const std::vector<std::string>& base_fnames);
        void set_DH(const std::vector<std::string>& dh_fnames);
        void set_DH_tool(const std::vector<std::string>& dh_tool_fnames, ToolMode mode);
        void print_manipulation_property();
        void set_assemble_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
        void set_storage_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
        void set_world_base(const std::string& world_base_fname);

        void update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                           const bool& joint_rad, const std::string& brick_name, const int& mode);
        std::string get_brick_name_by_id(const int& id, const int& seq_id);
        std::string get_brick_name_by_id(const int& id, const std::string& seq_id);
        void update_brick_connection();
        void calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                  const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                  const int& orientation, const int& press_side, const int& press_offset, Eigen::MatrixXd& T);
        void calc_brick_sup_pose(int robot_id, const int &sup_x, const int &sup_y, const int &sup_z, const int &sup_ori, const double &z_offset, Eigen::MatrixXd &T);
        
        void brick_pose_in_stock(const std::string& name, const int& press_side, const int& press_offset, Matrix4dRef T);
        void support_pose_down_pre(const int& x, const int& y, const int& z, const int& ori, Matrix4dRef T);
        void support_pose_down(const int& x, const int& y, const int& z, const int& ori, Matrix4dRef T);
        void support_pose(const int& x, const int& y, const int& z, const int& ori, Matrix4dRef T);
        void assemble_pose_from_top(const int& press_x, const int& press_y, const int& press_z, const int& press_ori, const int& press_side, Matrix4dRef T);
        bool joint_in_range(const math::VectorJd& theta, const bool& is_rad);
        math::VectorJd IK(const math::VectorJd& cur_q, Matrix4dConstRef goal_T, const Eigen::MatrixXd& DH, Matrix4dConstRef T_base, Matrix4dConstRef T_base_inv,
                          Matrix4dConstRef T_tool_inv, const bool& joint_rad, bool& status);

        int brick_instock(const std::string& name) {return brick_map_[name].in_stock;};
        int robot_dof(int robot_id) const;
        int robot_dof_1() {return robot_dof(0);};
        int robot_dof_2() {return robot_dof(1);};
        Matrix4d world_base_frame() {return world_base_frame_;};
        Matrix4d world_base_inv() {return world_T_base_inv_;};
        const Eigen::MatrixXd& robot_DH(int robot_id, ToolMode mode = ToolMode::Flange) const;
        const Eigen::MatrixXd& robot_DH_r1() {return robot_DH(0);};
        const Eigen::MatrixXd& robot_DH_tool_r1() {return robot_DH(0, ToolMode::Tool);};
        const Eigen::MatrixXd& robot_DH_tool_assemble_r1() {return robot_DH(0, ToolMode::ToolAssemble);};
        const Eigen::MatrixXd& robot_DH_tool_alt_r1() {return robot_DH(0, ToolMode::ToolAlt);};
        const Eigen::MatrixXd& robot_DH_tool_alt_assemble_r1() {return robot_DH(0, ToolMode::ToolAltAssemble);};
        const Eigen::MatrixXd& robot_DH_tool_disassemble_r1() {return robot_DH(0, ToolMode::ToolDisassemble);};
        const Eigen::MatrixXd& robot_DH_tool_handover_assemble_r1() {return robot_DH(0, ToolMode::ToolHandoverAssemble);};
        const Matrix4d& robot_base_r1() {return robot_base(0);};
        const Matrix4d& robot_base_inv_r1() {return robot_base_inv(0);};
        const Matrix4d& robot_ee_inv_r1() {return robot_ee_inv(0);};
        const Matrix4d& robot_tool_inv_r1() {return robot_tool_inv(0, ToolMode::Tool);};
        const Matrix4d& robot_tool_assemble_inv_r1() {return robot_tool_inv(0, ToolMode::ToolAssemble);};
        const Matrix4d& robot_tool_alt_inv_r1() {return robot_tool_inv(0, ToolMode::ToolAlt);};
        const Matrix4d& robot_tool_alt_assemble_inv_r1() {return robot_tool_inv(0, ToolMode::ToolAltAssemble);};
        const Matrix4d& robot_tool_disassemble_inv_r1() {return robot_tool_inv(0, ToolMode::ToolDisassemble);};
        const Matrix4d& robot_tool_handover_assemble_inv_r1() {return robot_tool_inv(0, ToolMode::ToolHandoverAssemble);};
        const Eigen::MatrixXd& robot_DH_r2() {return robot_DH(1);};
        const Eigen::MatrixXd& robot_DH_tool_r2() {return robot_DH(1, ToolMode::Tool);};
        const Eigen::MatrixXd& robot_DH_tool_assemble_r2() {return robot_DH(1, ToolMode::ToolAssemble);};
        const Eigen::MatrixXd& robot_DH_tool_disassemble_r2() {return robot_DH(1, ToolMode::ToolDisassemble);};
        const Eigen::MatrixXd& robot_DH_tool_alt_r2() {return robot_DH(1, ToolMode::ToolAlt);};
        const Eigen::MatrixXd& robot_DH_tool_alt_assemble_r2() {return robot_DH(1, ToolMode::ToolAltAssemble);};
        const Eigen::MatrixXd& robot_DH_tool_handover_assemble_r2() {return robot_DH(1, ToolMode::ToolHandoverAssemble);};
        const Matrix4d& robot_base_r2() {return robot_base(1);};
        const Matrix4d& robot_base_inv_r2() {return robot_base_inv(1);};
        const Matrix4d& robot_ee_inv_r2() {return robot_ee_inv(1);};
        const Matrix4d& robot_tool_inv_r2() {return robot_tool_inv(1, ToolMode::Tool);};
        const Matrix4d& robot_tool_assemble_inv_r2() {return robot_tool_inv(1, ToolMode::ToolAssemble);};
        const Matrix4d& robot_tool_alt_inv_r2() {return robot_tool_inv(1, ToolMode::ToolAlt);};
        const Matrix4d& robot_tool_alt_assemble_inv_r2() {return robot_tool_inv(1, ToolMode::ToolAltAssemble);};
        const Matrix4d& robot_tool_disassemble_inv_r2() {return robot_tool_inv(1, ToolMode::ToolDisassemble);};
        const Matrix4d& robot_tool_handover_assemble_inv_r2() {return robot_tool_inv(1, ToolMode::ToolHandoverAssemble);};

        const Matrix4d& robot_base(int robot_id) const;
        const Matrix4d& robot_base_inv(int robot_id) const;
        const Matrix4d& robot_ee_inv(int robot_id) const;
        const Matrix4d& robot_tool_inv(int robot_id, ToolMode mode) const;

        bool robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd, const int& robot_dof);
        bool robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal, const int& robot_dof);
        Matrix4d assemble_plate_pose() {return assemble_plate_.pose;};
        Matrix4d storage_plate_pose() {return storage_plate_.pose;};
        void calc_bric_asssemble_pose(const std::string &name, const int& brick_loc_x,
                            const int& brick_loc_y, const int& brick_loc_z, const int& orientation,
                            Matrix4dRef out_pose);
        double brick_height() {return brick_height_m_;};
        double lever_wall_height() {return lever_wall_height_;};
        double knob_height() {return knob_height_;};

        std::vector<std::string> get_brick_names();
        std::vector<std::string> get_active_bricks_names();
        std::vector<std::string> get_fixed_bricks_names();
        std::vector<std::string> get_brick_names_by_type(int id);
        std::vector<std::string> get_brick_above(const std::string& brick_name);
        std::vector<std::string> get_brick_below(const std::string& brick_name);
        void get_brick_sizes_by_type(const int& id, int &height, int &width);
        void brick_dimension_from_name(const std::string& b_name, int& height, int& width);

        void get_init_brick_xyzo(const std::string& brick_name, int& x, int& y, int& z, int &ori);
        Pose get_init_brick_pose(const std::string& brick_name);
        Pose get_curr_brick_pose(const std::string& brick_name);
        Pose get_table_pose();
        void get_brick_sizes(const std::string& brick_name, double& x, double& y, double& z);
        void get_table_size(double& x, double& y, double& z);
        bool is_press_pt_in_bound(const std::string& brick_name, int press_side, int press_offset);
        std::vector<std::vector<std::vector<std::string>>> gen_world_grid_from_graph(const Json::Value& task_json, int task_idx, int wx, int wy, int wz);
        
        void get_press_pt(int brick_x, int brick_y, int brick_type, int brick_ori, int press_side, int press_offset,
                    int &press_pt_x, int &press_pt_y, int &press_ori);
        void get_sup_side_ori(int support_ori, int &sup_press_side, int &sup_brick_ori);
        void get_lego_twist_next(const Json::Value &task_json, int task_idx, const std::string &brick_name, std::vector<std::string> &side_bricks);
        void get_lego_next(int press_x, int press_y, int brick_z, int press_side, int brick_ori, int brick_type, const std::string &brick_name,
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &side_bricks);
        void get_lego_around(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, const std::string &brick_name,
            const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &side_bricks);
        void get_lego_below(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, 
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &below_bricks);
        void get_lego_above(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, 
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &above_bricks);   
        void lego_pose_from_press_pose(const math::VectorJd& theta, int robot_id, int brick_id, int press_side, int press_offset, 
            Matrix4dRef brick_loc);
};
}
}

#endif // LEGO_HPP
