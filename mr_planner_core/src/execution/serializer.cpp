#include "mr_planner/execution/tpg.h"
#include <fstream>
#include <google/protobuf/util/json_util.h>

namespace tpg {

bool TPG::serializeToString(std::string* output) const {
    mr_planner::TPG proto;
    toProto(&proto);
    return proto.SerializeToString(output);
}

bool TPG::parseFromString(const std::string& input) {
    mr_planner::TPG proto;
    if (!proto.ParseFromString(input)) {
        return false;
    }
    fromProto(proto);
    return true;
}

bool TPG::serializeToFile(const std::string& filename) const {
    std::string output;
    if (!serializeToString(&output)) {
        return false;
    }
    
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        return false;
    }
    
    file.write(output.data(), output.size());
    return file.good();
}

bool TPG::parseFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        return false;
    }
    
    std::string input;
    file.seekg(0, std::ios::end);
    input.resize(file.tellg());
    file.seekg(0, std::ios::beg);
    file.read(&input[0], input.size());
    
    return parseFromString(input);
}

void TPG::toProto(mr_planner::TPG* proto) const {
    proto->set_dt(dt_);
    proto->set_num_robots(num_robots_);
    
    // Convert type2Edges
    for (const auto& edge : type2Edges_) {
        auto* proto_edge = proto->add_type2_edges();
        type2EdgeToProto(edge, proto_edge);
    }
    
    // Convert start nodes
    for (const auto& node : start_nodes_) {
        auto* proto_node = proto->add_start_nodes();
        nodeToProto(node, proto_node);
    }
    
    // Convert end nodes
    for (const auto& node : end_nodes_) {
        auto* proto_node = proto->add_end_nodes();
        nodeToProto(node, proto_node);
    }
    
    // Convert numNodes
    for (int num : numNodes_) {
        proto->add_num_nodes(num);
    }
    
    // Convert solution
    for (const auto& traj : solution_) {
        auto* proto_traj = proto->add_solution();
        robotTrajectoryToProto(traj, proto_traj);
    }
    
    // Set other fields
    proto->set_pre_shortcut_flowtime(pre_shortcut_flowtime_);
    proto->set_pre_shortcut_makespan(pre_shortcut_makespan_);
    proto->set_post_shortcut_flowtime(post_shortcut_flowtime_);
    proto->set_post_shortcut_makespan(post_shortcut_makespan_);
    proto->set_t_shortcut(t_shortcut_);
    proto->set_t_init(t_init_);
    proto->set_t_simplify(t_simplify_);
    proto->set_t_shortcut_check(t_shortcut_check_);
    proto->set_num_shortcut_checks(num_shortcut_checks_);
    proto->set_num_valid_shortcuts(num_valid_shortcuts_);
    proto->set_flowtime_improv(flowtime_improv_);
    
    // Convert collision check matrix
    for (const auto& matrix_row : collisionCheckMatrix_) {
        for (const auto& matrix : matrix_row) {
            auto* proto_matrix = proto->add_collision_check_matrix();
            proto_matrix->set_rows(matrix.rows());
            proto_matrix->set_cols(matrix.cols());
            for (int i = 0; i < matrix.rows(); ++i) {
                for (int j = 0; j < matrix.cols(); ++j) {
                    proto_matrix->add_values(matrix(i, j));
                }
            }
        }
    }
}

void TPG::fromProto(const mr_planner::TPG& proto) {
    dt_ = proto.dt();
    num_robots_ = proto.num_robots();
    
    // Clear existing data
    type2Edges_.clear();
    start_nodes_.clear();
    end_nodes_.clear();
    numNodes_.clear();
    solution_.clear();
    
    // Convert type2Edges
    for (const auto& proto_edge : proto.type2_edges()) {
        type2Edges_.push_back(type2EdgeFromProto(proto_edge));
    }
    
    // Convert start nodes
    for (const auto& proto_node : proto.start_nodes()) {
        start_nodes_.push_back(nodeFromProto(proto_node));
    }
    
    // Convert end nodes
    for (const auto& proto_node : proto.end_nodes()) {
        end_nodes_.push_back(nodeFromProto(proto_node));
    }
    
    // Convert numNodes
    for (int num : proto.num_nodes()) {
        numNodes_.push_back(num);
    }
    
    // Convert solution
    for (const auto& proto_traj : proto.solution()) {
        solution_.push_back(robotTrajectoryFromProto(proto_traj));
    }
    
    // Set other fields
    pre_shortcut_flowtime_ = proto.pre_shortcut_flowtime();
    pre_shortcut_makespan_ = proto.pre_shortcut_makespan();
    post_shortcut_flowtime_ = proto.post_shortcut_flowtime();
    post_shortcut_makespan_ = proto.post_shortcut_makespan();
    t_shortcut_ = proto.t_shortcut();
    t_init_ = proto.t_init();
    t_simplify_ = proto.t_simplify();
    t_shortcut_check_ = proto.t_shortcut_check();
    num_shortcut_checks_ = proto.num_shortcut_checks();
    num_valid_shortcuts_ = proto.num_valid_shortcuts();
    flowtime_improv_ = proto.flowtime_improv();
    
    // Convert collision check matrix
    collisionCheckMatrix_.clear();
    int matrix_idx = 0;
    for (int i = 0; i < num_robots_; ++i) {
        std::vector<Eigen::MatrixXi> row;
        for (int j = 0; j < num_robots_; ++j) {
            const auto& proto_matrix = proto.collision_check_matrix(matrix_idx++);
            Eigen::MatrixXi matrix(proto_matrix.rows(), proto_matrix.cols());
            int value_idx = 0;
            for (int r = 0; r < proto_matrix.rows(); ++r) {
                for (int c = 0; c < proto_matrix.cols(); ++c) {
                    matrix(r, c) = proto_matrix.values(value_idx++);
                }
            }
            row.push_back(matrix);
        }
        collisionCheckMatrix_.push_back(row);
    }
}

// Implement the helper conversion methods
void TPG::nodeToProto(const NodePtr& node, mr_planner::Node* proto) const {
    if (!node) return;
    
    // Convert RobotPose
    auto* pose_proto = proto->mutable_pose();
    pose_proto->set_robot_id(node->pose.robot_id);
    pose_proto->set_robot_name(node->pose.robot_name);
    for (double val : node->pose.joint_values) {
        pose_proto->add_joint_values(val);
    }
    for (double val : node->pose.hand_values) {
        pose_proto->add_hand_values(val);
    }
    
    // Set other fields
    proto->set_time_step(node->timeStep);
    proto->set_robot_id(node->robotId);
    proto->set_node_id(node->nodeId);
    proto->set_act_id(node->actId);
}

NodePtr TPG::nodeFromProto(const mr_planner::Node& proto) {
    auto node = std::make_shared<Node>();
    
    // Convert RobotPose
    node->pose.robot_id = proto.pose().robot_id();
    node->pose.robot_name = proto.pose().robot_name();
    for (double val : proto.pose().joint_values()) {
        node->pose.joint_values.push_back(val);
    }
    for (double val : proto.pose().hand_values()) {
        node->pose.hand_values.push_back(val);
    }
    
    // Set other fields
    node->timeStep = proto.time_step();
    node->robotId = proto.robot_id();
    node->nodeId = proto.node_id();
    node->actId = proto.act_id();
    
    return node;
}

}