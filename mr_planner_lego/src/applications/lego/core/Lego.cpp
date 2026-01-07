#include "mr_planner/applications/lego/lego/Lego.hpp"
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace lego_manipulation
{
namespace math
{

template Eigen::Matrix<float, Eigen::Dynamic, 1> ToEigen<float>(std::vector<float> data);
template Eigen::Matrix<double, Eigen::Dynamic, 1> ToEigen<double>(std::vector<double> data);

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data)
{
    Eigen::Matrix<T, Eigen::Dynamic, 1> vec;
    vec.resize(data.size(), 1);
    for (size_t i = 0; i < data.size(); i++)
    {
        vec(i) = data.at(i);
    }
    return vec;
}
/* -------------------------------------------------------------------------- */
/*                                   Matrix                                   */
/* -------------------------------------------------------------------------- */
Eigen::MatrixXd PInv(const Eigen::MatrixXd& M)
{
    auto nrow = M.rows();
    auto ncol = M.cols();
    Eigen::MatrixXd Minv;

    if (nrow > ncol)
    {
        Minv = ( ( M.transpose() * M ).inverse() ) * M.transpose();
    }
    else if (nrow < ncol)
    {
        Minv = M.transpose() * ( ( M * M.transpose() ).inverse() );
    }
    else
    {
        Minv = M.inverse().eval();
    }

    return Minv;
}

Eigen::MatrixXd EigenVcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
{
    /** 
     * [mat1; mat2]
     */

    std::cout << "start vcat" << std::endl;
    try
    {
        if (mat1.rows() == 0){
            return mat2;
        }
        else if (mat2.rows() == 0){
            return mat1;
        }
        else{
            if (mat1.cols() != mat2.cols())
            {
                std::ostringstream ss;
                ss << "Expected mat1 and mat2 to have same cols(), got ["
                    << mat1.cols() << "], [" << mat2.cols() << "]\n";
                throw std::runtime_error(ss.str());
            }
            Eigen::MatrixXd new_mat(mat1.rows()+mat2.rows(), mat1.cols());
            new_mat.topRows(mat1.rows()) = mat1;
            new_mat.bottomRows(mat2.rows()) = mat2;
            std::cout << "finished vcat" << std::endl;
            return new_mat;
        } 
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

Eigen::MatrixXd EigenHcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
{
    /** 
     * [mat1 mat2]
     */

    try
    {
        if (mat1.cols() == 0){
            return mat2;
        }
        else if (mat2.cols() == 0){
            return mat1;
        }
        else{
            if (mat1.rows() != mat2.rows())
            {
                std::ostringstream ss;
                ss << "Expected mat1 and mat2 to have same rows(), got ["
                    << mat1.rows() << "], [" << mat2.rows() << "]\n";
                throw std::runtime_error(ss.str());
            }
            Eigen::MatrixXd new_mat(mat1.rows(), mat1.cols()+mat2.cols());
            new_mat.leftCols(mat1.cols()) = mat1;
            new_mat.rightCols(mat2.cols()) = mat2;
            return new_mat;
        } 
    }
    catch(const std::exception& e)
    {
        throw;
    }
}


Eigen::MatrixXd FK(const VectorJd& q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad)
{
    Eigen::MatrixXd R(3, 3);
    Eigen::MatrixXd T(3, 1);
    Eigen::MatrixXd trans_mtx = Eigen::MatrixXd::Identity(4, 4);
    trans_mtx = base_frame;
    Eigen::MatrixXd tmp(4, 4);
    Eigen::MatrixXd DH_cur = DH;
    VectorJd q_rad = q;

    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<q.rows(); i++)
        {
            q_rad(i) = q(i) * M_PI / 180;
        }
    }

    DH_cur.col(0) = DH.col(0) + q_rad;
    for(int i=0; i<DH_cur.rows(); i++)
    {
        R << cos(DH_cur.coeff(i, 0)), -sin(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)),  sin(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             sin(DH_cur.coeff(i, 0)),  cos(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)), -cos(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             0,                        sin(DH_cur.coeff(i, 3)),                            cos(DH_cur.coeff(i, 3));

        T << DH_cur.coeff(i, 2) * cos(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 2) * sin(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 1);
        tmp << R, T, 0, 0, 0, 1;
        trans_mtx = (trans_mtx * tmp);
    }
    return trans_mtx;
}


bool ApproxEqNum(const double& a, const double& b, const double& thres){
    return (bool) (abs(a-b) < thres);
}

}

namespace io 
{
Eigen::MatrixXd LoadMatFromFile(const std::string fname)
{
    try
    {
        std::cout << "start loading mat from file: " << fname << std::endl;
        std::ifstream file(fname);
        if (!file.is_open())
        {
            std::ostringstream ss;
            ss << "Cannot open file " << fname;
            throw std::runtime_error(ss.str());
        }
        std::cout << "File opened successfully" << std::endl;

        std::vector<double> values;
        std::size_t rows = 0;
        std::size_t cols = 0;
        std::string line;
        while (std::getline(file, line))
        {
            std::vector<double> row;
            std::string token;
            auto flush_token = [&](std::string& word)
            {
                if (word.empty())
                {
                    return;
                }
                row.push_back(std::stod(word));
                word.clear();
            };
            for (char ch : line)
            {
                if (ch == ',' || std::isspace(static_cast<unsigned char>(ch)))
                {
                    flush_token(token);
                }
                else
                {
                    token.push_back(ch);
                }
            }
            flush_token(token);

            if (row.empty())
            {
                continue;
            }

            if (cols == 0)
            {
                cols = row.size();
            }
            else if (row.size() != cols)
            {
                std::ostringstream ss;
                ss << "Inconsistent column count while parsing " << fname
                   << ": expected " << cols << " values but got " << row.size();
                throw std::runtime_error(ss.str());
            }

            values.insert(values.end(), row.begin(), row.end());
            ++rows;
        }

        file.close();

        if (rows == 0 || cols == 0)
        {
            std::cout << "Loaded empty mat from [" << fname << "]" << std::endl;
            return Eigen::MatrixXd(0, 0);
        }

        if (values.size() != rows * cols)
        {
            std::ostringstream ss;
            ss << "Unexpected parsed value count " << values.size()
               << " for shape [" << rows << ", " << cols << "] in file " << fname;
            throw std::runtime_error(ss.str());
        }

        // Fill a row-major buffer first so Eigen gets contiguous storage before the final copy.
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_major_mat(
            static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
        std::copy(values.begin(), values.end(), row_major_mat.data());
        Eigen::MatrixXd mat = row_major_mat;

        std::cout << "Loaded mat of shape [" << mat.rows()
                    << ", " << mat.cols() << "] from [" << fname << "]" << std::endl;

        return mat;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void SaveMatToFile(const Eigen::MatrixXd& mat, const std::string& fname)
{
    try
    {
        std::ofstream file(fname);
        if (!file.is_open())
        {
            std::ostringstream ss;
            ss << "Cannot write to file " << fname;
            throw std::runtime_error(ss.str());
        }
        Eigen::IOFormat fmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "\n", "", "", "", "");
        file << mat.format(fmt);
        file.close();

        std::cout << "Wrote mat of shape [" << mat.rows()
                    << ", " << mat.cols() << "] to [" << fname << "]\n";
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}

namespace lego
{
namespace
{
Matrix4d make_tool_frame_from_dh(const Eigen::MatrixXd& dh)
{
    if (dh.rows() <= 5)
    {
        throw std::runtime_error("DH matrix must have at least 6 rows to compute tool frame.");
    }
    Matrix4d tool = Matrix4d::Identity(4, 4);
    tool(0, 3) = dh(5, 2);
    tool(1, 3) = 0.0;
    tool(2, 3) = -dh(5, 1);
    return tool;
}
}

Lego::Lego()
{
}
        
void Lego::setup(const std::string& env_setup_fname,
                 const std::string& lego_lib_fname,
                 const std::string &plate_calib_fname,
                 const bool& assemble,
                 const Json::Value& task_json,
                 const std::string& world_base_fname,
                 const std::vector<RobotCalibration>& robot_files)
{
    if (robot_files.empty())
    {
        throw std::runtime_error("Lego::setup requires at least one robot calibration entry");
    }

    std::ifstream config_file(env_setup_fname, std::ifstream::binary);
    double x, y, z, roll, pitch, yaw;
    Quaterniond quat(Matrix3d::Identity(3, 3));
    Matrix4d brick_pose_mtx;
    Matrix3d z_90;
    z_90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Matrix3d rot_mtx = Matrix3d::Identity(3, 3);
    std::string brick_name;
    std::ifstream lego_lib_file(lego_lib_fname, std::ifstream::binary);
    std::ifstream plate_calib_file(plate_calib_fname, std::ifstream::binary);

    thetamax_.resize(default_robot_dof_, 2);
    thetamax_rad_.resize(default_robot_dof_, 2);
    thetamax_ << -170, 170,
                 -110, 130,
                 -65, 200,
                 -200, 200,
                 -120, 120,
                 -450, 450;
    for(int i=0; i<default_robot_dof_; i++)
    {
        thetamax_rad_.row(i) << thetamax_(i, 0) / 180 * M_PI, thetamax_(i, 1) / 180 * M_PI;
    }
    set_world_base(world_base_fname);
    robots_.clear();
    robots_.resize(robot_files.size());

    auto gather_paths = [&](auto selector)
    {
        std::vector<std::string> paths;
        paths.reserve(robot_files.size());
        for (const auto& robot_file : robot_files)
        {
            paths.push_back(selector(robot_file));
        }
        return paths;
    };

    set_robot_base(gather_paths([](const RobotCalibration& rc) { return rc.base_fname; }));
    set_DH(gather_paths([](const RobotCalibration& rc) { return rc.dh_fname; }));
    set_DH_tool(gather_paths([](const RobotCalibration& rc) { return rc.dh_tool_fname; }), ToolMode::Tool);
    set_DH_tool(gather_paths([](const RobotCalibration& rc) { return rc.dh_tool_assemble_fname; }), ToolMode::ToolAssemble);
    set_DH_tool(gather_paths([](const RobotCalibration& rc) { return rc.dh_tool_disassemble_fname; }), ToolMode::ToolDisassemble);
    set_DH_tool(gather_paths([](const RobotCalibration& rc) { return rc.dh_tool_alt_fname; }), ToolMode::ToolAlt);
    set_DH_tool(gather_paths([](const RobotCalibration& rc) { return rc.dh_tool_alt_assemble_fname; }), ToolMode::ToolAltAssemble);
    set_DH_tool(gather_paths([](const RobotCalibration& rc) { return rc.dh_tool_handover_assemble_fname; }), ToolMode::ToolHandoverAssemble);

    if (!robots_.empty())
    {
        default_robot_dof_ = robots_.front().dof;
    }
    print_manipulation_property();
    config_file >> config_;
    lego_lib_file >> lego_library_;
    Json::Value plate_calib;
    plate_calib_file >> plate_calib;

    for(auto brick = plate_calib.begin(); brick != plate_calib.end(); brick++)
    {
        //brick_pose.model_name = brick.name();
        if(brick.name().compare("storage_plate") == 0)
        {
            x = (*brick)["x"].asDouble();
            y = (*brick)["y"].asDouble();
            z = (*brick)["z"].asDouble();
            roll = (*brick)["roll"].asDouble();
            pitch = (*brick)["pitch"].asDouble();
            yaw = (*brick)["yaw"].asDouble();
            Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
            Matrix3d rot = yawAngle.toRotationMatrix();
            rot *= pitchAngle.toRotationMatrix();
            rot *= rollAngle.toRotationMatrix();
            quat = Quaterniond(rot);
            storage_plate_.pose = Matrix4d::Identity(4, 4);
            storage_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
            storage_plate_.pose.col(3) << x, y, z, 1;
            storage_plate_.width = (*brick)["width"].asInt();
            storage_plate_.height = (*brick)["height"].asInt();
            storage_plate_.pose = (world_base_frame_ * storage_plate_.pose).eval();
            x = storage_plate_.pose(0, 3);
            y = storage_plate_.pose(1, 3);
            z = storage_plate_.pose(2, 3);
            Matrix3d rot_mtx = storage_plate_.pose.block(0, 0, 3, 3);
            quat = rot_mtx;
        }
        else if(brick.name().compare("assemble_plate") == 0)
        {
            x = (*brick)["x"].asDouble();
            y = (*brick)["y"].asDouble();
            z = (*brick)["z"].asDouble();
            roll = (*brick)["roll"].asDouble();
            pitch = (*brick)["pitch"].asDouble();
            yaw = (*brick)["yaw"].asDouble();
            Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
            Matrix3d rot = yawAngle.toRotationMatrix();
            rot *= pitchAngle.toRotationMatrix();
            rot *= rollAngle.toRotationMatrix();
            quat = Quaterniond(rot);
            assemble_plate_.pose = Matrix4d::Identity(4, 4);
            assemble_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
            assemble_plate_.pose.col(3) << x, y, z, 1;
            assemble_plate_.width = (*brick)["width"].asInt();
            assemble_plate_.height = (*brick)["height"].asInt();
            assemble_plate_.pose = (world_base_frame_ * assemble_plate_.pose).eval();
            x = assemble_plate_.pose(0, 3);
            y = assemble_plate_.pose(1, 3);
            z = assemble_plate_.pose(2, 3);
            Matrix3d rot_mtx = assemble_plate_.pose.block(0, 0, 3, 3);
            quat = rot_mtx;
        }
        else{
            continue;
        }
        // brick_pose.pose.position.x = x;
        // brick_pose.pose.position.y = y;
        // brick_pose.pose.position.z = z;
        // brick_pose.pose.orientation.x = quat.x();
        // brick_pose.pose.orientation.y = quat.y();
        // brick_pose.pose.orientation.z = quat.z();
        // brick_pose.pose.orientation.w = quat.w();
        // if (brick.name().compare("storage_plate") == 0 || brick.name().compare("assemble_plate") == 0) {
        //     brick_pose.pose.position.z = z + 0.0016;
        // }
        //setmodelstate_.request.model_state = brick_pose;
        //client_.call(setmodelstate_);
    }

    for(auto brick = config_.begin(); brick != config_.end(); brick++)
    {
        //brick_pose.model_name = brick.name();
        if(brick.name()[0] == 'b')
        {
            lego_brick l_brick;
            l_brick.brick_name = brick.name();
            brick_dimension_from_name(brick.name(), l_brick.height, l_brick.width);
            calc_brick_loc(l_brick, storage_plate_, (*brick)["ori"].asInt(),
                           (*brick)["x"].asInt(), (*brick)["y"].asInt(), (*brick)["z"].asInt(),
                           brick_pose_mtx);
            x = brick_pose_mtx(0, 3);
            y = brick_pose_mtx(1, 3);
            z = brick_pose_mtx(2, 3);

            l_brick.x = x;
            l_brick.y = y;
            l_brick.z = z;
            l_brick.cur_x = x;
            l_brick.cur_y = y;
            l_brick.cur_z = z;
            l_brick.in_stock = true;
            rot_mtx = brick_pose_mtx.block(0, 0, 3, 3);
            quat = rot_mtx;
            l_brick.quat_x = quat.x();
            l_brick.quat_y = quat.y();
            l_brick.quat_z = quat.z();
            l_brick.quat_w = quat.w();
            l_brick.cur_quat = quat;
            if (brick->isMember("fixed")) {
                l_brick.fixed = (*brick)["fixed"].asBool();
            }
            else {
                l_brick.fixed = false;
            }
            brick_map_[brick.name()] = l_brick;
        }
        else
        {
            std::cout << "Unknown brick type: " << brick.name() << " !" << std::endl;
            continue;
        }
        // brick_pose.pose.position.x = x;
        // brick_pose.pose.position.y = y;
        // brick_pose.pose.position.z = z;
        // brick_pose.pose.orientation.x = quat.x();
        // brick_pose.pose.orientation.y = quat.y();
        // brick_pose.pose.orientation.z = quat.z();
        // brick_pose.pose.orientation.w = quat.w();
        // setmodelstate_.request.model_state = brick_pose;
        // client_.call(setmodelstate_);
    }

    for(int i=1; i<=task_json.size(); i++)
    {
        auto cur_graph_node = task_json[std::to_string(i)];
        // check if brick_seq is in the cur_graph_node
        if (cur_graph_node.isMember("brick_seq")) {
            std::string brick_seq = cur_graph_node["brick_seq"].asString();
            bool from_station = false;
            if (brick_seq.find(".") != std::string::npos) {
                brick_seq = brick_seq.substr(0, brick_seq.find("."));
                from_station = true;
            }
            brick_name = get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), brick_seq);
            calc_brick_loc(brick_map_[brick_name], assemble_plate_, cur_graph_node["ori"].asInt(),
                            cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), cur_graph_node["z"].asInt(),
                            brick_pose_mtx);
            x = brick_pose_mtx(0, 3);
            y = brick_pose_mtx(1, 3);
            z = brick_pose_mtx(2, 3);
            rot_mtx = brick_pose_mtx.block(0, 0, 3, 3);
            brick_map_[brick_name].in_stock = assemble;
            brick_map_[brick_name].press_side = cur_graph_node["press_side"].asInt();
            if (cur_graph_node.isMember("press_offset")) {
                brick_map_[brick_name].press_offset = cur_graph_node["press_offset"].asInt();
            }
            quat = rot_mtx;

            if (from_station) {
                lego_brick brick_task = brick_map_[brick_name];
                // add a copy of the brick with new name to the brick map
                std::string new_brick_name = "b" + std::to_string(cur_graph_node["brick_id"].asInt()) + "_" + cur_graph_node["brick_seq"].asString();

                brick_task.brick_name = new_brick_name;
                brick_map_[new_brick_name] = brick_task;
            }

            if(!assemble)
            {
                brick_map_[brick_name].cur_x = x;
                brick_map_[brick_name].cur_y = y;
                brick_map_[brick_name].cur_z = z;
                brick_map_[brick_name].cur_quat = quat;

                // brick_pose.model_name = brick_name;
                // brick_pose.pose.position.x = x;
                // brick_pose.pose.position.y = y;
                // brick_pose.pose.position.z = z;
                // brick_pose.pose.orientation.x = quat.x();
                // brick_pose.pose.orientation.y = quat.y();
                // brick_pose.pose.orientation.z = quat.z();
                // brick_pose.pose.orientation.w = quat.w();
                // setmodelstate_.request.model_state = brick_pose;
                // client_.call(setmodelstate_);
            }
        }
    }
    update_brick_connection();
    usleep(1000 * 1000); 
}

void Lego::brick_dimension_from_name(const std::string& b_name, int& height, int& width)
{
    auto dash_id = b_name.find("_");
    std::string id = b_name.substr(1, dash_id - 1);
    height = lego_library_[id]["height"].asInt();
    width = lego_library_[id]["width"].asInt();
}

void Lego::set_world_base(const std::string& world_base_fname)
{
    std::cout << "Load World Base from: " << world_base_fname << std::endl;
    auto base = io::LoadMatFromFile(world_base_fname);
    std::cout << "Returned from LoadMatFromFile\n in set world base" << std::endl;
    if (base.rows() !=4 || base.cols() !=4)
    {
        throw std::runtime_error("World base frame should be a 4x4 matrix!");
    }
    world_base_frame_ = base;
    auto nrow = world_base_frame_.rows();
    auto ncol = world_base_frame_.cols();
    Eigen::MatrixXd Minv;

    if (nrow > ncol)
    {
        Minv = ( ( world_base_frame_.transpose() * world_base_frame_ ).inverse() ) * world_base_frame_.transpose();
    }
    else if (nrow < ncol)
    {
        Minv = world_base_frame_.transpose() * ( ( world_base_frame_ * world_base_frame_.transpose() ).inverse() );
    }
    else
    {
        Minv = world_base_frame_.inverse().eval();
    }
    world_T_base_inv_ = Minv;

}

void Lego::set_robot_base(const std::vector<std::string>& base_fnames)
{
    if (base_fnames.size() != robots_.size())
    {
        throw std::runtime_error("Base filename count does not match number of robots");
    }
    for (std::size_t idx = 0; idx < base_fnames.size(); ++idx)
    {
        std::cout << "Load Robot " << idx + 1 << " Base from: " << base_fnames[idx] << std::endl;
        auto base = io::LoadMatFromFile(base_fnames[idx]);
        if (base.rows() != 4 || base.cols() != 4)
        {
            throw std::runtime_error("Robot base frame should be a 4x4 matrix!");
        }
        robots_[idx].base_frame = (world_base_frame_ * base).eval();
        robots_[idx].base_inv = math::PInv(robots_[idx].base_frame);
    }
}

void Lego::print_manipulation_property()
{
    for (std::size_t idx = 0; idx < robots_.size(); ++idx)
    {
        std::cout << "\nRobot " << idx + 1 << " Base: \n" << robots_[idx].base_frame << std::endl;
        std::cout << "\nRobot " << idx + 1 << " DH: \n" << robots_[idx].dh << std::endl;
        const auto tool_idx = static_cast<std::size_t>(ToolMode::Tool);
        if (robots_[idx].tool_dh[tool_idx].size() > 0)
        {
            std::cout << "\nRobot " << idx + 1 << " Tool DH: \n" << robots_[idx].tool_dh[tool_idx] << std::endl;
        }
        const auto assemble_idx = static_cast<std::size_t>(ToolMode::ToolAssemble);
        if (robots_[idx].tool_dh[assemble_idx].size() > 0)
        {
            std::cout << "\nRobot " << idx + 1 << " Tool Assemble DH: \n" << robots_[idx].tool_dh[assemble_idx] << std::endl;
        }
        const auto dis_idx = static_cast<std::size_t>(ToolMode::ToolDisassemble);
        if (robots_[idx].tool_dh[dis_idx].size() > 0)
        {
            std::cout << "\nRobot " << idx + 1 << " Tool Disassemble DH: \n" << robots_[idx].tool_dh[dis_idx] << std::endl;
        }
    }
    std::cout << "\n" << std::endl;
}

void Lego::set_DH(const std::vector<std::string>& dh_fnames)
{
    if (dh_fnames.size() != robots_.size())
    {
        throw std::runtime_error("DH filename count does not match number of robots");
    }
    for (std::size_t idx = 0; idx < dh_fnames.size(); ++idx)
    {
        std::cout << "Load Robot " << idx + 1 << " DH from: " << dh_fnames[idx] << std::endl;
        robots_[idx].dh = io::LoadMatFromFile(dh_fnames[idx]);
        robots_[idx].dof = static_cast<int>(robots_[idx].dh.rows());
        Matrix4d flange = make_tool_frame_from_dh(robots_[idx].dh);
        robots_[idx].ee_inv = math::PInv(flange);
    }
}


void Lego::set_DH_tool(const std::vector<std::string>& tool_fnames, ToolMode mode)
{
    if (tool_fnames.size() != robots_.size())
    {
        throw std::runtime_error("Tool DH filename count does not match number of robots");
    }
    const std::size_t mode_idx = static_cast<std::size_t>(mode);
    for (std::size_t idx = 0; idx < tool_fnames.size(); ++idx)
    {
        std::cout << "Load Robot " << idx + 1 << " DH for tool mode " << static_cast<int>(mode)
                  << " from: " << tool_fnames[idx] << std::endl;
        robots_[idx].tool_dh[mode_idx] = io::LoadMatFromFile(tool_fnames[idx]);
        Matrix4d tool_frame = make_tool_frame_from_dh(robots_[idx].tool_dh[mode_idx]);
        robots_[idx].tool_inv[mode_idx] = math::PInv(tool_frame);
    }
}







void Lego::set_assemble_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
    Matrix3d rot = yawAngle.toRotationMatrix();
    rot *= pitchAngle.toRotationMatrix();
    rot *= rollAngle.toRotationMatrix();
    Quaterniond quat(rot);
    assemble_plate_.pose = Matrix4d::Identity(4, 4);
    assemble_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
    assemble_plate_.pose.col(3) << x, y, z, 1;
}

void Lego::set_storage_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
    Matrix3d rot = yawAngle.toRotationMatrix();
    rot *= pitchAngle.toRotationMatrix();
    rot *= rollAngle.toRotationMatrix();
    Quaterniond quat(rot);
    storage_plate_.pose = Matrix4d::Identity(4, 4);
    storage_plate_.pose.block(0, 0, 3, 3) = quat.matrix();
    storage_plate_.pose.col(3) << x, y, z, 1;
}

void Lego::calc_brick_loc(const lego_brick& brick, const lego_plate& plate, const int& orientation,
                            const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z, 
                            Matrix4dRef out_pose)
{
    int brick_height = brick.height;
    int brick_width = brick.width;
    Matrix4d refpose = plate.pose;
    Matrix4d topleft_offset = Matrix4d::Identity(4, 4);
    Matrix4d brick_offset = Matrix4d::Identity(4, 4);
    Matrix4d brick_center_offset = Matrix4d::Identity(4, 4);
    Matrix4d z_90;
    z_90 << 0, -1, 0, 0, 
             1, 0, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    brick_offset.col(3) << brick_loc_x * P_len_ - brick_len_offset_,
                           brick_loc_y * P_len_ - brick_len_offset_,
                           brick_loc_z * brick_height_m_,
                           1;
    brick_center_offset.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0,
                                  (brick_width * P_len_ - brick_len_offset_) / 2.0,
                                 0,
                                 1;
    topleft_offset.col(3) << -(plate.width * P_len_ - brick_len_offset_) / 2.0,
                             -(plate.height * P_len_ - brick_len_offset_) / 2.0,
                             0,
                             1;
    Matrix4d base_pose = (refpose * topleft_offset).eval();
    base_pose = (base_pose * brick_offset).eval();
    if(orientation == 1)
    {
        Matrix4d flipped_center = brick_center_offset;
        flipped_center(1, 3) = -flipped_center(1, 3);
        Matrix4d rotated = (base_pose * z_90).eval();
        out_pose = (rotated * flipped_center).eval();
    }
    else
    {
        out_pose = (base_pose * brick_center_offset).eval();
    }
}

void Lego::brick_pose_in_stock(const std::string& name, const int& press_side, const int& press_offset, Matrix4dRef T)
{
    lego_brick l_brick = brick_map_[name];
    int brick_height = l_brick.height;
    int brick_width = l_brick.width;
    brick_map_[name].press_side = press_side;
    brick_map_[name].press_offset = press_offset;
    
    double x, y, z;
    Matrix4d y_180, z_180, z_90;
    y_180 << -1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, -1, 0,
             0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Matrix3d rot_mtx = Matrix3d::Identity(3, 3);
    Matrix4d brick_pose_mtx = Matrix4d::Identity(4, 4);
    Matrix4d grab_offset_mtx = Matrix4d::Identity(4, 4);
    Matrix4d grab_pose_mtx = Matrix4d::Identity(4, 4);
    Quaterniond quat(l_brick.quat_w, l_brick.quat_x, l_brick.quat_y, l_brick.quat_z);
    rot_mtx = quat.normalized().toRotationMatrix();
    brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
    brick_pose_mtx(0, 3) = l_brick.x;
    brick_pose_mtx(1, 3) = l_brick.y;
    brick_pose_mtx(2, 3) = l_brick.z;
    brick_pose_mtx = brick_pose_mtx * y_180;
    int center_press_offset = 0;

    if(press_side == 1)
    {
        grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
        if(brick_width == 1)
        {
            grab_offset_mtx(1, 3) = (P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_width / 2) - 1;
            grab_offset_mtx(1, 3) = (press_offset - center_press_offset) * P_len_;
        }
        grab_offset_mtx = grab_offset_mtx * z_180;
    }
    else if(press_side == 2)
    {
        if(brick_height == 1)
        {
            grab_offset_mtx(0, 3) = -(P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_height / 2) - 1;
            grab_offset_mtx(0, 3) = (center_press_offset - press_offset) * P_len_;
        }
        grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
    }
    else if(press_side == 3)
    {
        if(brick_height == 1)
        {
            grab_offset_mtx(0, 3) = (P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_height / 2) - 1;
            grab_offset_mtx(0, 3) = (center_press_offset - press_offset) * P_len_;
        }
        grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_90;
    }
    else if(press_side == 4)
    {
        grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
        if(brick_width == 1)
        {
            grab_offset_mtx(1, 3) = -(P_len_ - brick_len_offset_) / 2.0;
        }
        else
        {
            center_press_offset = (brick_width / 2) - 1;
            grab_offset_mtx(1, 3) = (press_offset - center_press_offset) * P_len_;
        }
        grab_offset_mtx = grab_offset_mtx;
    }
    grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;  
    T = grab_pose_mtx;
}

void Lego::support_pose_down_pre(const int& x, const int& y, const int& z, const int& ori, Matrix4dRef T)
{
    Eigen::MatrixXd init_q(default_robot_dof_, 1);
    Matrix4d init_T, tmp_T;
    int goal_x, goal_y, goal_z, goal_ori, goal_press_side;
    goal_z = z - 1;
    if(ori == 0)
    {
        init_q.col(0) << 0, 0, 0, 0, 0, 0;
        goal_x = x - 3;
        goal_y = y;
        goal_ori = 0;
        goal_press_side = 1;
    }
    else if(ori == 1)
    {
        init_q.col(0) << -90, 0, 0, 0, 0, 0;
        goal_x = x;
        goal_y = y + 3;
        goal_ori = 1;
        goal_press_side = 4;
    }
    else if(ori == 2)
    {
        init_q.col(0) << 90, 0, 0, 0, 0, 0;
        goal_x = x;
        goal_y = y - 3;
        goal_ori = 1;
        goal_press_side = 1;
    }
    else
    {
        init_q.col(0) << 180, 0, 0, 0, 0, 0;
        goal_x = x + 3;
        goal_y = y;
        goal_ori = 0;
        goal_press_side = 4;
    }
    init_T = math::FK(init_q, robot_DH_tool_r1(), robot_base_r1(), false);
    assemble_pose_from_top(goal_x, goal_y, goal_z, goal_ori, goal_press_side, tmp_T);
    init_T.col(3) << tmp_T(0, 3), tmp_T(1, 3), tmp_T(2, 3), 1;
    T = init_T;
}

void Lego::support_pose_down(const int& x, const int& y, const int& z, const int& ori, Matrix4dRef T)
{
    Eigen::MatrixXd init_q(default_robot_dof_, 1);
    Matrix4d init_T, tmp_T;
    int goal_x, goal_y, goal_z, goal_ori, goal_press_side;
    goal_z = z - 1;
    if(ori == 0)
    {
        init_q.col(0) << 0, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_x = x + 1;
        goal_y = y;
        goal_press_side = 1;
    }
    else if(ori == 1)
    {
        init_q.col(0) << -90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_x = x;
        goal_y = y - 1;
        goal_press_side = 4;
    }
    else if(ori == 2)
    {
        init_q.col(0) << 90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_x = x;
        goal_y = y + 1;
        goal_press_side = 1;
    }
    else
    {
        init_q.col(0) << 180, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_x = x - 1;
        goal_y = y;
        goal_press_side = 4;
    }
    init_T = math::FK(init_q, robot_DH_tool_r1(), robot_base_r1(), false);
    assemble_pose_from_top(goal_x, goal_y, goal_z, goal_ori, goal_press_side, tmp_T);
    init_T.col(3) << tmp_T(0, 3), tmp_T(1, 3), tmp_T(2, 3) + (brick_height_m_ - (P_len_ - brick_len_offset_)), 1;
    Eigen::MatrixXd offset_T = Eigen::MatrixXd::Identity(4, 4);
    offset_T.col(3) << 0, 0, -lever_wall_height_, 1;
    T = init_T * offset_T;
}

void Lego::support_pose(const int& x, const int& y, const int& z, const int& ori, Matrix4dRef T)
{
    Eigen::MatrixXd init_q(default_robot_dof_, 1);
    Matrix4d init_T, tmp_T;
    int goal_x, goal_y, goal_z, goal_ori, goal_press_side;
    goal_z = z;

    if(ori == 0)
    {
        init_q.col(0) << 0, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_press_side = 1;
        goal_x = x + 1;
        goal_y = y;
    }
    else if(ori == 1)
    {
        init_q.col(0) << -90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_press_side = 4;
        goal_x = x;
        goal_y = y - 1;
    }
    else if(ori == 2)
    {
        init_q.col(0) << 90, 0, 0, 0, 0, 0;
        goal_ori = 1;
        goal_press_side = 1;
        goal_x = x;
        goal_y = y + 1;
    }
    else
    {
        init_q.col(0) << 180, 0, 0, 0, 0, 0;
        goal_ori = 0;
        goal_press_side = 4;
        goal_x = x - 1;
        goal_y = y;
    }
    init_T = math::FK(init_q, robot_DH_tool_r1(), robot_base_r1(), false);
    assemble_pose_from_top(goal_x, goal_y, goal_z, goal_ori, goal_press_side, tmp_T);
    init_T.col(3) << tmp_T(0, 3), tmp_T(1, 3), tmp_T(2, 3) + (brick_height_m_ - (P_len_ - brick_len_offset_)), 1;
    Matrix4d offset_T = Matrix4d::Identity(4, 4);
    offset_T.col(3) << 0, 0, -lever_wall_height_, 1;
    T = init_T * offset_T;
}

void Lego::assemble_pose_from_top(const int& press_x, const int& press_y, const int& press_z, const int& press_ori, const int& press_side, Matrix4dRef T)
{
    lego_brick l_brick;
    if (brick_map_.find("b9_1") == brick_map_.end())
    {
        l_brick.height = 1;
        l_brick.width = 2;
    }
    else {
        l_brick = brick_map_["b9_1"];
    }
    
    Quaterniond quat;
    double x, y, z;
    Matrix4d y_180, z_180, z_90;
    y_180 << -1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, -1, 0,
             0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Matrix3d rot_mtx = Matrix3d::Identity(3, 3);
    Matrix4d brick_pose_mtx = Matrix4d::Identity(4, 4);
    Matrix4d grab_offset_mtx = Matrix4d::Identity(4, 4);
    Matrix4d grab_pose_mtx = Matrix4d::Identity(4, 4);
    int side;

    if(press_ori == 0){
        if(press_side == 1){
            side = 1;
        }
        else if(press_side == 2){
            side = 1;
        }
        else if(press_side == 3){
            side = 4;
        }
        else if(press_side == 4){
            side = 4;
        }
    }
    else if(press_ori == 1){
        if(press_side == 1){
            side = 1;
        }
        else if(press_side == 2){
            side = 4;
        }
        else if(press_side == 3){
            side = 1;
        }
        else if(press_side == 4){
            side = 4;
        }
    }
    calc_brick_loc(l_brick, assemble_plate_, press_ori, press_x, press_y, press_z-1, brick_pose_mtx);
    brick_pose_mtx = brick_pose_mtx * y_180;
    int brick_height = 1;
    int brick_width = 2;
    if(side == 1)
    {
        grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx(1, 3) = 0;  
        grab_offset_mtx = grab_offset_mtx * z_180;
    }
    else if(side == 2)
    {
        grab_offset_mtx(0, 3) = 0;
        grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
    }
    else if(side == 3)
    {
        grab_offset_mtx(0, 3) = 0;
        grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_90;
    }
    else if(side == 4)
    {
        grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx(1, 3) = 0;
    }
    grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
    T = grab_pose_mtx;
}

void Lego::lego_pose_from_press_pose(const math::VectorJd& theta, int robot_id, int brick_id, int press_side, int press_offset, 
            Matrix4dRef brick_loc)
{
    // do forward kinematic
    Eigen::MatrixXd cart_T = Matrix4d::Identity(4, 4);
    if (robot_id == 0) {
        cart_T = math::FK(theta, robot_DH_tool_r1(), robot_base_r1(), false);
    }
    else {
        cart_T = math::FK(theta, robot_DH_tool_r2(), robot_base_r2(), false);

    }

    Eigen::MatrixXd grab_offset_mtx = Matrix4d::Identity(4, 4);
    Matrix4d y_180, z_180, z_90;
    y_180 << -1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, -1, 0,
             0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    int brick_height, brick_width;
    get_brick_sizes_by_type(brick_id, brick_height, brick_width);
    if(press_side == 1)
    {
        grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
        grab_offset_mtx = grab_offset_mtx * z_180;
    }
    else if(press_side == 2)
    {
        grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
        grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
    }
    else if(press_side == 3)
    {
        grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
        grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx = grab_offset_mtx * z_90;
    }
    else if(press_side == 4)
    {
        grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
        grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
        grab_offset_mtx = grab_offset_mtx;
    }

    // invert the grab offset matrix
    Matrix4d grab_offset_mtx_inv = math::PInv(grab_offset_mtx);
    brick_loc = cart_T * grab_offset_mtx_inv;
}

int Lego::robot_dof(int robot_id) const
{
    if (robot_id < 0 || robot_id >= static_cast<int>(robots_.size()))
    {
        throw std::out_of_range("robot_dof: invalid robot index");
    }
    return robots_[robot_id].dof;
}

const Eigen::MatrixXd& Lego::robot_DH(int robot_id, ToolMode mode) const
{
    if (robot_id < 0 || robot_id >= static_cast<int>(robots_.size()))
    {
        throw std::out_of_range("robot_DH: invalid robot index");
    }
    const auto mode_idx = static_cast<std::size_t>(mode);
    if (mode == ToolMode::Flange)
    {
        return robots_[robot_id].dh;
    }
    const auto& mat = robots_[robot_id].tool_dh[mode_idx];
    if (mat.rows() == 0 || mat.cols() == 0)
    {
        throw std::runtime_error("Requested tool DH not initialized for robot " + std::to_string(robot_id));
    }
    return mat;
}

const Matrix4d& Lego::robot_base(int robot_id) const
{
    if (robot_id < 0 || robot_id >= static_cast<int>(robots_.size()))
    {
        throw std::out_of_range("robot_base: invalid robot index");
    }
    return robots_[robot_id].base_frame;
}

const Matrix4d& Lego::robot_base_inv(int robot_id) const
{
    if (robot_id < 0 || robot_id >= static_cast<int>(robots_.size()))
    {
        throw std::out_of_range("robot_base_inv: invalid robot index");
    }
    return robots_[robot_id].base_inv;
}

const Matrix4d& Lego::robot_ee_inv(int robot_id) const
{
    if (robot_id < 0 || robot_id >= static_cast<int>(robots_.size()))
    {
        throw std::out_of_range("robot_ee_inv: invalid robot index");
    }
    return robots_[robot_id].ee_inv;
}

const Matrix4d& Lego::robot_tool_inv(int robot_id, ToolMode mode) const
{
    if (robot_id < 0 || robot_id >= static_cast<int>(robots_.size()))
    {
        throw std::out_of_range("robot_tool_inv: invalid robot index");
    }
    const auto mode_idx = static_cast<std::size_t>(mode);
    if (robots_[robot_id].tool_inv[mode_idx].rows() == 0)
    {
        throw std::runtime_error("Requested tool transform not initialized for robot " + std::to_string(robot_id));
    }
    return robots_[robot_id].tool_inv[mode_idx];
}


bool Lego::joint_in_range(const math::VectorJd& theta, const bool& is_rad)
{
    math::VectorJd theta_deg = theta;
    if(is_rad)
    {
        // Rad to Deg
        for(int i=0; i<theta.rows(); i++)
        {
            theta_deg(i) = theta_deg(i) / M_PI * 180;
        }
    }
    for(int i=0; i<theta.rows(); i++)
    {
        if(theta_deg(i) < thetamax_(i, 0) || theta_deg(i) > thetamax_(i, 1))
        {
            return false;
        }
    }
    return true;
}

math::VectorJd Lego::IK(const math::VectorJd& cur_q, Matrix4dConstRef goal_T, const Eigen::MatrixXd& DH, Matrix4dConstRef T_base, Matrix4dConstRef T_base_inv,
                         Matrix4dConstRef T_tool_inv, const bool& joint_rad, bool& status)
{
    double eps = 1e-10;
    status = false;
    math::VectorJd theta = cur_q;
    Eigen::MatrixXd DH_cur = DH;
    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<cur_q.rows(); i++)
        {
            theta(i) = theta(i) * M_PI / 180;
        }
    }
    math::VectorJd cur_theta = theta;
    math::VectorJd theta_tmp = theta;
    Matrix4d T = (T_base_inv * goal_T).eval();
    T = (T * T_tool_inv).eval();
    Matrix3d R = T.block(0, 0, 3, 3);
    Vector3d P = T.block(0, 3, 3, 1);
    double X, Y, Z, r, r2, a1, a12, a2, a22, a3, a32, d4, d42, m, e, c, l, l2, h, f1, f2, t1, t2, t3, k, g1, g2, q1, q2, min_diff;
    double th1_tmp, th2_tmp, th3_tmp, th4_tmp, th5_tmp, th6_tmp;
    X = P(0, 0);
    Y = P(1, 0);
    Z = P(2, 0);
    r = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));
    a1 = DH(0, 2);
    a2 = DH(1, 2);
    a3 = DH(2, 2);
    d4 = DH(3, 1);
    r2 = pow(r, 2);
    a12 = pow(a1, 2);
    a22 = pow(a2, 2);
    a32 = pow(a3, 2);
    d42 = pow(d4, 2);
    m = a32 + d42;
    e = 2 * r2;
    c = 4 * a12;
    l = a2 * (2 * m + 2 * a12 + 2 * a22 - c - e);
    l2 = pow(l, 2);
    h = (c + e) * m - pow((m + a12 + a22), 2) + a12 * e + a22 * e + 4 * a12 * a22 - 4 * a12 * pow(Z, 2) - pow(r, 4);

    double cond1, cond2, th2_tmp1, th2_tmp2;
    cond1 = 4 * l2 + 16 * a22 * h;
    min_diff = 10000000;
    Eigen::MatrixXd th23_candidates;
    int th23_candidate_cnt, th1_candidate_cnt, all_candidate_cnt;
    th23_candidate_cnt = 0;
    th1_candidate_cnt = 0;
    all_candidate_cnt = 0;
    th23_candidates.resize(8, 2);

    if(cond1 < 0 && abs(cond1) < 0.0001)
    {
        cond1 = 0;
    }
    if(cond1 >= 0)
    {
        f1 = (-2 * l + sqrt(cond1)) / (8 * a22 + eps);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2 >= 0)
        {
            // First candidate
            th3_tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1 + eps));

            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;

            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }

            // Second candidate
            th3_tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
        }
        f1 = (-2 * l - sqrt(cond1)) / (8 * a22 + eps);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2)
        {
            // Third candidate
            th3_tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
            
            // Fourth candidate
            th3_tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1 + eps));
            f1 = -sin(th3_tmp) * d4 + a3 * cos(th3_tmp);
            f2 = cos(th3_tmp) * d4 + a3 * sin(th3_tmp);
            t1 = f1 + a2;
            k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
            t2 = (r2 - k) / (2 * a1 + eps);
            t1 = f2;
            t2 = -f1-a2;
            t3 = Z;
            th2_tmp1 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;
            th2_tmp2 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3 + eps)) + M_PI / 2;
            if(th3_tmp < thetamax_rad_(2, 1) && th3_tmp > thetamax_rad_(2, 0))
            {
                if(th2_tmp1 < thetamax_rad_(1, 1) && th2_tmp1 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp1, th3_tmp;
                    th23_candidate_cnt ++;
                }
                if(th2_tmp2 < thetamax_rad_(1, 1) && th2_tmp2 > thetamax_rad_(1, 0))
                {
                    th23_candidates.row(th23_candidate_cnt) << th2_tmp2, th3_tmp;
                    th23_candidate_cnt ++;
                }
            }
        }
    }
    else
    {
        std::cout<<"IK failed condition 1"<<std::endl;
        status = false;
        return cur_q;
    }
    
    Eigen::MatrixXd th1_candidates, candidates;
    Matrix4d verify_T;
    th1_candidates.resize(2, 1);
    candidates.resize(32, default_robot_dof_);
    double th1_tmp1, th1_tmp2;
    for(int i=0; i<th23_candidate_cnt; i++)
    {
        th2_tmp = th23_candidates(i, 0) - M_PI / 2;
        th3_tmp = th23_candidates(i, 1);
        th1_candidate_cnt = 0;

        g1 = f1 * cos(th2_tmp) + f2 * sin(th2_tmp) + a2 * cos(th2_tmp);
        g2 = f1 * sin(th2_tmp) - f2 * cos(th2_tmp) + a2 * sin(th2_tmp);
        q1 = g1+a1;
        q2 = 0;
        cond1 = pow(q2, 2) + pow(q1, 2) - pow(X, 2);
        q1 = 0;
        q2 = g1+a1;
        th1_tmp1 = 2 * atan((q2 + sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y + eps));
        th1_tmp2 = 2 * atan((q2 - sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y + eps));
        if(th1_tmp1 < thetamax_rad_(0, 1) && th1_tmp1 > thetamax_rad_(0, 0))
        {
            th1_candidates.row(th1_candidate_cnt) << th1_tmp1;
            th1_candidate_cnt ++;
        }
        if(th1_tmp2 < thetamax_rad_(0, 1) && th1_tmp2 > thetamax_rad_(0, 0))
        {
            th1_candidates.row(th1_candidate_cnt) << th1_tmp2;
            th1_candidate_cnt ++;
        }
        for(int j=0; j<th1_candidate_cnt; j++)
        {
            theta_tmp(0) = th1_candidates(j, 0);
            theta_tmp(1) = th2_tmp + M_PI / 2;;
            theta_tmp(2) = th3_tmp;
            DH_cur.col(0) = DH.col(0) + theta_tmp;
            Matrix3d R03 = Matrix3d::Identity(3, 3);
            Eigen::MatrixXd a = DH_cur.col(3);
            Eigen::MatrixXd q = DH_cur.col(0);
            Matrix3d temp(3, 3); 
            for(int k=0; k<3; k++)
            {
                temp << cos(q(k)), -sin(q(k)) * cos(a(k)),  sin(q(k)) * sin(a(k)),  
                        sin(q(k)),  cos(q(k)) * cos(a(k)), -cos(q(k)) * sin(a(k)),  
                        0,          sin(a(k)),              cos(a(k));
                R03 = R03 * temp;
            }
            Matrix3d R36 = math::PInv(R03) * R;
            th5_tmp = acos(-R36(2, 2));
            double s5 = sin(th5_tmp) + eps;

            if(abs(s5) <= 0.001)
            {
                th4_tmp = 0;
                th5_tmp = 0;
                th6_tmp = atan2(R36(0, 1), R36(0, 0));
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }

                th5_tmp = M_PI;
                th6_tmp = atan2(R36(1, 0), -R36(1, 1));
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
            }
            else
            {
                double th4_1 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                double th6_1 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
                double sum1 = sqrt(pow(th5_tmp, 2) + pow(th4_1, 2) + pow(th6_1, 2));
                s5 = sin(-th5_tmp);
                double th4_2 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                double th6_2 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
                double sum2 = sqrt(pow(th5_tmp, 2) + pow(th4_2, 2) + pow(th6_2, 2));

                th4_tmp = th4_1;
                th6_tmp = th6_1;
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
                
                th5_tmp = -th5_tmp;
                th4_tmp = th4_2;
                th6_tmp = th6_2;
                theta_tmp(3) = th4_tmp;
                theta_tmp(4) = th5_tmp;
                theta_tmp(5) = th6_tmp;
                if(joint_in_range(theta_tmp, 1))
                {
                    candidates.row(all_candidate_cnt) << theta_tmp.transpose(); 
                    all_candidate_cnt ++;
                }
            } 
        }  
    }
    status = false;
    for(int i=0; i<all_candidate_cnt; i++)
    {   
        for(int j=-1; j<2; j++)
        {
            theta_tmp = candidates.row(i);
            theta_tmp(5) = theta_tmp(5) + j * 2 * M_PI;
            verify_T = math::FK(theta_tmp, DH, T_base, 1);
            
            if(verify_T.isApprox(goal_T, 0.1) && verify_T.col(3).isApprox(goal_T.col(3), 0.01) && (theta_tmp - cur_theta).norm() < min_diff && joint_in_range(theta_tmp, 1))
            {
                theta = theta_tmp;
                min_diff = (theta_tmp - cur_theta).norm();
                status = true;            
            }
        }
        
    }
    if(!status)
    {
        std::cout<<"IK failed! No valid candidate."<<std::endl;
        return cur_q;
    }

    // Rad to Deg
    for(int i=0; i<theta.rows(); i++)
    {
        theta(i) = theta(i) * 180 / M_PI;
    }
    return theta;
}

void Lego::calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                       const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                       const int& orientation, const int& press_side, const int& press_offset,
                                       Eigen::MatrixXd& T)
{
    if (brick_map_.find(name) == brick_map_.end())
    {
        std::cout << "Brick " << name << " not found in the brick map!" << std::endl;
        return;
    }
    lego_brick l_brick = brick_map_[name];
    int brick_height = l_brick.height;
    int brick_width = l_brick.width;
    brick_map_[name].press_side = press_side;
    brick_map_[name].press_offset = press_offset;
    
    double x, y, z;
    Matrix4d y_180, z_180, z_90;
    y_180 << -1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, -1, 0,
             0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Matrix3d rot_mtx = Matrix3d::Identity(3, 3);
    Matrix4d brick_pose_mtx = Matrix4d::Identity(4, 4);
    Matrix4d grab_offset_mtx = Matrix4d::Identity(4, 4);
    Matrix4d grab_pose_mtx = Matrix4d::Identity(4, 4);
    
    // Get pose to grab the brick
    if(take_brick)
    {
        // Brick on storage plate
        if(l_brick.in_stock)
        {
            Quaterniond quat(l_brick.quat_w, l_brick.quat_x, l_brick.quat_y, l_brick.quat_z);
            rot_mtx = quat.normalized().toRotationMatrix();
            brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
            brick_pose_mtx(0, 3) = l_brick.x;
            brick_pose_mtx(1, 3) = l_brick.y;
            brick_pose_mtx(2, 3) = l_brick.z;
            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else if(press_side == 4)
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
                grab_offset_mtx = grab_offset_mtx;
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }  
        // Brick on assemble plate
        else
        {
            Quaterniond quat(l_brick.cur_quat.w(), l_brick.cur_quat.x(), l_brick.cur_quat.y(), l_brick.cur_quat.z());
            rot_mtx = quat.normalized().toRotationMatrix();
            brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
            brick_pose_mtx(0, 3) = l_brick.cur_x;
            brick_pose_mtx(1, 3) = l_brick.cur_y;
            brick_pose_mtx(2, 3) = l_brick.cur_z;
            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else if(press_side == 4)
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_; 
                grab_offset_mtx = grab_offset_mtx;
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }
    }
    // Get pose to place the brick
    else
    {
        // Place on storage plate
        if(!assemble_pose)
        {
            Quaterniond quat(l_brick.quat_w, l_brick.quat_x, l_brick.quat_y, l_brick.quat_z);
            rot_mtx = quat.normalized().toRotationMatrix();
            brick_pose_mtx.block(0, 0, 3, 3) << rot_mtx;
            brick_pose_mtx(0, 3) = l_brick.x;
            brick_pose_mtx(1, 3) = l_brick.y;
            brick_pose_mtx(2, 3) = l_brick.z;
            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else if(press_side == 4)
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_; 
                grab_offset_mtx = grab_offset_mtx;
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }  
        // Place on assemble plate
        else
        {
            calc_brick_loc(l_brick, assemble_plate_, orientation, 
                           brick_assemble_x, brick_assemble_y, brick_assemble_z, brick_pose_mtx);

            brick_pose_mtx = brick_pose_mtx * y_180;

            if(press_side == 1)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
                grab_offset_mtx = grab_offset_mtx * z_180;
            }
            else if(press_side == 2)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = (brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_180 * z_90;
            }
            else if(press_side == 3)
            {
                grab_offset_mtx(0, 3) = (brick_height * P_len_) / 2.0 - (press_offset + 1) * P_len_;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx = grab_offset_mtx * z_90;
            }
            else
            {
                grab_offset_mtx(0, 3) = -(brick_height * P_len_ - brick_len_offset_) / 2.0;
                grab_offset_mtx(1, 3) = -(brick_width * P_len_) / 2.0 + (press_offset + 1) * P_len_;
            }
            grab_pose_mtx = brick_pose_mtx * grab_offset_mtx;
        }
    }
    T = grab_pose_mtx;
}

void Lego::calc_brick_sup_pose(int robot_id, const int &sup_x,
    const int &sup_y, const int &sup_z, const int &sup_ori, const double &z_offset, Eigen::MatrixXd &T)
{
    // calc the support x y z
    Matrix4d sup_pose_mtx = Matrix4d::Identity(4, 4);
    lego_brick l_brick;
    l_brick.height = 1;
    l_brick.width = 2;
    calc_brick_loc(l_brick, assemble_plate_, sup_ori, sup_x, sup_y, sup_z, sup_pose_mtx);
    
    // subtract the thickness of the tool
    sup_pose_mtx(2, 3) -= 0.0078;

    sup_pose_mtx(2, 3) += z_offset;

    // copy the sup pose's orientation
    sup_pose_mtx.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);

    T = sup_pose_mtx;
}

std::string Lego::get_brick_name_by_id(const int& id, const int& seq_id)
{
    std::string brick_name = "b" + std::to_string(id) + "_" + std::to_string(seq_id);
    if(brick_map_.find(brick_name) == brick_map_.end())
    {
        std::cout << "No available brick! ID: " << id << ", Seq ID: " << seq_id << std::endl;
    }
    return brick_name;
}

std::string Lego::get_brick_name_by_id(const int& id, const std::string& seq_id)
{
    std::string brick_name = "b" + std::to_string(id) + "_" + seq_id;
    if(brick_map_.find(brick_name) == brick_map_.end())
    {
        std::cout << "No available brick! ID: " << id << ", Seq ID: " << seq_id << std::endl;
    }
    return brick_name;
}

void Lego::get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry)
{
    // Landscape orientation
    if((math::ApproxEqNum(b1.cur_quat.x(), 0,EPS_*10000) && math::ApproxEqNum(b1.cur_quat.y(), 0,EPS_*10000) && math::ApproxEqNum(b1.cur_quat.z(), 0,EPS_*10000) && math::ApproxEqNum(b1.cur_quat.w(), 1,EPS_*10000)))
    {
        lx = b1.cur_x - (b1.height * P_len_ - brick_len_offset_) / 2.0;
        ly = b1.cur_y - (b1.width * P_len_ - brick_len_offset_) / 2.0;
        rx = b1.cur_x + (b1.height * P_len_ - brick_len_offset_) / 2.0;
        ry = b1.cur_y + (b1.width * P_len_ - brick_len_offset_) / 2.0;
    }
    // Vertcal orientation
    else
    {
        lx = b1.cur_x - (b1.width * P_len_ - brick_len_offset_) / 2.0;
        ly = b1.cur_y - (b1.height * P_len_ - brick_len_offset_) / 2.0;
        rx = b1.cur_x + (b1.width * P_len_ - brick_len_offset_) / 2.0;
        ry = b1.cur_y + (b1.height * P_len_ - brick_len_offset_) / 2.0;   
    }
}


bool Lego::bricks_overlap(const lego_brick& b1, const lego_brick& b2)
{
    double l1x, l1y, r1x, r1y, l2x, l2y, r2x, r2y;
    get_brick_corners(b1, l1x, l1y, r1x, r1y);
    get_brick_corners(b2, l2x, l2y, r2x, r2y);

    if(math::ApproxEqNum(l1x, r1x, EPS_) || 
       math::ApproxEqNum(l1y, r1y, EPS_) || 
       math::ApproxEqNum(l2x, r2x, EPS_) || 
       math::ApproxEqNum(l2y, r2y, EPS_))
    {
        return false;
    }
    if(l1x >= r2x || l2x >= r1x)
    {
        return false;
    }
    if(l2y >= r1y || l1y >= r2y)
    {
        return false;
    }
    return true;
}

bool Lego::is_top_connect(const lego_brick& b1, const lego_brick& b2)
{
    double b1x = b1.cur_x;
    double b1y = b1.cur_y;
    double b1z = b1.cur_z;
    double b2x = b2.cur_x;
    double b2y = b2.cur_y;
    double b2z = b2.cur_z;
    if(b2z - b1z < 0.1 * brick_height_m_ || (b2z - b1z) > 1.1 * brick_height_m_)
    {
        return false;
    }
    
    if(!bricks_overlap(b1, b2))
    {
        return false;
    }
    return true;
}

bool Lego::is_bottom_connect(const lego_brick& b1, const lego_brick& b2)
{
    double b1x = b1.cur_x;
    double b1y = b1.cur_y;
    double b1z = b1.cur_z;
    double b2x = b2.cur_x;
    double b2y = b2.cur_y;
    double b2z = b2.cur_z;
    if(b1z - b2z < 0.1 * brick_height_m_ || (b1z - b2z) > 1.1 * brick_height_m_)
    {
        return false;
    }
    
    if(!bricks_overlap(b1, b2))
    {
        return false;
    }
    return true;
}


void Lego::update_brick_connection()
{
    auto start = high_resolution_clock::now();
    for(auto b1:brick_map_)
    {
        brick_map_[b1.second.brick_name].top_connect.clear();
        brick_map_[b1.second.brick_name].bottom_connect.clear();
        b1.second.top_connect.clear();
        b1.second.bottom_connect.clear();
        for(auto b2:brick_map_)
        {
            if(b1.second.brick_name.compare(b2.second.brick_name) == 0)
            {
                continue;
            }
            if(is_top_connect(b1.second, b2.second))
            {
                brick_map_[b1.second.brick_name].top_connect[b2.second.brick_name] = b2.second.brick_name;
            }
            if(is_bottom_connect(b1.second, b2.second))
            {
                brick_map_[b1.second.brick_name].bottom_connect[b2.second.brick_name] = b2.second.brick_name;
            }
        }
    }
    
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "\nUpdate brick connection time: " << duration.count() / 1000000.0 << " s" << std::endl;
}


void Lego::update_all_top_bricks(const std::string& brick_name, Matrix4dConstRef dT)
{
    for(auto top_brick_n:brick_map_[brick_name].top_connect)
    {
        std::string top_brick_name = top_brick_n.second;
        lego_brick top_brick = brick_map_[top_brick_name];

        Matrix4d cur_T = Matrix4d::Identity(4, 4);
        cur_T.col(3) << top_brick.cur_x, top_brick.cur_y, top_brick.cur_z, 1;
        cur_T.block(0, 0, 3, 3) << top_brick.cur_quat.normalized().toRotationMatrix();
        Matrix4d new_T = dT * cur_T;
        Matrix3d new_rot = new_T.block(0, 0, 3, 3);
        Quaterniond new_quat(new_rot);

        brick_map_[top_brick_name].cur_x = new_T.coeff(0, 3);
        brick_map_[top_brick_name].cur_y = new_T.coeff(1, 3);
        brick_map_[top_brick_name].cur_z = new_T.coeff(2, 3);
        brick_map_[top_brick_name].cur_quat = new_quat;
        // Update brick in stock status
        brick_map_[top_brick_name].in_stock = (math::ApproxEqNum(brick_map_[top_brick_name].x, brick_map_[top_brick_name].cur_x, EPS_ * 100) && 
                                               math::ApproxEqNum(brick_map_[top_brick_name].y, brick_map_[top_brick_name].cur_y, EPS_ * 100) && 
                                               math::ApproxEqNum(brick_map_[top_brick_name].z, brick_map_[top_brick_name].cur_z, EPS_ * 100)); // scale up eps_

        //gazebo_msgs::ModelState new_pose;
        // new_pose.model_name = top_brick_name;
        // new_pose.pose.position.x = brick_map_[top_brick_name].cur_x;
        // new_pose.pose.position.y = brick_map_[top_brick_name].cur_y;
        // new_pose.pose.position.z = brick_map_[top_brick_name].cur_z;
        // new_pose.pose.orientation.x = brick_map_[top_brick_name].cur_quat.x();
        // new_pose.pose.orientation.y = brick_map_[top_brick_name].cur_quat.y();
        // new_pose.pose.orientation.z = brick_map_[top_brick_name].cur_quat.z();
        // new_pose.pose.orientation.w = brick_map_[top_brick_name].cur_quat.w();

        // setmodelstate_.request.model_state = new_pose;
        // client_.call(setmodelstate_);
        // update_all_top_bricks(top_brick_name, dT);
    }

}

void Lego::update(const std::string& brick_name, Matrix4dConstRef T_init)
{
    lego_brick cur_brick = brick_map_[brick_name];
    int brick_height = cur_brick.height;
    int brick_width = cur_brick.width;
    int press_offset = cur_brick.press_offset;
    
    Matrix4d tmp = Matrix4d::Identity(4, 4);
    Matrix4d dT, new_brick_T;
    Matrix4d cur_brick_T = Matrix4d::Identity(4, 4);
    cur_brick_T.col(3) << cur_brick.cur_x, cur_brick.cur_y, cur_brick.cur_z, 1;
    cur_brick_T.block(0, 0, 3, 3) << cur_brick.cur_quat.normalized().toRotationMatrix();

    Matrix4d y_180, z_90, z_180;
    z_90 << 0, -1, 0, 0, 
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    y_180 << -1, 0, 0, 0,
              0, 1, 0, 0, 
              0, 0, -1, 0, 
              0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    new_brick_T = T_init * y_180 * z_180;
    int center_press_offset = 0;
    if(cur_brick.press_side == 1)
    {
        center_press_offset = (brick_width / 2) - 1;
        if(brick_width % 2 == 0)
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, (center_press_offset - press_offset) * P_len_, 0, 1;
        }
        else
        {
            tmp.col(3) << (brick_height * P_len_ - brick_len_offset_) / 2.0, -(P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        new_brick_T = new_brick_T * tmp;
        
    }
    if(cur_brick.press_side == 4)
    {
        center_press_offset = (brick_width / 2) - 1;
        if(brick_width % 2 == 0)
        {
            tmp.col(3) << -(brick_height * P_len_ - brick_len_offset_) / 2.0, (center_press_offset - press_offset) * P_len_, 0, 1;
        }
        else
        {
            tmp.col(3) << -(brick_height * P_len_ - brick_len_offset_) / 2.0, (P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        new_brick_T = new_brick_T * z_180 * tmp;
    }
    else if(cur_brick.press_side == 2)
    {
        center_press_offset = (brick_height / 2) - 1;
        if(brick_height % 2 == 0)
        {
            tmp.col(3) << (press_offset - center_press_offset) * P_len_, -(brick_width * P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        else
        {
            tmp.col(3) << -(P_len_ - brick_len_offset_) / 2.0, -(brick_width * P_len_ - brick_len_offset_) / 2.0,  0, 1;
        }
        new_brick_T = new_brick_T * z_90 * tmp ;
    }
    else if(cur_brick.press_side == 3)
    {
        
        if(brick_height % 2 == 0)
        {
            tmp.col(3) << (press_offset - center_press_offset) * P_len_, (brick_width * P_len_ - brick_len_offset_) / 2.0, 0, 1;
        }
        else
        {
            tmp.col(3) << -(P_len_ - brick_len_offset_) / 2.0, (brick_width * P_len_ - brick_len_offset_) / 2.0,  0, 1;
        }
        new_brick_T = new_brick_T * z_90 * z_180 * tmp ;
    }
    dT = new_brick_T * math::PInv(cur_brick_T);
    
    update_all_top_bricks(brick_name, dT);
    Matrix3d rot_mtx = new_brick_T.block(0, 0, 3, 3);
    Quaterniond new_quat(rot_mtx);
    brick_map_[brick_name].cur_x = new_brick_T.coeff(0, 3);
    brick_map_[brick_name].cur_y = new_brick_T.coeff(1, 3);
    brick_map_[brick_name].cur_z = new_brick_T.coeff(2, 3);
    brick_map_[brick_name].cur_quat =  new_quat;
    
    // Update brick in stock status
    brick_map_[brick_name].in_stock = (math::ApproxEqNum(brick_map_[brick_name].x, brick_map_[brick_name].cur_x, EPS_ * 100) && 
                                       math::ApproxEqNum(brick_map_[brick_name].y, brick_map_[brick_name].cur_y, EPS_ * 100) && 
                                       math::ApproxEqNum(brick_map_[brick_name].z, brick_map_[brick_name].cur_z, EPS_ * 100));

    // gazebo_msgs::ModelState new_pose;
    // new_pose.model_name = brick_name;
    // new_pose.pose.position.x = brick_map_[brick_name].cur_x;
    // new_pose.pose.position.y = brick_map_[brick_name].cur_y;
    // new_pose.pose.position.z = brick_map_[brick_name].cur_z;
    // new_pose.pose.orientation.x = brick_map_[brick_name].cur_quat.x();
    // new_pose.pose.orientation.y = brick_map_[brick_name].cur_quat.y();
    // new_pose.pose.orientation.z = brick_map_[brick_name].cur_quat.z();
    // new_pose.pose.orientation.w = brick_map_[brick_name].cur_quat.w();
    
    // setmodelstate_.request.model_state = new_pose;
    // client_.call(setmodelstate_);
}

void Lego::update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                                const bool& joint_rad, const std::string& brick_name, const int& mode)
{
    Matrix4d T = math::FK(robot_q, DH, base_frame, joint_rad);
    if(mode == 1)
    {
        Matrix4d y_p90, z_180;
        y_p90 << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
        z_180 << -1, 0, 0, 0,
                 0, -1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
        T = T * y_p90 * z_180;
    }
    std::string closest_brick_name = brick_name;
    update(brick_name, T);
}


bool Lego::robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd, const int& robot_dof)
{
    for(int i=0; i<robot_dof; i++)
    {
        if(abs(robot_qd(i)) > EPS_ || abs(robot_qdd(i)) > EPS_)
        {
            return false;
        }
    }
    return true;
}


bool Lego::robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal, const int& robot_dof)
{
    for(int i=0; i<robot_dof; i++)
    {
        if(abs(robot_q(i) - goal(i)) > EPS_)
        {
            return false;
        }
    }
    return true;
}

std::vector<std::string> Lego::get_brick_names() {
    std::vector<std::string> brick_names;
    for(auto brick:brick_map_)
    {
        brick_names.push_back(brick.first);
    }
    return brick_names;
}

std::vector<std::string> Lego::get_active_bricks_names() {
    std::vector<std::string> brick_names;
    for(auto brick:brick_map_)
    {
        if(!brick.second.fixed) {
            brick_names.push_back(brick.first);
        }
    }
    return brick_names;
}

std::vector<std::string> Lego::get_fixed_bricks_names() {
    std::vector<std::string> brick_names;
    for(auto brick:brick_map_)
    {
        if(brick.second.fixed) {
            brick_names.push_back(brick.first);
        }
    }
    return brick_names;
}

std::vector<std::string> Lego::get_brick_names_by_type(int id) {
    std::string brick_id = "b" + std::to_string(id) + "_";
    
    std::vector<std::string> brick_names;
    for(auto brick:brick_map_)
    {
        if(brick.first.find(brick_id) == 0) {
            brick_names.push_back(brick.first);
        }
    }
    return brick_names;
}

std::vector<std::string> Lego::get_brick_above(const std::string& brick_name) {
    std::vector<std::string> above_bricks;
    for(auto brick:brick_map_[brick_name].top_connect)
    {
        above_bricks.push_back(brick.first);
    }
    return above_bricks;
}

std::vector<std::string> Lego::get_brick_below(const std::string& brick_name) {
    std::vector<std::string> below_bricks;
    for(auto brick:brick_map_[brick_name].bottom_connect)
    {
        below_bricks.push_back(brick.first);
    }
    return below_bricks;
}

void Lego::get_init_brick_xyzo(const std::string& brick_name, int& x, int& y, int& z, int &ori)
{
    for(auto brick = config_.begin(); brick != config_.end(); brick++)
    {
        if(brick.name() == brick_name)
        {
            x = (*brick)["x"].asInt();
            y = (*brick)["y"].asInt();
            z = (*brick)["z"].asInt();
            ori = (*brick)["ori"].asInt();
            break;
        }
    }
    return;
}

Pose Lego::get_init_brick_pose(const std::string& brick_name)
{
    Pose brick_pose;
    brick_pose.position = Vector3d(brick_map_[brick_name].x,
                                   brick_map_[brick_name].y,
                                   brick_map_[brick_name].z);
    brick_pose.orientation.x() = brick_map_[brick_name].quat_x;
    brick_pose.orientation.y() = brick_map_[brick_name].quat_y;
    brick_pose.orientation.z() = brick_map_[brick_name].quat_z;
    brick_pose.orientation.w() = brick_map_[brick_name].quat_w;
    brick_pose.orientation.normalize();
    return brick_pose;
}

Pose Lego::get_curr_brick_pose(const std::string& brick_name)
{
    Pose brick_pose;
    brick_pose.position = Vector3d(brick_map_[brick_name].cur_x,
                                   brick_map_[brick_name].cur_y,
                                   brick_map_[brick_name].cur_z);
    brick_pose.orientation = brick_map_[brick_name].cur_quat;
    brick_pose.orientation.normalize();
    return brick_pose;
}

Pose Lego::get_table_pose()
{
    Pose table_pose;
    double avg_x = 0.0;
    if (!robots_.empty())
    {
        for (const auto& robot : robots_)
        {
            avg_x += robot.base_frame(0, 3);
        }
        avg_x /= static_cast<double>(robots_.size());
    }
    table_pose.position.x() = avg_x;
    table_pose.position.y() = 0;
    table_pose.position.z() = storage_plate_.pose(2, 3);
    table_pose.orientation = Quaterniond::Identity();
    return table_pose;
}

bool Lego::is_press_pt_in_bound(const std::string& brick_name, int press_side, int press_offset)
{
    int height = brick_map_[brick_name].height;
    int width = brick_map_[brick_name].width;
    if (press_side == 1)
    {
        if (press_offset < 0 || press_offset >= width)
        {
            return false;
        }
    }
    else if (press_side == 2)
    {
        if (press_offset < 0 || press_offset >= height)
        {
            return false;
        }
    }
    else if (press_side == 3)
    {
        if (press_offset < 0 || press_offset >= height)
        {
            return false;
        }
    }
    else if (press_side == 4)
    {
        if (press_offset < 0 || press_offset >= width)
        {
            return false;
        }
    }
    return true;
}

void Lego::get_brick_sizes(const std::string& brick_name, double& x, double &y, double& z)
{
    x = brick_map_[brick_name].height * P_len_ - 0.0002;
    y = brick_map_[brick_name].width * P_len_ - 0.0002;
    z = brick_height_m_;
}

void Lego::get_brick_sizes_by_type(const int& id, int & height, int & width)
{
    height = lego_library_[std::to_string(id)]["height"].asInt();
    width = lego_library_[std::to_string(id)]["width"].asInt();
}

void Lego::get_table_size(double& x, double& y, double& z)
{
    x = table_width_;
    y = table_length_;
    z = 0.02;
}

void Lego::calc_bric_asssemble_pose(const std::string &name, const int& brick_loc_x,
                            const int& brick_loc_y, const int& brick_loc_z, const int& orientation,
                            Matrix4dRef out_pose)
{
    lego_brick l_brick = brick_map_[name];
    calc_brick_loc(l_brick, assemble_plate_, orientation, brick_loc_x, brick_loc_y, brick_loc_z, out_pose);

    Matrix4d y_180;
    y_180 << -1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, -1, 0,
             0, 0, 0, 1;

    out_pose = out_pose * y_180;
}

std::vector<std::vector<std::vector<std::string>>> Lego::gen_world_grid_from_graph(const Json::Value& task_json, int task_idx, int wx, int wy, int wz) {
    std::vector<std::vector<std::vector<std::string>>> world_grid = 
        std::vector<std::vector<std::vector<std::string>>>(wx, std::vector<std::vector<std::string>>(wy, std::vector<std::string>(wz, "")));
    
    // Seed the world with only the bricks present in the environment setup JSON.
    // Note: brick_map_ may contain additional "virtual" bricks created for tasks (e.g., station reuse),
    // which do not have entries in config_ and should not be treated as occupying space at t=0.
    for (auto brick = config_.begin(); brick != config_.end(); ++brick)
    {
        const std::string brick_name = brick.name();
        if (brick_name.empty() || brick_name.front() != 'b')
        {
            continue;
        }
        const int x = (*brick)["x"].asInt();
        const int y = (*brick)["y"].asInt();
        const int z = (*brick)["z"].asInt();
        const int ori = (*brick)["ori"].asInt();

        int h = 0;
        int w = 0;
        brick_dimension_from_name(brick_name, h, w);

        if (ori == 1)
        {
            std::swap(h, w);
        }
        if (z <= 0 || z > wz)
        {
            throw std::runtime_error("env_setup brick z out of bounds for '" + brick_name + "'");
        }
        if (x < 0 || y < 0 || x + h > wx || y + w > wy)
        {
            throw std::runtime_error("env_setup brick xy out of bounds for '" + brick_name + "'");
        }
        for (int i = x; i < x + h; i++)
        {
            for (int j = y; j < y + w; j++)
            {
                world_grid[i][j][z - 1] = brick_name;
            }
        }
    }

    for (int i = 0; i < task_idx; i++) {
        auto cur_graph_node = task_json[std::to_string(i+1)];
        int brick_id = cur_graph_node["brick_id"].asInt();
        int brick_x = cur_graph_node["x"].asInt();
        int brick_y = cur_graph_node["y"].asInt();
        int brick_z = cur_graph_node["z"].asInt() - 1;
        std::string brick_seq;
        if (cur_graph_node.isMember("brick_seq")) {
            brick_seq = cur_graph_node["brick_seq"].asString();
        } else {
            brick_seq = "t" + std::to_string(i+1);
        }
        int ori = cur_graph_node["ori"].asInt();
        int height, width;
        get_brick_sizes_by_type(brick_id, height, width);

        if (ori == 1) {
            std::swap(height, width);
        }
        if (brick_z < 0 || brick_z >= wz)
        {
            throw std::runtime_error("task brick z out of bounds at idx " + std::to_string(i + 1));
        }
        if (brick_x < 0 || brick_y < 0 || brick_x + height > wx || brick_y + width > wy)
        {
            throw std::runtime_error("task brick xy out of bounds at idx " + std::to_string(i + 1));
        }
        for (int x = brick_x; x < brick_x + height; x++)
        {
            for (int y = brick_y; y < brick_y + width; y++)
            {
                world_grid[x][y][brick_z] = "b" + std::to_string(brick_id) + "_" + brick_seq;
            }
        }
    }
    return world_grid;
}

void Lego::get_press_pt(int brick_x, int brick_y, int brick_type, int brick_ori, int press_side, int press_offset,
                    int &press_pt_x, int &press_pt_y, int &press_ori) {
    
    int height, width;
    get_brick_sizes_by_type(brick_type, height, width);
    if (brick_ori == 0) {
        if(press_side == 1) {
            press_pt_x = brick_x;
            press_pt_y = brick_y + press_offset;
            press_ori = 0;
        }
        else if(press_side == 2) {
            press_ori = 1;
            press_pt_x = brick_x + press_offset;
            press_pt_y = brick_y + width - 1;
        }
        else if(press_side == 3) {
            press_ori = 1;
            press_pt_x = brick_x + press_offset;
            press_pt_y = brick_y;
        }
        else if(press_side == 4) {
            press_ori = 0;
            press_pt_x = brick_x + height - 1;
            press_pt_y = brick_y + press_offset;
        }
    }
    else if(brick_ori == 1) {
        std::swap(height, width);
        if(press_side == 1) {
            press_ori = 1;
            press_pt_x = brick_x + height - 2 - press_offset;
            press_pt_y = brick_y;
        }
        else if(press_side == 2) {
            press_ori = 0;
            press_pt_x = brick_x;
            press_pt_y = brick_y + press_offset;
        }
        else if(press_side == 3) {
            press_ori = 0;
            press_pt_x = brick_x + height - 1;
            press_pt_y = brick_y + press_offset;
        }
        else if(press_side == 4) {
            press_ori = 1;
            press_pt_x = brick_x + height - 2 - press_offset;
            press_pt_y = brick_y + width - 1;
        }
    }
}


void Lego::get_sup_side_ori(int support_ori, int &sup_press_side, int &sup_brick_ori) {
    if (support_ori == 0) {
        sup_press_side = 1;
        sup_brick_ori = 0;
    }
    if (support_ori == 1) {
        sup_press_side = 4;
        sup_brick_ori = 1;
    }
    if (support_ori == 2) {
        sup_press_side = 1;
        sup_brick_ori = 1;
    }
    if (support_ori == 3) {
        sup_press_side = 4;
        sup_brick_ori = 0;
    }
}


void Lego::get_lego_twist_next(const Json::Value &task_json, int task_idx, const std::string &brick_name, std::vector<std::string> &side_bricks) {
    std::vector<std::vector<std::vector<std::string>>> world_grid = gen_world_grid_from_graph(task_json, task_idx, 48, 48, 48);

    auto &cur_node = task_json[std::to_string(task_idx)];
    int press_x, press_y, brick_z, press_ori;
    int press_side = cur_node["press_side"].asInt();
    int press_offset = cur_node["press_offset"].asInt();
    int brick_type = cur_node["brick_id"].asInt();
    int brick_ori = cur_node["ori"].asInt();
    if (cur_node.isMember("press_x")) {
        press_x = cur_node["press_x"].asInt();
        press_y = cur_node["press_y"].asInt();
        brick_z = cur_node["z"].asInt();
        press_ori = cur_node["ori"].asInt();
    } else {
        int x = cur_node["x"].asInt();
        int y = cur_node["y"].asInt();
        brick_z = cur_node["z"].asInt();
        get_press_pt(x, y, brick_type, brick_ori, press_side, press_offset, press_x, press_y, press_ori);
    }

    get_lego_next(press_x, press_y, brick_z, press_side, brick_ori, brick_type, brick_name, world_grid, side_bricks);
}

void Lego::get_lego_around(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, const std::string &brick_name,
    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &side_bricks) {
    side_bricks.clear();

    if (world_grid.empty() || world_grid[0].empty() || world_grid[0][0].empty()) {
        return;
    }

    int height, width;
    get_brick_sizes_by_type(brick_type, height, width);
    if (brick_ori == 1) {
        std::swap(height, width);
    }

    const int max_x = static_cast<int>(world_grid.size());
    const int max_y = static_cast<int>(world_grid[0].size());
    const int max_z = static_cast<int>(world_grid[0][0].size());
    const int z_idx = brick_z - 1;
    if (z_idx < 0 || z_idx >= max_z) {
        return;
    }

    for (int i = brick_x - 1; i <= brick_x + height; i++) {
        for (int j = brick_y - 1; j <= brick_y + width; j++) {
            const bool inside_x = (i >= brick_x && i < brick_x + height);
            const bool inside_y = (j >= brick_y && j < brick_y + width);
            if (inside_x && inside_y) {
                continue;
            }
            if (i <= 0 || i >= max_x || j <= 0 || j >= max_y) {
                continue;
            }
            const std::string &neighbor = world_grid[i][j][z_idx];
            if (!neighbor.empty() && neighbor != brick_name) {
                side_bricks.push_back(neighbor);
            }
        }
    }

    std::sort(side_bricks.begin(), side_bricks.end());
    side_bricks.erase(std::unique(side_bricks.begin(), side_bricks.end()), side_bricks.end());
}

void Lego::get_lego_next(int press_x, int press_y, int brick_z, int press_side, int brick_ori, int brick_type, const std::string &brick_name,
        const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &side_bricks) {
    
    side_bricks.clear();

    int side_x1, side_y1, side_x2, side_y2, side_x3, side_y3, side_x4, side_y4,
        side_x5, side_y5, side_x6, side_y6, side_x7, side_y7, side_x8, side_y8,
        side_x9, side_y9, side_x10, side_y10;
    //* 1, 2 the brick infront of EOAT
    // * 3, 4 the brick on the side of EOAT
    // * 5, 6 the brick on the side-back of EOAT
    // * 7, 8 the brick on the side-back-back of EOAT
    // * 9, 10 the brick on the side-front of EOAT
    int height, width;
    get_brick_sizes_by_type(brick_type, height, width);

    if (brick_ori == 1) {
        std::swap(height, width);
    }

    if ((brick_ori == 0 && press_side == 1) || (brick_ori == 1 && press_side == 2)) {
        side_x1 = press_x + 1;
        side_y1 = press_y;
        side_x2 = press_x + 1;
        side_y2 = press_y + 1;
        side_x3 = press_x;
        side_y3 = press_y - 1;
        side_x4 = press_x;
        side_y4 = press_y + 2;
        side_x5 = press_x - 1;
        side_y5 = press_y - 1;
        side_x6 = press_x - 1;
        side_y6 = press_y + 2;
        side_x7 = press_x - 2;
        side_y7 = press_y - 1;
        side_x8 = press_x - 2;
        side_y8 = press_y + 2;
        side_x9 = press_x + 1;
        side_y9 = press_y - 1;
        side_x10 = press_x + 1;
        side_y10 = press_y + 2;
    }
    else if((brick_ori == 0 && press_side == 2) || (brick_ori == 1 && press_side == 4)) {
        side_x1 = press_x;
        side_y1 = press_y - 1;
        side_x2 = press_x + 1;
        side_y2 = press_y - 1;
        side_x3 = press_x - 1;
        side_y3 = press_y;
        side_x4 = press_x + 2;
        side_y4 = press_y;
        side_x5 = press_x - 1;
        side_y5 = press_y + 1;
        side_x6 = press_x + 2;
        side_y6 = press_y + 1;
        side_x7 = press_x - 1;
        side_y7 = press_y + 2;
        side_x8 = press_x + 2;
        side_y8 = press_y + 2;
        side_x9 = press_x - 1;
        side_y9 = press_y - 1;
        side_x10 = press_x + 2;
        side_y10 = press_y - 1;

    }
    else if((brick_ori == 0 && press_side == 3) || (brick_ori == 1 && press_side == 1)) {
        side_x1 = press_x;
        side_y1 = press_y + 1;
        side_x2 = press_x + 1;
        side_y2 = press_y + 1;
        side_x3 = press_x - 1;
        side_y3 = press_y;
        side_x4 = press_x + 2;
        side_y4 = press_y;
        side_x5 = press_x - 1;
        side_y5 = press_y - 1;
        side_x6 = press_x + 2;
        side_y6 = press_y - 1;
        side_x7 = press_x - 1;
        side_y7 = press_y - 2;
        side_x8 = press_x + 2;
        side_y8 = press_y - 2;
        side_x9 = press_x - 1;
        side_y9 = press_y + 1;
        side_x10 = press_x + 2;
        side_y10 = press_y + 1;
    }
    else if((brick_ori == 0 && press_side == 4) || (brick_ori == 1 && press_side == 3)) {
        side_x1 = press_x - 1;
        side_y1 = press_y;
        side_x2 = press_x - 1;
        side_y2 = press_y + 1;
        side_x3 = press_x;
        side_y3 = press_y - 1;
        side_x4 = press_x;
        side_y4 = press_y + 2;
        side_x5 = press_x + 1;
        side_y5 = press_y - 1;
        side_x6 = press_x + 1;
        side_y6 = press_y + 2;
        side_x7 = press_x + 2;
        side_y7 = press_y - 1;
        side_x8 = press_x + 2;
        side_y8 = press_y + 2;
        side_x9 = press_x - 1;
        side_y9 = press_y - 1;
        side_x10 = press_x - 1;
        side_y10 = press_y + 2;
    }
    
    
    
    if (side_x1 > 0 && side_x1 < 48 && side_y1 > 0 && side_y1 < 48) {
        std::string side1 = world_grid[side_x1][side_y1][brick_z - 1];
        if (!side1.empty() && side1 != brick_name) {
            side_bricks.push_back(side1);
        }
    }
    if (side_x2 > 0 && side_x2 < 48 && side_y2 > 0 && side_y2 < 48) {
        std::string side2 = world_grid[side_x2][side_y2][brick_z - 1];
        if (!side2.empty() && side2 != brick_name) {
            side_bricks.push_back(side2);
        }
    }
    if (side_x3 > 0 && side_x3 < 48 && side_y3 > 0 && side_y3 < 48) {
        std::string side3 = world_grid[side_x3][side_y3][brick_z - 1];
        if (!side3.empty() && side3 != brick_name) {
            side_bricks.push_back(side3);
        }
    }
    if (side_x4 > 0 && side_x4 < 48 && side_y4 > 0 && side_y4 < 48) {
        std::string side4 = world_grid[side_x4][side_y4][brick_z - 1];
        if (!side4.empty() && side4 != brick_name) {
            side_bricks.push_back(side4);
        }
    }
    if (side_x5 > 0 && side_x5 < 48 && side_y5 > 0 && side_y5 < 48) {
        std::string side5 = world_grid[side_x5][side_y5][brick_z - 1];
        if (!side5.empty() && side5 != brick_name) {
            side_bricks.push_back(side5);
        }
    }
    if (side_x6 > 0 && side_x6 < 48 && side_y6 > 0 && side_y6 < 48) {
        std::string side6 = world_grid[side_x6][side_y6][brick_z - 1];
        if (!side6.empty() && side6 != brick_name) {
            side_bricks.push_back(side6);
        }
    }
    if (side_x7 > 0 && side_x7 < 48 && side_y7 > 0 && side_y7 < 48) {
        std::string side7 = world_grid[side_x7][side_y7][brick_z - 1];
        if (!side7.empty() && side7 != brick_name) {
            side_bricks.push_back(side7);
        }
    }
    if (side_x8 > 0 && side_x8 < 48 && side_y8 > 0 && side_y8 < 48) {
        std::string side8 = world_grid[side_x8][side_y8][brick_z - 1];
        if (!side8.empty() && side8 != brick_name) {
            side_bricks.push_back(side8);
        }
    }
    if (side_x9 > 0 && side_x9 < 48 && side_y9 > 0 && side_y9 < 48) {
        std::string side9 = world_grid[side_x9][side_y9][brick_z - 1];
        if (!side9.empty() && side9 != brick_name) {
            side_bricks.push_back(side9);
        }
    }
    if (side_x10 > 0 && side_x10 < 48 && side_y10 > 0 && side_y10 < 48) {
        std::string side10 = world_grid[side_x10][side_y10][brick_z - 1];
        if (!side10.empty() && side10 != brick_name) {
            side_bricks.push_back(side10);
        }
    }

    // remove duplicates
    std::sort(side_bricks.begin(), side_bricks.end());
    side_bricks.erase(std::unique(side_bricks.begin(), side_bricks.end()), side_bricks.end());

}

void Lego::get_lego_below(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, 
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &below_bricks) 
{
    below_bricks.clear();
    if (world_grid.empty() || world_grid.front().empty() || world_grid.front().front().empty())
    {
        return;
    }
    const int wx = static_cast<int>(world_grid.size());
    const int wy = static_cast<int>(world_grid.front().size());
    const int wz = static_cast<int>(world_grid.front().front().size());
    // brick_z is expected to be 1-indexed (as in task JSON); "below" is at (brick_z - 2).
    if (brick_z <= 1)
    {
        return;
    }
    const int below_layer = brick_z - 2;
    if (below_layer < 0 || below_layer >= wz)
    {
        return;
    }
    int height, width;
    get_brick_sizes_by_type(brick_type, height, width);

    if (brick_ori == 1) {
        std::swap(height, width);
    }

    for (int i = brick_x; i < brick_x + height; i++) {
        for (int j = brick_y; j < brick_y + width; j++) {
            if (i >= 0 && i < wx && j >= 0 && j < wy) {
                std::string below_brick = world_grid[i][j][below_layer];
                if (!below_brick.empty()) {
                    below_bricks.push_back(below_brick);
                }
            }
        }
    }

    // remove duplicates
    std::sort(below_bricks.begin(), below_bricks.end());
    below_bricks.erase(std::unique(below_bricks.begin(), below_bricks.end()), below_bricks.end());
}

void Lego::get_lego_above(int brick_x, int brick_y, int brick_z, int brick_ori, int brick_type, 
                    const std::vector<std::vector<std::vector<std::string>>>& world_grid, std::vector<std::string> &above_bricks) 
{
    above_bricks.clear();
    if (world_grid.empty() || world_grid.front().empty() || world_grid.front().front().empty())
    {
        return;
    }
    const int wx = static_cast<int>(world_grid.size());
    const int wy = static_cast<int>(world_grid.front().size());
    const int wz = static_cast<int>(world_grid.front().front().size());
    // brick_z is treated as a 0-indexed layer in world_grid (callers pass whatever layer they want to query).
    if (brick_z < 0 || brick_z >= wz)
    {
        return;
    }
    int height, width;
    get_brick_sizes_by_type(brick_type, height, width);

    if (brick_ori == 1) {
        std::swap(height, width);
    }

    for (int i = brick_x; i < brick_x + height; i++) {
        for (int j = brick_y; j < brick_y + width; j++) {
            if (i >= 0 && i < wx && j >= 0 && j < wy) {
                std::string above_brick = world_grid[i][j][brick_z];
                if (!above_brick.empty()) {
                    above_bricks.push_back(above_brick);
                }
            }
        }
    }

    // remove duplicates
    std::sort(above_bricks.begin(), above_bricks.end());
    above_bricks.erase(std::unique(above_bricks.begin(), above_bricks.end()), above_bricks.end());
}

}  // namespace lego
}  // namespace lego_manipulation
