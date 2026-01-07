/*
***********************************************************************************************************************************************************************
Copyright notice for IP Docket # 2022-013, The CFS Library in C++ for Robot Arm Trajectory Planning 2020 Carnegie Mellon University. 
All rights reserved.
National Robotics Engineering Center, Carnegie Mellon University www.nrec.ri.cmu.edu
Confidential and Proprietary - Do not distribute without prior written permission.

Rui Chen      ruic3@andrew.cmu.edu
Ruixuan Liu   ruixuanl@andrew.cmu.edu
Weiye Zhao    weiyezha@andrew.cmu.edu
Changliu Liu  cliu6@andrew.cmu.edu

A023793. Sponsor is provided a non-exclusive, world-wide license to the Licensed Technology in the field of robotic painting and arcwelding.

This notice must appear in all copies of this file and its derivatives.
***********************************************************************************************************************************************************************
*/
#ifndef LEGO_UTILS_MATH_HPP
#define LEGO_UTILS_MATH_HPP

#include "mr_planner/applications/lego/lego/Utils/Common.hpp"
#include "mr_planner/applications/lego/lego/Utils/ErrorHandling.hpp"

#define CARTESIAN_DIMS 6
#define TRANS_DIMS 3
#define AXIS_DIMS 3
#define ORIENTATION_DIMS 3
#define DEG2RAD(angle) (static_cast<double>(angle)*M_PI/180.0)
#define RAD2DEG(angle) (static_cast<double>(angle)/M_PI*180.0)

#define N_JOINTS 6 // ruic: for now 6 joints

namespace lego_manipulation
{

using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Quaterniond = Eigen::Quaternion<double>;
using Matrix4dRef = Eigen::Ref<Matrix4d>;
using Matrix4dConstRef = Eigen::Ref<const Matrix4d>;

struct Pose {
    Vector3d position;
    Quaterniond orientation;

    Pose()
        : position(Vector3d::Zero()),
          orientation(Quaterniond::Identity()) {}

    Pose(const Vector3d& pos, const Quaterniond& ori)
        : position(pos),
          orientation(ori) {}

    static Pose FromMatrix(Matrix4dConstRef mat) {
        Pose pose;
        pose.position = mat.block<3, 1>(0, 3);
        pose.orientation = Quaterniond(mat.block<3, 3>(0, 0));
        pose.orientation.normalize();
        return pose;
    }

    Matrix4d ToMatrix() const {
        Matrix4d mat = Matrix4d::Identity();
        mat.block<3, 3>(0, 0) = orientation.toRotationMatrix();
        mat.block<3, 1>(0, 3) = position;
        return mat;
    }
};

namespace math
{

/* -------------------------------------------------------------------------- */
/*                             Vector definitions                             */
/* -------------------------------------------------------------------------- */
using Vector6d = Eigen::Matrix<double, 6, 1>;
using VectorJd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

/* -------------------------------------------------------------------------- */
/*                                   Matrix                                   */
/* -------------------------------------------------------------------------- */
Eigen::MatrixXd PInv(const Eigen::MatrixXd& M);
Eigen::MatrixXd EigenVcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);
Eigen::MatrixXd EigenHcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);


/* -------------------------------------------------------------------------- */
/*                                 Kinematics                                 */
/* -------------------------------------------------------------------------- */

Eigen::MatrixXd FK(const VectorJd& q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad);

bool ApproxEqNum(const double& a, const double& b, const double& thres);

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data);

}
}

#endif // LEGO_UTILS_MATH_HPP
