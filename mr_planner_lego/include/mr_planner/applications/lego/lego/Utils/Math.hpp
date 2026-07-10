// This file is derived from APEX-MR (https://github.com/intelligent-control-lab/APEX-MR),
// Copyright (c) 2025 Intelligent Control Lab, licensed under the MIT License.
// See THIRD_PARTY_LICENSES.md for the full license text.
// Modifications for VAMP-MR: ROS-free integration with the mr_planner_core planning engine.
//
// Math utilities for the LEGO utility layer: Eigen matrix/vector aliases, the
// Pose struct (position + quaternion with 4x4 conversions), angle-unit macros,
// and kinematics helpers (pseudo-inverse, matrix concatenation, DH-based FK).
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

#define N_JOINTS 6 // assumes 6-DOF arms

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
