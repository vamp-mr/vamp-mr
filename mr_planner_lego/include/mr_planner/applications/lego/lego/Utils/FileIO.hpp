// This file is derived from APEX-MR (https://github.com/intelligent-control-lab/APEX-MR),
// Copyright (c) 2025 Intelligent Control Lab, licensed under the MIT License.
// See THIRD_PARTY_LICENSES.md for the full license text.
// Modifications for VAMP-MR: ROS-free integration with the mr_planner_core planning engine.
//
// File I/O helpers for the LEGO utility layer: load/save Eigen matrices from/
// to plain-text files (used for DH parameters, base frames, and calibration).
#ifndef LEGO_UTILS_FILEIO_HPP
#define LEGO_UTILS_FILEIO_HPP

#include <mr_planner/applications/lego/lego/Utils/Math.hpp>
#include <mr_planner/applications/lego/lego/Utils/ErrorHandling.hpp>

namespace lego_manipulation
{
namespace io
{
Eigen::MatrixXd LoadMatFromFile(const std::string fname);
void SaveMatToFile(const Eigen::MatrixXd& mat, const std::string& fname);

}

}
#endif // LEGO_UTILS_FILEIO_HPP