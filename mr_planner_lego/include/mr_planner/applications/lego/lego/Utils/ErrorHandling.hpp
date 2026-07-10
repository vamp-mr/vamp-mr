// This file is derived from APEX-MR (https://github.com/intelligent-control-lab/APEX-MR),
// Copyright (c) 2025 Intelligent Control Lab, licensed under the MIT License.
// See THIRD_PARTY_LICENSES.md for the full license text.
// Modifications for VAMP-MR: ROS-free integration with the mr_planner_core planning engine.
//
// Error-reporting helper for the LEGO utility layer: the ERR_HEADER macro
// prefixes messages with the current source file name and line number.
#ifndef LEGO_UTILS_ERRORHANDLING_HPP
#define LEGO_UTILS_ERRORHANDLING_HPP

#include <string>

#define ERR_HEADER ("[" + std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/")+1) + ":" + std::to_string(uint(__LINE__)) + "] ")

#endif // LEGO_UTILS_ERRORHANDLING_HPP