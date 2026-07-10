// This file is derived from APEX-MR (https://github.com/intelligent-control-lab/APEX-MR),
// Copyright (c) 2025 Intelligent Control Lab, licensed under the MIT License.
// See THIRD_PARTY_LICENSES.md for the full license text.
// Modifications for VAMP-MR: ROS-free integration with the mr_planner_core planning engine.
//
// Common includes for the LEGO utility layer: standard library, Eigen, and
// jsoncpp headers, plus the DEBUG_PRINT switch derived from the build type.
#ifndef LEGO_UTILS_COMMON_HPP
#define LEGO_UTILS_COMMON_HPP

#if BUILD_TYPE == BUILD_TYPE_DEBUG
    #define DEBUG_PRINT
#else
    #undef DEBUG_PRINT
#endif

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <cstdio>
#include <ctime>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <iterator>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <math.h>
#include <cmath>
#include <iomanip>
#include <map>
#include <utility>
#include <memory>
#include <chrono>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

using namespace std::chrono;

#endif // LEGO_UTILS_COMMON_HPP
