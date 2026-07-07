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
