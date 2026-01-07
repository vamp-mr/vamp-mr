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
#ifndef LEGO_UTILS_ERRORHANDLING_HPP
#define LEGO_UTILS_ERRORHANDLING_HPP

#include <string>

#define ERR_HEADER ("[" + std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/")+1) + ":" + std::to_string(uint(__LINE__)) + "] ")

#endif // LEGO_UTILS_ERRORHANDLING_HPP