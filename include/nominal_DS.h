#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>

#include <Eigen/Dense>

Eigen::Vector3d nominal_DS(Eigen::Matrix3d &rotation, Eigen::Matrix3d &gain, Eigen::Vector3d &current_end_effector, Eigen::Vector3d &attractor_main);