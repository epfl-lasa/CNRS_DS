#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>

#include <Eigen/Dense>

Eigen::Vector3d modulated_DS(Eigen::Vector3d &attractor_main, const Eigen::Vector3d &object_position_init, Eigen::Vector3d &current_end_effector, double sigma);