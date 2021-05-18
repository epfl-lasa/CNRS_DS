#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"


#include <Eigen/Dense>

double calculate_alpha(Eigen::Vector3d &end_effector_position, const Eigen::Vector3d &end_effector_init, const Eigen::Vector3d &object_position, Eigen::Vector3d &attractor_main);