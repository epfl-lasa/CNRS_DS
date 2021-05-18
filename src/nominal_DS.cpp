#include "../include/nominal_DS.h"

Eigen::Vector3d nominal_DS(Eigen::Matrix3d &rotation, Eigen::Matrix3d &gain, Eigen::Vector3d &current_end_effector, Eigen::Vector3d &attractor){
    
    Eigen::Vector3d velocity_nominal = rotation*gain*rotation.transpose()*(current_end_effector - attractor);

    return velocity_nominal;
};