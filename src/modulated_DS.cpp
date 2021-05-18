#include "../include/modulated_DS.h"

Eigen::Vector3d modulated_DS(Eigen::Vector3d &attractor_main, const Eigen::Vector3d &object_position_init, Eigen::Vector3d &current_end_effector, double sigma){
    
    Eigen::Vector3d projection = object_position_init + (current_end_effector - object_position_init).dot(attractor_main - object_position_init)*(attractor_main - object_position_init)/(attractor_main - object_position_init).squaredNorm();
    
    double rbf_kernel = exp(-(current_end_effector - projection).squaredNorm()/(sigma * sigma));

    Eigen::Vector3d perp = (current_end_effector - object_position_init) - (current_end_effector - object_position_init).dot(attractor_main - object_position_init)*(attractor_main - object_position_init)/(attractor_main - object_position_init).squaredNorm();

    Eigen::Vector3d velocity_modulated;
    velocity_modulated = - rbf_kernel * perp;

    return velocity_modulated;
};