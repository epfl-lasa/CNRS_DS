#include "../include/modulated_DS.h"

Eigen::Vector3d modulated_DS(Eigen::Vector3d &attractor_main, const Eigen::Vector3d &release_position, Eigen::Vector3d &current_end_effector, double sigma){
    
    Eigen::Vector3d projection = release_position + (current_end_effector - release_position).dot(attractor_main - release_position)*(attractor_main - release_position)/(attractor_main - release_position).squaredNorm();
    
    double rbf_kernel = exp(-(current_end_effector - projection).squaredNorm()/(sigma * sigma));

    Eigen::Vector3d perp = (current_end_effector - release_position) - (current_end_effector - release_position).dot(attractor_main - release_position)*(attractor_main - release_position)/(attractor_main - release_position).squaredNorm();

    Eigen::Vector3d velocity_modulated;
    velocity_modulated = - rbf_kernel * perp;

    return velocity_modulated;
};