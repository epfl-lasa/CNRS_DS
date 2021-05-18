#include "../include/calculate_alpha.h"

double calculate_alpha(Eigen::Vector3d &end_effector_position, const Eigen::Vector3d &end_effector_init, const Eigen::Vector3d &object_position, Eigen::Vector3d &attractor_main){

    Eigen::Vector3d normal_vector = attractor_main - object_position;
    Eigen::Vector3d initial_vector = end_effector_init - object_position;
    Eigen::Vector3d current_vector = end_effector_position - object_position;

    Eigen::Vector3d initial_vector_projection = initial_vector - initial_vector.dot(normal_vector)*normal_vector / (normal_vector.squaredNorm());
    Eigen::Vector3d current_vector_projection = current_vector - current_vector.dot(normal_vector)*normal_vector / (normal_vector.squaredNorm());

    double initial_vector_norm = initial_vector_projection.norm();
    double current_vector_norm = current_vector_projection.norm();
    double alpha;

    if (initial_vector_norm == 0){
        alpha = 0;
    }
    else {
        alpha = current_vector_norm / initial_vector_norm;
    }

    return alpha;
};