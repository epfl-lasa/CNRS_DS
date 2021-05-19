#include <iostream>
#include "../include/nominal_DS.h"
#include "../include/modulated_DS.h"
#include "../include/calculate_alpha.h"

int main(){

    /*
    Inputs to the DS
    1. Release position, release velocity

    Creates two attractors
    1. auxiliary attractor
    2. main attractor

    The release position is in between the two attractors and this allows for the DS to pass through the release position with a release velocity
    
    This is ofcourse subject to the gains of the linear DS used in the combination of the DSs.
    
    */ 


    /*
    EXCEPTION: The motion will not be nice if the end effector start on the line joining the two attractors - self explanatory
    */

    double sigma = 3.0;

    Eigen::Matrix3d gain_main;
    Eigen::Matrix3d gain_aux;

    Eigen::Matrix3d rotation;
    Eigen::Vector3d attractor_main;
    Eigen::Vector3d attractor_aux;

    gain_main << -0.2, 0.0, 0.0,
                0.0, -0.6, 0.0,
                0.0, 0.0, -0.9;

  
    gain_aux << -0.3, 0.0, 0.0,
                0.0, -0.3, 0.0,
                0.0, 0.0, -0.3;

    rotation << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;

    Eigen::Vector3d release_position = {0.2, 0.4, 0.4};   // Examples
    Eigen::Vector3d release_velocity = {0.2, 0.2, 0.2};   // Examples
    Eigen::Vector3d current_end_effector = {0.1, 0.3, 0.2};   // Examples
    Eigen::Vector3d end_effector_init = {0.1, 0.1, 0.1};

    attractor_main = release_position + 0.8*release_velocity/release_velocity.norm();
    attractor_aux = (4.0/3.0)*release_position - (1.0/3.0)*attractor_main;

    double alpha = calculate_alpha(current_end_effector, end_effector_init, release_position, attractor_main);


    Eigen::Vector3d velocity_nominal_main = nominal_DS(rotation, gain_main, current_end_effector, attractor_main);
    Eigen::Vector3d velocity_nominal_aux = nominal_DS(rotation, gain_aux, current_end_effector, attractor_aux);
    Eigen::Vector3d velocity_modulated = modulated_DS(attractor_main, release_position, current_end_effector, sigma);
    
    Eigen::Vector3d net_velocity = alpha*velocity_nominal_aux + (1 - alpha)*velocity_nominal_main + velocity_modulated;

    net_velocity = (net_velocity/net_velocity.norm()) * release_velocity.norm();

    std::cout << net_velocity << std::endl;


    return 0;
}