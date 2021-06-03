#include <iostream>

#include <cnrs_ds/nominal_DS.h>
#include <cnrs_ds/modulated_DS.h>
#include <cnrs_ds/calculate_alpha.h>

void orthonormalize(Eigen::Matrix3d& basis){
    assert(basis.rows() == basis.cols());
    uint dim = basis.rows();
    basis.col(0).normalize();
    for(uint i=1;i<dim;i++){
        for(uint j=0;j<i;j++)
            basis.col(i) -= basis.col(j).dot(basis.col(i))*basis.col(j);
        basis.col(i).normalize();
    }
}

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



    double sigma = 3.0;  // parameter for the rbf kernel in modulated DS

    Eigen::Matrix3d gain_main;  // simplified gains for main DS
    Eigen::Matrix3d gain_aux;  // simplified gains for main DS

    Eigen::Vector3d attractor_main;
    Eigen::Vector3d attractor_aux;

    gain_main << -0.2, 0.0, 0.0,
                0.0, -0.6, 0.0,
                0.0, 0.0, -0.9;


    gain_aux << -0.3, 0.0, 0.0,
                0.0, -0.3, 0.0,
                0.0, 0.0, -0.3;


    Eigen::Vector3d release_position = {0.2, 0.4, 0.4};   // Examples
    Eigen::Vector3d release_velocity = {0.2, 0.2, 0.2};   // Examples
    Eigen::Vector3d current_end_effector = {0.1, 0.3, 0.2};   // Examples
    Eigen::Vector3d end_effector_init = {0.1, 0.1, 0.1};

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Random(3,3); 
    rotation.block<3,1>(0,0) = 1.0/release_velocity.norm() * release_velocity;      
    orthonormalize(rotation);

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
