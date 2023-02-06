/**
 * @brief this code is for trajectory planning algorithm
 * @author  DSOR team
 * @author  Persons in charges: Hung
 * @date    2021
 */


#include "TrajectoryPlanningAlgorithm.h"

/* The constructor for the TrajectoryPlanningAlgorithm  */
TrajectoryPlanningAlgorithm::TrajectoryPlanningAlgorithm() {
/* These gains later can be put in a config file */
 
}
/* Setter for the trajectory update */
void TrajectoryPlanningAlgorithm::updateTargetTrajectory(Eigen::Vector3d target_pos, 
                                                         Eigen::Vector3d target_vel) {
  target_pos_ = target_pos;
  target_vel_ = target_vel;
}
/* Setter for the path parameter */
void TrajectoryPlanningAlgorithm::updateGamma(double gamma){
  gamma_ = gamma;
}

void TrajectoryPlanningAlgorithm::callTrajectoryPlanningAlgorithm(double dt) {
    traj_pos_ = target_pos_;
    traj_vel_ = target_vel_;
    updatePath();
}   
void TrajectoryPlanningAlgorithm::updatePath(){
  if (pursuit_mode_ == "rotate"){
	  path_pos_  << rx_*cos(gamma_ + phi_), 					 // position of the referene point			                                    
                 	ry_*sin(gamma_ + phi_),
	  					    0.0 ;					 
    path_vel_gamma_ << -rx_*sin(gamma_ + phi_),             // partial derivative of r respect to gamma
                        ry_*cos(gamma_ + phi_),
	  					          0.0;
 //   std::cout << pursuit_mode_ << std::endl;
                     
  }    

  if (pursuit_mode_ == "fixed"){
  //  std::cout << pursuit_mode_ << std::endl;

    Eigen::Matrix2d rot;
    Eigen::Matrix2d rot_gamma;
    gamma_ = 0;
    Eigen::Vector2d target_vel_2D;

    rot <<        cos(gamma_ + phi_+ alpha_),  -sin(gamma_ + phi_+ alpha_),
                  sin(gamma_ + phi_+ alpha_),   cos(gamma_ + phi_+ alpha_);    

    rot_gamma << -sin(gamma_ + phi_+ alpha_),  -cos(gamma_ + phi_+ alpha_),
                  cos(gamma_ + phi_+ alpha_),  -sin(gamma_ + phi_+ alpha_);       

    target_vel_2D << target_vel_(0), target_vel_(1);
  
    Eigen::Vector2d normalized_target_vel;

    if (target_vel_2D.norm() == 0){
      normalized_target_vel << 0,0;
    }
    else{
      normalized_target_vel << target_vel_2D/target_vel_2D.norm();       
    } 
    Eigen::Vector2d tem1, tem2;
    tem1 = rot*normalized_target_vel;
    tem2 = rot_gamma*normalized_target_vel;
    path_pos_  << rx_*tem1(0), 					 // position of the referene point			                                    
                  ry_*tem1(1),
                  0.0;
     
    path_vel_gamma_ << 0,0,0;
}
}
void TrajectoryPlanningAlgorithm::updatePathParameter(double rx, double ry, double rz, double formation_offset)
{
  rx_ = rx;
  ry_ = ry;
  rz_ = rz;
  phi_ = formation_offset;          
}
/* Method that will run in the first iteration of the algorithm */
void TrajectoryPlanningAlgorithm::start() {
  
  /* Publish the initial virtual re value to get the data from the path */
 // MedusaGimmicks::publishValue<std_msgs::Float64, const double>(rabbit_pub_, gamma_);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool TrajectoryPlanningAlgorithm::stop() {

  /**
   * Check if the gamma is greater then the gamma max of the path 
   * If so, we have reached the end
   */
 // if(gamma_ >= path_state_.gamma_max) return true;

  return false;
}

Eigen::Vector3d TrajectoryPlanningAlgorithm::getTrajPos(){
  return traj_pos_;
} 
Eigen::Vector3d TrajectoryPlanningAlgorithm::getTrajVel()
{
  return traj_vel_;
}
Eigen::Vector3d TrajectoryPlanningAlgorithm::getPathPos(){
  return path_pos_;
}
Eigen::Vector3d TrajectoryPlanningAlgorithm::getPathVelGamma(){
  return path_vel_gamma_;
}

void TrajectoryPlanningAlgorithm::updateRotateTargetFormationParameter(double desired_distance){
  rx_ = desired_distance;
  ry_ = desired_distance;
  pursuit_mode_ = "rotate";
}
void TrajectoryPlanningAlgorithm::updateFixedTargetFormationParameter(double desired_distance, double desired_angle){
  rx_ = desired_distance;
  ry_ = desired_distance;
  alpha_ = desired_angle;
  pursuit_mode_ = "fixed";

}

