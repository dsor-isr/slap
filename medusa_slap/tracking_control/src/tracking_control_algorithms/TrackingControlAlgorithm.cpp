#include "TrackingControlAlgorithm.h"

/* The constructor for the TrackingControlAlgorithm Path Following Controller */
TrackingControlAlgorithm::TrackingControlAlgorithm() {


}
/* Setter for the vehicle state structure */
void TrackingControlAlgorithm::updateVehicleState(VehicleState vehicle_state) {
  vehicle_state_ = vehicle_state;
}
 
/* Setter for the target state structure */
void TrackingControlAlgorithm::updateSTCurveState(STCurveState st_curve_state) {
  st_curve_state_ = st_curve_state;
}


 /* Method used to set controller gains - optional */
// bool TrackingControlAlgorithm::setPFGains(std::vector<double> gains) {

//   /* Handle the case where the number of gains received does not coincide with the
//    * number of gains of the algorithm
//    */
//   if(gains.size() != 6) {
//     return false;
//   }
  
//   /* Initiate the gains of the controller */
//   Delta_ << 1.0, 0.0, 0.0, -gains[0];
//   Delta_inv_ << 1.0, 0.0, 0.0, -1 / gains[0];
//   kk_ << gains[1], 0.0, 0.0, gains[2];
//   kz_ = gains[3];
//   epsilon_ << gains[0], 0.0;

//   /* Observer gains for the ocean currents */
//   k_pos_ = gains[4];
//   k_currents_ = gains[5];

//   return true; //gains where updated successfully
// }

/* Method that implements the tracking control law */
void TrackingControlAlgorithm::callTrackingControl(double dt) {

  /* Declare vectors to store information of the vehicle, the path, and the target state in 2D plane */   
  Eigen::Vector2d veh_pos;
  Eigen::Vector2d veh_vel;

  Eigen::Vector2d path_pos;
  Eigen::Vector2d path_vel_gamma;                                         // partial derivative respect to gamma

  Eigen::Vector2d traj_pos;
  Eigen::Vector2d traj_vel;


  /* Get the vehicle information */
  double vehicle_yaw = vehicle_state_.orientation[2];
  veh_pos << vehicle_state_.pos[0], vehicle_state_.pos[1];

  /* Get the path information */
  path_pos << st_curve_state_.path_pos[0], st_curve_state_.path_pos[1];                       
  path_vel_gamma << st_curve_state_.path_vel_gamma[0], st_curve_state_.path_vel_gamma[1];               // partial derivative respect to gamma
  /* Get the target information */
  traj_pos << st_curve_state_.traj_pos[0], st_curve_state_.traj_pos[1];
  traj_vel << st_curve_state_.traj_vel[0], st_curve_state_.traj_vel[1];

//  std::cout << "target trajectory: " << traj_pos <<  std::endl;
//  std::cout << "target velocity: " << traj_vel <<  std::endl;

  /* Compute the rotation matrix from the Body to the initial */
  Eigen::Matrix2d R_I_B;                                                    // rotation matrix from I  to B
  R_I_B << cos(vehicle_yaw), sin(vehicle_yaw), 
          -sin(vehicle_yaw), cos(vehicle_yaw);
 
  /* Compute the position error between the vehicle and the S-T curve in the Body frame */
  Eigen::Vector2d pos_error = R_I_B * (veh_pos - path_pos - traj_pos) - epsilon_;
  Eigen::Vector2d pos_error_tanh;
  pos_error_tanh << tanh(pos_error[0]), tanh(pos_error[1]);                 // just to saturate the error - it is optional

  /* Compute total desired angular speed */
   
  double omega_d = st_curve_state_.vd_bar + st_curve_state_.vc ;

  // std::cout << "vd_bar is: " << st_curve_state_.vd_bar <<  std::endl;
  // std::cout << "vc is: " << st_curve_state_.vc <<  std::endl;


  /* Compute the velocity vector of the S-T curve */

  Eigen::Vector2d  pd_dot = path_vel_gamma*omega_d + traj_vel;

  /* Trajectory tracking control law - a vector of desired surge and yaw_rate */
  if (gain_scale_ <=1){ 
      gain_scale_ += 0.005;      // just a technique to increase the gains of the controller, avoid vehicle over-reacts when it start
      pd_dot = path_vel_gamma*omega_d;
  }
  Eigen::Vector2d u = Delta_inv_ * (R_I_B*pd_dot - Kp_*pos_error_tanh);
  // std::cout << "u is: " << u <<  std::endl;

  /* get desired surge speed from u */
  double surge_min = 0;
  double surge_max = 1.5;
  desired_surge_ = std::max(std::min (u[0],surge_max), surge_min) ;

  /* get desired yaw_rate from u and converge it to deg/s */

  desired_yaw_rate_ = gain_scale_*u[1] * 180.0 / M_PI;

  /* Compute the control law for the evolution of the referene point on the path */
  std::string controller_type_ = "TypeII";
  if (!controller_type_.compare("TypeI")){
    gamma_dot_ = omega_d;
  }
  else
  {
    double gamma_dot_error = gamma_dot_ - omega_d;
    gamma_ddot_ = -kz_ * gamma_dot_error + pos_error.transpose() * R_I_B * path_vel_gamma;
        
    gamma_dot_ = std::min(std::max(gamma_dot_, -0.01), 0.01);
  //  gamma_ddot_ = std::min(std::max(gamma_dot_, -0.01), 0.01);
    gamma_dot_ += gamma_ddot_ * dt;

  }

  /* Virtual Target Control Law */
  double gamma_dot_max = 0.1;
  double gamma_dot_min = 0;
  gamma_dot_ = std::min(std::max(gamma_dot_, gamma_dot_min), gamma_dot_max);
    
  // std::cout << "gamma_dot is: " << gamma_dot_ <<  std::endl;
}

/* Method that will run in the first iteration of the algorithm */
// void TrackingControlAlgorithm::start() {
  
//   /* Publish the initial virtual re value to get the data from the path */
//  // MedusaGimmicks::publishValue<std_msgs::Float64, const double>(rabbit_pub_, gamma_);
// }

/* Method used to check whether we reached the end of the algorithm or not */
// bool TrackingControlAlgorithm::stop() {

//   /**
//    * Check if the gamma is greater then the gamma max of the path 
//    * If so, we have reached the end
//    */
//  // if(gamma_ >= path_state_.gamma_max) return true;

//   return false;
// }

/* Method to reset all the algorithm data when the path following restarts */
bool TrackingControlAlgorithm::reset() {
  
  /* Reset the velocity references */
  desired_surge_ = 0.0;
  desired_yaw_rate_ = 0.0;
  gain_scale_ = 0;

  /* Reset the virtual target */
  gamma_ddot_ = 0.0;
  gamma_dot_ = 0.0;

  return true;
}

void TrackingControlAlgorithm::setTCGains(double delta, double kx, double ky, double kz){

/* These gains later can be put in a config file */
  std::cout << delta << std::endl;
  std::cout << kx << std::endl;
  std::cout << ky << std::endl;
  std::cout << kz << std::endl;
  
  Delta_ << 1.0, 0.0, 0.0, -delta;
  Delta_inv_ << 1.0, 0.0, 0.0, -1 /delta;
  Kp_ << kx, 0.0, 0.0, ky;
  kz_ = kz;
  epsilon_ << delta, 0.0;
}

double TrackingControlAlgorithm::getDesiredSurge(){
  return desired_surge_;
} 
double TrackingControlAlgorithm::getDesiredYawRate()
{
  return desired_yaw_rate_;
}
double TrackingControlAlgorithm::getGammaDot(){
  return gamma_dot_;
}