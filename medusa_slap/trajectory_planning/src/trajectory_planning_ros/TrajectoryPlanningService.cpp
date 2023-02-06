#include "TrajectoryPlanningNode.h"
/**
 * @brief  Method to initialize all the services
 */
void TrajectoryPlanningNode::initializeServices() {
  ROS_INFO("Initializing Services for TrajectoryPlanningNode");

  /* Get the service names for starting and stoping the path following */
  std::string desired_nominal_gamma_dot_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/nominal_gamma_dot", "/nominal_gamma_dot");
  std::string fixed_target_mode_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/behind_target_mode", "/behind_target_mode");
  std::string rotate_target_mode_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/rotate_target_mode", "/rotate_target_mode");
  std::string set_gamma_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/set_gamma", "/set_gamma");

  /* Advertise the services with these names */
  desired_gamma_dot_srv_= nh_.advertiseService(desired_nominal_gamma_dot_srv_name, 
                                              &TrajectoryPlanningNode::updateDesiredGammaDot, this);
  fixed_target_mode_srv_= nh_.advertiseService(fixed_target_mode_srv_name, 
                                              &TrajectoryPlanningNode::setFixedTargetMode, this);   
  rotate_target_mode_srv_= nh_.advertiseService(rotate_target_mode_srv_name, 
                                              &TrajectoryPlanningNode::setRotateTargetMode, this);   
  set_gamma_srv_= nh_.advertiseService(set_gamma_srv_name, 
                                              &TrajectoryPlanningNode::setGamma, this);                                                                                                                                     
}

/* Start Service callback */
bool TrajectoryPlanningNode::updateDesiredGammaDot(medusa_slap_msg::SetFloat64::Request &req, medusa_slap_msg::SetFloat64::Response &res) {
    vd_bar_new_ = req.data;
    vd_bar_old_ = vd_bar_; 
    ROS_INFO("Just updated desired nominal speed profile for gamma_dot");
    res.success = true;
  return true;
}

bool TrajectoryPlanningNode::setFixedTargetMode(medusa_slap_msg::SetFixedTargetMode::Request &req, medusa_slap_msg::SetFixedTargetMode::Response &res) {
    double desired_angle = req.desired_angle*M_PI/180;
    double desired_distance = req.desired_distance;
    trajectory_planning_algorithm_.updateFixedTargetFormationParameter(desired_distance, desired_angle);
    ROS_INFO("Just switch the slap to fix target mode");
    res.success = true;
  return true;
} 
bool TrajectoryPlanningNode::setRotateTargetMode(medusa_slap_msg::SetRotateTargetMode::Request &req, medusa_slap_msg::SetRotateTargetMode::Response &res) {
    vd_bar_new_ = req.desired_angular_rate;
    vd_bar_old_ = vd_bar_; 
    double desired_distance = req.desired_distance;
    trajectory_planning_algorithm_.updateRotateTargetFormationParameter(desired_distance);
    ROS_INFO("Just switch slap to rotate target mode");
    res.success = true;
  return true;
} 
bool TrajectoryPlanningNode::setGamma(medusa_slap_msg::SetFloat64::Request &req, medusa_slap_msg::SetFloat64::Response &res) {
    gamma_ = req.data;
    ROS_INFO("Just set new gamma in trajectory planning mode");
    res.success = true;
  return true;
} 