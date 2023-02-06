#include "CooperativeControlNode.h"
/**
 * @brief  Method to initialize all the services
 */
void CooperativeControlNode::initializeServices() {
  ROS_INFO("Initializing Services for CooperativeControlNode");

  /* Get the service names for starting and stoping the path following */
  std::string kc_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/cooperative_control_gain", "/cooperative_control_gain");
  std::string start_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/start_cooperative_control", "/start_cooperative_control");
  std::string stop_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/stop_cooperative_control", "/stop_cooperative_control");
  std::string set_etc_parameter_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/set_ETC_parameter", "/cpf/set_etc_parameter");

  /* Advertise the services with these names */
  kc_srv_= nh_.advertiseService(kc_srv_name, &CooperativeControlNode::setGainsService, this);
  start_srv_= nh_.advertiseService(start_srv_name, &CooperativeControlNode::startCooperativeControl, this);
  stop_srv_= nh_.advertiseService(stop_srv_name, &CooperativeControlNode::stopCooperativeControl, this);
  set_etc_parameter_srv_= nh_.advertiseService(set_etc_parameter_srv_name, &CooperativeControlNode::setETCParameter, this);

}
/* Start Service callback */
bool CooperativeControlNode::setGainsService(medusa_slap_msg::SetFloat64::Request &req, medusa_slap_msg::SetFloat64::Response &res) {
    double kc_ = req.data;
    cooperative_control_algorithm_.updateGains(kc_);
    ROS_INFO("Just updated gain");
    res.success = true;
  return true;
}

/* Start Service callback */
bool CooperativeControlNode::startCooperativeControl(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {
  if (timer_.hasStarted()) {
    ROS_INFO("cooperative controller already stated");
  }
  else{
    timer_.start();
    initial_time = ros::Time::now();
    ROS_INFO("just stated cooperative controller");
    res.success = true;
  }
  return true;
}

/* Stop Service callback */
bool CooperativeControlNode::stopCooperativeControl(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {

  /* Check if the timer was running or not */
  if (timer_.hasStarted()) {
    ROS_INFO("cooperative controller will stop.");
  } else {
    ROS_INFO("cooperative controller was not running.");
  }
  timer_.stop();
  /* Reset cooperative controller*/
  cooperative_control_algorithm_.reset();
    
  res.success = true;
  return true;
}

bool CooperativeControlNode::setETCParameter(medusa_slap_msg::SetETCParameter::Request &req, medusa_slap_msg::SetETCParameter::Response &res){

 
double c0 = req.c0;
double c1 = req.c1;
double c2 = req.c2;

cooperative_control_algorithm_.setETCParameters(c0, c1, c2);

ROS_INFO("update ETC paramter for cpf successfully");

res.success = true;

return true;
}


 

 