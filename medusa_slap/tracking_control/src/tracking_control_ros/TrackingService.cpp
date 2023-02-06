#include "TrackingControlNode.h"
/**
 * @brief  Method to initialize all the services
 */
void TrackingControlNode::initializeServices() {
  ROS_INFO("Initializing Services for TrackingControlNode");

  /* Get the service names for starting and stoping the path following */
  std::string start_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/start_tracking", "/start_tracking");
  std::string stop_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/stop_tracking", "/stop_tracking");
  std::string set_tc_gains_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/set_tc_gains", "/set_tc_gains");

  /* Advertise the services with these names */
  start_tracking_srv_= nh_.advertiseService(start_srv_name, &TrackingControlNode::startTracking, this);
  stop_tracking_srv_ = nh_.advertiseService(stop_srv_name, &TrackingControlNode::stopTracking, this);
  set_tc_gains_srv_ = nh_.advertiseService(set_tc_gains_srv_name, &TrackingControlNode::setTCGains, this);

}

/* Start Service callback */
bool TrackingControlNode::startTracking(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {
  if (timer_.hasStarted()) {
    ROS_INFO("tracking controller already stated");
  }
  else{
    timer_.start();
    ROS_INFO("just stated tracking controller");
    res.success = true;
  }
  return true;
}

/* Stop Service callback */
bool TrackingControlNode::stopTracking(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {

  /* Check if the timer was running or not */
  if (timer_.hasStarted()) {
    ROS_INFO("tracking controller will stop.");
  } else {
    ROS_INFO("tracking controller was not running.");
  }
  timer_.stop();
  /* Reset tracking controller*/
  tracking_algorithm_.reset();
    
  res.success = true;
  return true;
}
bool TrackingControlNode::setTCGains(medusa_slap_msg::SetTCGains::Request &req, medusa_slap_msg::SetTCGains::Response &res) {
  double delta = req.delta;
  double kx = req.kx;
  double ky = req.ky;
  double kz = req.kz;

  tracking_algorithm_.setTCGains(delta,kx,ky,kz); 
  res.success = true;
  return true;
}
 