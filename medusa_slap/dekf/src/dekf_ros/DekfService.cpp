#include "DekfNode.h"
/**
 * @brief  Method to initialize all the services
 */
void DekfNode::initializeServices() {
  ROS_INFO("Initializing Services for DekfNode");

  /* Get the service names for starting and stoping the path following */
  std::string start_ekf_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/start_ekf", "/start_ekf");
  std::string stop_ekf_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/stop_ekf", "/stop_ekf");
  std::string reset_ekf_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/reset_ekf", "/reset_ekf");

  std::string start_dekf_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/start_dekf", "/start_dekf");
  std::string stop_dekf_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/stop_dekf", "/stop_dekf");

  std::string set_target_pdf_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/set_target_pdf", "/set_target_pdf");

  std::string set_matrices_QR_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/set_matrices_QR", "/set_matrice_QR");

  std::string set_ETC_parameter_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/set_ETC_parameter", "/set_ETC_parameter");

  std::string set_target_depth_srv_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/services/set_target_depth", "/set_target_depth");


  /* Advertise the services with these names */
  start_dekf_srv_= nh_.advertiseService(start_dekf_srv_name, &DekfNode::startDekf, this);
  stop_dekf_srv_ = nh_.advertiseService(stop_dekf_srv_name, &DekfNode::stopDekf, this);


  start_ekf_srv_= nh_.advertiseService(start_ekf_srv_name, &DekfNode::startEkf, this);
  stop_ekf_srv_ = nh_.advertiseService(stop_ekf_srv_name, &DekfNode::stopEkf, this);
  reset_ekf_srv_ = nh_.advertiseService(reset_ekf_srv_name, &DekfNode::resetEkf, this);


  set_target_pdf_srv_ = nh_.advertiseService(set_target_pdf_srv_name, &DekfNode::setTargetPDF, this);


  set_matrices_QR_srv_= nh_.advertiseService(set_matrices_QR_srv_name, &DekfNode::setMatricesQR, this);

  set_ETC_parameter_srv_ = nh_.advertiseService(set_ETC_parameter_srv_name, &DekfNode::setETCParameter, this);

  set_target_depth_srv_ = nh_.advertiseService(set_target_depth_srv_name, &DekfNode::setTargetDepth, this);

}

/* Start Service callback */
bool DekfNode::startEkf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {
    
  if (ekf_enable_) {
    ROS_INFO("EKF already started.");
  } else {
    ekf_enable_ = true;
    ROS_INFO("Just started EKF.");
    res.success = true;
  }
  return true;
}

/* Stop Service callback */
bool DekfNode::stopEkf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {

  /* Check if the timer was running or not */
  if (ekf_enable_) {
    ROS_INFO("Just stoped EKF.");
  } else {
    ROS_INFO("Ekf was not running.");
      res.success = true;
  }
  /* stop Ekf */
  ekf_enable_ = false;    // 
    
  return true;
}

/* Start Service callback */
bool DekfNode::startDekf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {

  if (dekf_enable_) {
    ROS_INFO("DEKF mode already started");
  } else {
    ROS_INFO("Just started DEKF mode.");
  //  dekf_algorithm_.setDekfMode(true);  
    dekf_enable_ = true;
    res.success = true;

  }
  return true;

}

/* Stop Service callback */
bool DekfNode::stopDekf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {
  if (!dekf_enable_) {
    ROS_INFO("Was not in DEKF mode");
  } else {
    ROS_INFO("Just stoped DEKF mode.");
  //  dekf_algorithm_.setDekfMode(false);
    dekf_enable_ = false;
    res.success = true;
  }
  return true;
} 

bool DekfNode::resetEkf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res) {
  dekf_algorithm_.setLocalTargetPDF(target_pdf_local_init_);
  smoothed_target_state_ =  target_pdf_local_init_.state;
  res.success = true;
  ROS_INFO("Reset Target Distribution.");

  return true;
} 



bool DekfNode::setTargetPDF(medusa_slap_msg::SetTargetPDF::Request &req, medusa_slap_msg::SetTargetPDF::Response &res){

Eigen::Vector4d relative_state;
double pos_x = req.pos_x;
double pos_y = req.pos_y;
double vel_x = req.vel_x;
double vel_y = req.vel_y;
relative_state << pos_x,
                  pos_y,
                  vel_x,
                  vel_y;
double var_pos_x = req.var_pos_x;
double var_pos_y = req.var_pos_y;
double var_vel_x = req.var_vel_x;
double var_vel_y = req.var_vel_y;

TargetPDF target_pdf;
target_pdf.state = relative_state;

target_pdf.cov.setZero(4,4); 

target_pdf.cov.diagonal() << var_pos_x, var_pos_y, var_vel_x, var_vel_y;


dekf_algorithm_.setLocalTargetPDF(target_pdf); 			

res.success = true;

ROS_INFO(" Set target PDF successfully");

return true;
}

bool DekfNode::setMatricesQR(medusa_slap_msg::SetMatricesQR::Request &req, medusa_slap_msg::SetMatricesQR::Response &res){

Eigen::Matrix4d Q_;
Eigen::MatrixXd R_;
R_.resize(1,1);
double Q11 = req.Q11;
double Q22 = req.Q22;
double Q33 = req.Q33;
double Q44 = req.Q44;
double R11 = req.R;
Q_.diagonal() << Q11, Q22, Q33, Q44;
R_ <<  R11;

dekf_algorithm_.setMatricesWV(Q_,R_);

ROS_INFO("update matrices Q, R successfully");

res.success = true;

return true;
}

bool DekfNode::setETCParameter(medusa_slap_msg::SetETCParameter::Request &req, medusa_slap_msg::SetETCParameter::Response &res){

 
double c0 = req.c0;
double c1 = req.c1;
double c2 = req.c2;

dekf_algorithm_.setETCParameters(c0, c1, c2);

ROS_INFO("update matrices ETC paramter successfully");

res.success = true;

return true;
}


bool DekfNode::setTargetDepth(medusa_slap_msg::SetFloat64::Request &req, medusa_slap_msg::SetFloat64::Response &res){
 
double target_depth = req.data;

dekf_algorithm_.setTargetDepth(target_depth);

ROS_INFO("update target depth successfully");

res.success = true;

return true;
}