 
#include "TrackingControlNode.h"

 /**
  * @brief Constructor, will be called when an object is instantiated
 */
 TrackingControlNode::TrackingControlNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
 :nh_(*nh), nh_p_(*nh_p) {
 	ROS_INFO("in class constructor of TrackingControlNode");
 	loadParams();
 	initializeSubscribers();
 	initializePublishers();
 	initializeTimer();
	initializeServices();
 }

 /**
  * @brief Destructor
 */
 TrackingControlNode::~TrackingControlNode() {

 	// +.+ shutdown publishers
 	// ---> add publishers here
 	// Example: uref_pub.shutdown();
 
 	// +.+ shutdown subscribers
 	// ---> add subscribers here
 	// Example: state_sub.shutdown();
 	flag_sub_.shutdown();

 	// +.+ stop timer
 	timer_.stop();

 	// +.+ shutdown node
 	nh_.shutdown();


 }

 /**
  * @brief Initialize subcribers
 */
 void TrackingControlNode::initializeSubscribers() {
 	ROS_INFO("Initializing Subscribers for TrackingControlNode"); 

  	std::string flag_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/flag");
	/* These topic names later can be gotten from config file */
	
	// std::string state_topic_name =  "/mblack/nav/filter/state";
	std::string vehicle_state_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/vehicle_state");
	//std::string vc_topic_name =  "/mblack/slap/internal/vc"; 
	std::string cpf_info_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/vc_info"); 
	
	std::string st_curve_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/st_curve"); 


	state_sub_ = nh_.subscribe(vehicle_state_topic_name, 10, &TrackingControlNode::vehicleStateCallback, this);
	cpf_info_sub_ =  nh_.subscribe(cpf_info_topic_name, 10, &TrackingControlNode::cpf_infoCallback, this);
	st_curve_sub_ =  nh_.subscribe(st_curve_topic_name, 10, &TrackingControlNode::stCurveCallback, this);

	flag_sub_ = nh_.subscribe(flag_topic_name, 10, &TrackingControlNode::flagCallback, this);

 }

 /**
  * @brief Initialize publishers
 */
 void TrackingControlNode::initializePublishers() {
 	ROS_INFO("Initializing Publishers for TrackingControlNode"); 	

	/* These topics names later can be get from config file */  
	// std::string surge_topic_name =  "/mblack/ref/surge";
	// std::string yaw_rate_topic_name =  "/mblack/ref/yaw_rate";
	// std::string gamma_topic_name = "/mblack/slap/internal/gamma";
	// std::string st_curve_name = "/mblack/Virtual/State";


	std::string surge_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/surge");  
	std::string yaw_rate_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/yaw_rate");
	std::string gamma_dot_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/gamma_dot");


	/* Initialize the publisher */
	
  	surge_pub_ = nh_.advertise<std_msgs::Float64>(surge_topic_name, 1);
  	yaw_rate_pub_ = nh_.advertise<std_msgs::Float64>(yaw_rate_topic_name , 1);
  	gamma_dot_pub_ = nh_.advertise<std_msgs::Float64>(gamma_dot_topic_name, 1);


 }

 /**
  * @brief Initialize timer
 */
 void TrackingControlNode::initializeTimer() {
 	ROS_INFO("Initializing timer for TrackingControlNode"); 	// ---> add publishers here
 	timer_ =nh_.createTimer(ros::Duration(1.0/TrackingControlNode::nodeFrequency()), 
	 						&TrackingControlNode::timerIterCallback, this);
  	timer_.stop();
 }

 /**
	@brief Setup the node frequency: 10 hz
 */
 double TrackingControlNode::nodeFrequency()
 {
 	double node_frequency;
 	nh_p_.param("node_frequency", node_frequency, 10.0);
 	ROS_INFO("Node will run at : %lf [hz]", node_frequency);
 	return node_frequency; 
 }

 /*
#######################################################################################################################
 @.@ Load the parameters
 #######################################################################################################################
 */
void TrackingControlNode::loadParams() {
 	ROS_INFO("Load the TrackingControlNode parameters");
    /* get parameters of the tracking controller  */

	double delta, kx, ky, kz ;
    nh_p_.getParam("delta", delta);
	nh_p_.getParam("kx", kx);
	nh_p_.getParam("ky", ky);
	nh_p_.getParam("kz", kz);
	
	tracking_algorithm_.setTCGains(delta, kx, ky, kz);

 }


/**
 * @brief  Vehicle state callback to update the state of the vehicle
 *
 * @param msg  A pointer to a auv_msgs::NavigationStatus that contains
 * information regarding the vehicle
 */
void TrackingControlNode::vehicleStateCallback(const auv_msgs::NavigationStatus &msg) {
//   std::cout << "Tracking Node callback vehicle state" << std::endl;
  /* If the algorithm is running, signal that we have received data from the
   * vehicle state */
  if ( !received_vehicle_state ){
    received_vehicle_state = true;
	ROS_INFO("Tracking control node has received the first vehicle state"); 	

  }

  /* Update the vehicle position */
  		vehicle_state_.pos << msg.position.north, msg.position.east, msg.altitude;
  /* Update the vehicle orientation (converting from deg to rad) */
	   	double roll = FarolGimmicks::wrap2pi(msg.orientation.x * M_PI / 180, 1);
		double pitch = FarolGimmicks::wrap2pi(msg.orientation.y * M_PI / 180, 1);
		double yaw = FarolGimmicks::wrap2pi(msg.orientation.z * M_PI / 180, 1);
		vehicle_state_.orientation << roll, pitch, yaw;

  /* Update the vehicle linear velocity */
  		vehicle_state_.linear_vel << msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z;

  /* Update the vehicle angular velocity (converting from deg to rad) */
  		vehicle_state_.angular_vel << msg.orientation_rate.x * M_PI / 180, 
		  							  msg.orientation_rate.y * M_PI / 180, 
		  						      msg.orientation_rate.z * M_PI / 180;
}
void TrackingControlNode::cpf_infoCallback(const medusa_slap_msg::CPFinfo &msg){
  if (!received_vc) {
    received_vc = true;
	ROS_INFO("Tracking control node has received the first correction speed vc"); 	

  }
	st_curve_state_.vc = msg.vc;
}

void TrackingControlNode::stCurveCallback(const medusa_slap_msg::STCurve &msg){
	if(!received_st_curve){
		ROS_INFO("Tracking control node has received the first ST curve"); 	
		received_st_curve = true;
	}

		/* update target position */
	for(int i=0; i<3; i++){
		st_curve_state_.traj_pos[i] = msg.traj_pos[i];	
		st_curve_state_.traj_vel[i] = msg.traj_vel[i];
		st_curve_state_.path_pos[i] = msg.path_pos[i];
		st_curve_state_.path_vel_gamma[i] = msg.path_vel_gamma[i];	
	}

		st_curve_state_.vd_bar = msg.vd_bar;
}
 /*
#######################################################################################################################
 @.@ Iteration via timer callback
 #######################################################################################################################
 */
 void TrackingControlNode::timerIterCallback(const ros::TimerEvent &event) {
 	/* If not updated information from the vehicle and target pdf yet then do nothing	*/
	  if (!received_vehicle_state || !received_st_curve)
      return;	
  /* Update the vehicle, path state, and target state for the tracking control */
		tracking_algorithm_.updateVehicleState(vehicle_state_);
   		tracking_algorithm_.updateSTCurveState(st_curve_state_);      

  /* Get the difference between previous update time and current update time */
   		ros::Time curr_time = ros::Time::now();
   		ros::Duration dt = curr_time - prev_time_;
   		prev_time_ = curr_time;

	//	std::cout << "Sapling time: " << dt.toSec() << std::endl;   

  /* Compute the control law */
   		tracking_algorithm_.callTrackingControl(0.1);
  /* Publish the results to inner loop control */	
   		FarolGimmicks::publishValue<std_msgs::Float64, const double>(surge_pub_, tracking_algorithm_.getDesiredSurge());
   		FarolGimmicks::publishValue<std_msgs::Float64, const double>(yaw_rate_pub_, tracking_algorithm_.getDesiredYawRate());

  /* Publish the gamma_dot value */
   		FarolGimmicks::publishValue<std_msgs::Float64, const double>(gamma_dot_pub_,  tracking_algorithm_.getGammaDot());


//  	// ###################################################################################################################
//  	aux_bool.data = true; 	wp_status_timer_pub.publish(std_msgs::Bool(aux_bool));
 }
 /*
#######################################################################################################################
 @.@ Callback Flag
########################################################################################################################
 */
 void TrackingControlNode::wpStatusCallback(const std_msgs::Int8& msg) {

 	aux_int.data = 4;
 	if (msg.data != aux_int.data){
 		 ROS_INFO("Timer will Stop");
 		 timer_.stop();
 	} 	else{
 		 ROS_INFO("Timer will Start");
 		 timer_.start();
 	}

	std::cout << "flag message" << msg.data << std::endl; 
 	
 	if (msg.data != 6){
 		 ROS_INFO("Stop tracking");
 		 timer_.stop();
 	}	 
 
 }


 void TrackingControlNode::flagCallback(const std_msgs::Int8 &msg) {

	std::cout << "flag message" << msg.data << std::endl; 
	int Flag = msg.data;
  /* If the flag changed and the path following was running, then stop */
  if (Flag == 0 && timer_.hasStarted()) {
	 timer_.stop();
  /* Reset tracking controller*/
  	 tracking_algorithm_.reset();  
  }
}
 /*
#######################################################################################################################
 @.@ Main
 #######################################################################################################################
 */
 int main(int argc, char** argv)
 {
 	// +.+ ROS set-ups:
 	ros::init(argc, argv, "tracking_control_node"); //node name
 	// +.+ create a node handle; need to pass this to the class constructor
 	ros::NodeHandle nh;

 	ros::NodeHandle nh_p("~");

 	ROS_INFO("main: instantiating an object of type TrackingControlNode");

 	// +.+ instantiate an TrackingControlNode class object and pass in pointer to nodehandle for constructor to use
 	TrackingControlNode TrackingControl(&nh,&nh_p);

 	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
 	ros::spin();

 	return 0;
 }

