/**
 * @brief this code is for trajectory planning node
 * @author  DSOR team
 * @author  Persons in charges: Hung
 * @date    2021
 */

 // this header incorporates all the necessary #include files and defines the class "TrajectoryPlanningNode"
 #include "TrajectoryPlanningNode.h"
 #include "TrajectoryPlanningAlgorithm.h"

 /**
  * @brief Constructor, will be called when an object is instantiated
 */
 TrajectoryPlanningNode::TrajectoryPlanningNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
                        :nh_(*nh), nh_p_(*nh_p) {
 	ROS_INFO("in class constructor of TrajectoryPlanningNode");
 	loadParams();
 	initializeSubscribers();
 	initializePublishers();
 	initializeTimer();
	initializeServices();


 }

 /**
  * @brief Destructor
 */
 TrajectoryPlanningNode::~TrajectoryPlanningNode() {
 	// +.+ stop timer
 //	timer_.stop();

 	// +.+ shutdown node
 	nh_.shutdown();
 }

 /**
  * @brief Initialize subscribers
  */
 void TrajectoryPlanningNode::initializeSubscribers() {
 	ROS_INFO("Initializing Subscribers for TrajectoryPlanningNode");
 	// ---> add subscribers here
 	// Example: state_sub = nh_.subscribe("State", 10, &TrajectoryPlanningNode::updateCallback,this);

 	flag_sub = nh_.subscribe("Flag",10, &TrajectoryPlanningNode::wpStatusCallback,this);

	
/* These topic names later can be gotten from config file */
	
	//std::string internal_gamma_topic_name =  "/mblack/slap/internal/gamma";
	std::string internal_gamma_dot_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_,"topics/subscribers/gamma_dot");
	// std::string targetPDF_topic_name =  "/mblack/slap/internal/target/pdf"; 
   	std::string targetPDF_absolute_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/target_pdf_absolute");  

	internal_gamma_dot_sub_ = nh_.subscribe(internal_gamma_dot_topic_name, 10, &TrajectoryPlanningNode::internalGammaDotCallback, this);
	target_pdf_sub_ = nh_.subscribe(targetPDF_absolute_topic_name, 10, &TrajectoryPlanningNode::targetPdfCallback, this);

  }

 /*
#######################################################################################################################
 @.@ Member helper function to set up publishers;
 #######################################################################################################################
 */
 void TrajectoryPlanningNode::initializePublishers() {
 	ROS_INFO("Initializing Publishers for TrajectoryPlanningNode"); 	// ---> add publishers here
 	//Example: uref_pub = nh_.advertise<std_msgs::Float64>("URef", 10); //Surge Reference

 	wp_status_timer_pub = nh_.advertise<std_msgs::Bool>("wp_status",10);

	//std::string vc_topic_name =  "/mblack/slap/internal/vc";
	std::string st_curve_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, 
	                                                                       "topics/publishers/st_curve");  
	std::string st_curve_to_console_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, 
	                                                                       "topics/publishers/st_curve_to_console");  

	std::string internal_gamma_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_,"topics/publishers/internal_gamma");
 																	   

	st_curve_pub = nh_.advertise<medusa_slap_msg::STCurve>(st_curve_topic_name,1); ; 
	st_curve_to_console_pub = nh_.advertise<farol_msgs::mState>(st_curve_to_console_topic_name,1); 
	internal_gamma_pub_ = nh_.advertise<farol_msgs::CPFGamma>(internal_gamma_topic_name, 1); 

 }

 /*
#######################################################################################################################
 @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
 #######################################################################################################################
 */
 void TrajectoryPlanningNode::initializeTimer() {
 	timer_ =nh_.createTimer(ros::Duration(1.0/TrajectoryPlanningNode::nodeFrequency()), &TrajectoryPlanningNode::timerIterCallback, this);
 //	timer_.stop();
 }

 /*
#######################################################################################################################
 @.@ Set frequency of the node default is 2
 #######################################################################################################################
 */
 double TrajectoryPlanningNode::nodeFrequency()
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
 void TrajectoryPlanningNode::loadParams() {
 	ROS_INFO("Load the TrajectoryPlanningNode parameters");
 	//---> params here, always p_paramName
	nh_p_.getParam("vd_bar", vd_bar_); 
	vd_bar_new_ = vd_bar_;
	vd_bar_old_ = vd_bar_;
 	double rx, ry, rz, formation_offset; 
	nh_p_.getParam("radius_x", rx); 
	nh_p_.getParam("radius_y", ry); 
	nh_p_.getParam("radius_z", rz); 
	nh_p_.getParam("formation_offset", formation_offset); 
	trajectory_planning_algorithm_.updatePathParameter(rx,ry,rz,formation_offset*M_PI/180);
 }

 void TrajectoryPlanningNode::targetPdfCallback(const medusa_slap_msg::TargetPDF &msg) {

  /* If the algorithm is running, signal that we have received data from the
   * vehicle state */
  if (!received_target_pdf) {
    received_target_pdf = true;
	ROS_INFO("Trajectory planning node has received the first target's pdf"); 	

  }
	/* update target position */
	target_pos_[0] = msg.state[0];	
	target_pos_[1] = msg.state[1];	

	/* update target velocity */
	target_vel_[0] = msg.state[2];	
	target_vel_[1] = msg.state[3];	
}
void TrajectoryPlanningNode::internalGammaDotCallback(const std_msgs::Float64& msg){
  if (!received_internal_gamma_dot ) {
    received_internal_gamma_dot = true;
	ROS_INFO("Trajectory planning node has received the first internal gamma"); 	
  }
	double gamma_dot_ = msg.data;
	gamma_ += gamma_dot_;

}
 /** #######################################################################################################################
 @.@ Iteration via timer callback
 #######################################################################################################################
 */
 void TrajectoryPlanningNode::timerIterCallback(const ros::TimerEvent &event) {
	 
	trajectory_planning_algorithm_.updateTargetTrajectory(target_pos_, target_vel_);
	trajectory_planning_algorithm_.updateGamma(gamma_);
 	trajectory_planning_algorithm_.callTrajectoryPlanningAlgorithm(0.1);
    

	Eigen::Vector3d traj_pos = trajectory_planning_algorithm_.getTrajPos();
    Eigen::Vector3d traj_vel = trajectory_planning_algorithm_.getTrajVel();
    Eigen::Vector3d path_pos = trajectory_planning_algorithm_.getPathPos();
    Eigen::Vector3d path_vel_gamma = trajectory_planning_algorithm_.getPathVelGamma();

	/* Publish desired S-T curve */
	medusa_slap_msg::STCurve msg;
	double precision_num = 0.001*(vd_bar_new_ - vd_bar_old_);
    if (abs(vd_bar_ - vd_bar_new_) >= abs(precision_num)){
		vd_bar_ += precision_num ; 		// just to make the transition smooth
	}
	msg.vd_bar  = vd_bar_;
	for(int i = 0; i < 3; i++) {
		msg.traj_pos[i] = traj_pos[i];
		msg.traj_vel[i] = traj_vel[i];
		msg.path_pos[i] = path_pos[i];
		msg.path_vel_gamma[i] = path_vel_gamma[i];
	}
	st_curve_pub.publish(msg) ; 
	
	/* Publish gamma internally */
	farol_msgs::CPFGamma internal_gamma_msg;
	internal_gamma_msg.gamma =  gamma_;
	internal_gamma_msg.vd =  vd_bar_;
	
   	internal_gamma_pub_.publish(internal_gamma_msg);


/* Publish the reference point on the S-T curve that vehicle need to track  */
      farol_msgs::mState st_state_msg; 

	  st_state_msg.Y = traj_pos[0] + path_pos[0];
      st_state_msg.X = traj_pos[1] + path_pos[1];
      st_state_msg.Z = traj_pos[2] + path_pos[2];
	  double heading = atan2(traj_vel[1] + path_vel_gamma[1], traj_vel[0] + path_vel_gamma[0])* 180 /M_PI;
	  st_state_msg.Yaw = (int(heading) + 360) % 360;
	// // //  st_state_msg.Y =  4290841;
    // // //  st_state_msg.X =  491926;
    // // //  st_state_msg.Z =   0.0;
    // //  st_state_msg.Yaw = 0.0; 
	  st_curve_to_console_pub.publish(st_state_msg);



 //	aux_bool.data = true; 	wp_status_timer_pub.publish(std_msgs::Bool(aux_bool));
 }
 /*
#######################################################################################################################
 @.@ Callback Flag
 #######################################################################################################################
 */
 void TrajectoryPlanningNode::wpStatusCallback(const std_msgs::Int8& msg) {

 	aux_int.data = 4;
 	if (msg.data != aux_int.data){
 		 ROS_INFO("Timer will Stop");
 		 timer_.stop();
 	} 	else{
 		 ROS_INFO("Timer will Start");
 		 timer_.start();
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
 	ros::init(argc, argv, "trajectory_planning_node"); //node name
 	// +.+ create a node handle; need to pass this to the class constructor
 	ros::NodeHandle nh;

 	ros::NodeHandle nh_p("~");

 	ROS_INFO("main: instantiating an object of type TrajectoryPlanningNode");

 	// +.+ instantiate an TrajectoryPlanningNode class object and pass in pointer to nodehandle for constructor to use
 	TrajectoryPlanningNode trajectoryPlanning(&nh,&nh_p);

 	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
 	ros::spin();

 	return 0;
 }

