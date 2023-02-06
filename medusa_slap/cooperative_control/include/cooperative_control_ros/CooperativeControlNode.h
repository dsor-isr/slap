/**
 * @brief This defines class for CooperativeControllerNode
 * @author  DSOR team
 * @author  Person in charges: Hung, Quintas, Cruz
 * @date    2021
 */
#pragma once

 //some generically useful stuff to include...
 #include <math.h>
 #include <stdlib.h>
 #include <std_msgs/String.h>
 #include <vector>

 #include <ros/ros.h> //ALWAYS need to include this 

 // ROS messages and stuff
 // Examples
 //#include <medusa_msgs/mState.h>
 //#include <geometry_msgs/PointStamped.h>
 //#include <geometry_msgs/Vector3Stamped.h>
 #include <std_msgs/Float64.h>
 #include <std_msgs/Int8.h>

 #include <std_msgs/Bool.h>

 #include <farol_msgs/CPFGamma.h>
 #include <medusa_slap_msg/SetFloat64.h>
 #include <medusa_slap_msg/StartStop.h>
 #include <medusa_slap_msg/ETCInfo.h>

 #include <medusa_slap_msg/SetETCParameter.h>



 #include "CooperativeControlAlgorithm.h"
 
 
 class CooperativeControlNode {
 public:
 	/*	Constructor */
 	CooperativeControlNode(ros::NodeHandle* nh, ros::NodeHandle *nh_p);
	/*	Destructor	*/ 
 	~CooperativeControlNode();
	/*	Get node frequency	*/
 	double nodeFrequency();

 private:
 	// put private member data here; "private" data will only be available to member functions of this class;
 	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

 	ros::NodeHandle nh_p_; // we will need this, to pass between "main" and constructor

	/* Define a cooperative control */

	CooperativeControlAlgorithm cooperative_control_algorithm_ =  CooperativeControlAlgorithm();

	ros::Subscriber internal_gamma_sub_;
	ros::Subscriber neighbor_gamma_sub_; 
	
	/**	Publishers */
 	
	ros::Publisher vc_pub_;
	ros::Publisher to_neighbor_gamma_pub_;		
	ros::Publisher cpf_etc_info_pub_;		

	/** Services */
	ros::ServiceServer kc_srv_;		
	ros::ServiceServer start_srv_;
	ros::ServiceServer stop_srv_;		
	ros::ServiceServer set_etc_parameter_srv_;
		


	/** Timer */
 	ros::Timer timer_;

 	std_msgs::Int8 aux_int;
 	std_msgs::Bool aux_bool;

 	// #######################################################################################
 	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	// #######################################################################################
 	void initializeSubscribers();
 	void initializePublishers();
 	void initializeTimer();
 	void loadParams();
	void initializeServices();

 	// #######################################################################################
 	// @.@ Callbacks declaration
 	// #######################################################################################
 	void timerIterCallback(const ros::TimerEvent& event);
	void internalGammaCallback(const farol_msgs::CPFGamma& msg);
	void neighborGammaCallback(const farol_msgs::CPFGamma& msg);

/**
	 * @brief methods using service to start and stop tracing algorithm
	 */
	bool startCooperativeControl(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);
	bool stopCooperativeControl(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);
	bool setGainsService(medusa_slap_msg::SetFloat64::Request &req, medusa_slap_msg::SetFloat64::Response &res);

	bool setETCParameter(medusa_slap_msg::SetETCParameter::Request &req, medusa_slap_msg::SetETCParameter::Response &res);
//	bool setETCParameter(medusa_slap_msg::SetETCParameter::Request &req, medusa_slap_msg::SetETCParameter::Response &res){

/* Variable to store data subscribed */
	double internal_gamma_;
	double neighbor_gamma_; 
	double internal_gamma_desired_speed_ ;
	double vc_;	
/* Auxilar variable */
	bool received_internal_gamma{false};
	bool received_neighbor_gamma{false};
	int seq_{0};
	Eigen::VectorXi neighbor_set_;
	Eigen::VectorXd gamma_vector_;
	Eigen::VectorXd gamma_desired_speed_vector_;

	int Veh_ID;

	ros::Time initial_time;

};
