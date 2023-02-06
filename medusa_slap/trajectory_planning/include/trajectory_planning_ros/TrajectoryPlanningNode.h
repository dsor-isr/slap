/**
 * @brief This defines class for TrajectoryPlanningNode
 * @author  DSOR team
 * @author  Person in charges: Hung, Quintas, Cruz
 * @date    2021
 */
#pragma once

 /* some generically useful stuff to include...  */
 #include <math.h>
 #include <stdlib.h>
 #include <vector>
 #include <ros/ros.h> //ALWAYS need to include this 

 /*  ROS messages and stuff  */
 #include <std_msgs/Float64.h>
 #include <std_msgs/Int8.h>
 #include <std_msgs/Bool.h>

/* To use wrap function in */ 
#include <farol_gimmicks_library/FarolGimmicks.h>

/* To use the tracking algorithm */
#include "TrajectoryPlanningAlgorithm.h"

/* to subscribe message containing target pdf */  
 #include <medusa_slap_msg/TargetPDF.h>
 #include <medusa_slap_msg/STCurve.h>
 #include <medusa_slap_msg/SetFloat64.h>
 #include <medusa_slap_msg/SetFixedTargetMode.h>
 #include <medusa_slap_msg/SetRotateTargetMode.h>

 #include <farol_msgs/CPFGamma.h>
 #include <farol_msgs/mState.h>



 class TrajectoryPlanningNode {
 public:
 	/* Constructor */
 	TrajectoryPlanningNode(ros::NodeHandle* nh, ros::NodeHandle *nh_p);
    
	/* Destructor  */
 	~TrajectoryPlanningNode();
	
	/* Public method	*/
 	double nodeFrequency();

 private:
 	// put private member data here; "private" data will only be available to member functions of this class;
 	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

 	ros::NodeHandle nh_p_; // we will need this, to pass between "main" and constructor

 	/* object of Dekf to be used 	*/
	TrajectoryPlanningAlgorithm trajectory_planning_algorithm_ = TrajectoryPlanningAlgorithm();

 	/** subscriber 
	 * @brief define subcribers for target trajectory 
	 */
 	ros::Subscriber flag_sub;
	ros::Subscriber target_pdf_sub_; 
	ros::Subscriber internal_gamma_dot_sub_;


	/** publishers 
	 * @brief define publishers to publish target's pdf (state and covariance) to tracking_controller and communication 
	 */
	ros::Publisher wp_status_timer_pub;
	ros::Publisher st_curve_pub;
	ros::Publisher internal_gamma_pub_;

			/* to console 	*/
	ros::Publisher st_curve_to_console_pub;

    /** services
	 * brief define service
	 */
	ros::ServiceServer desired_gamma_dot_srv_;
	ros::ServiceServer fixed_target_mode_srv_;
	ros::ServiceServer rotate_target_mode_srv_;
	ros::ServiceServer set_gamma_srv_;


 	// #######################
 	// @.@ Timer
 	// #######################
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
	void wpStatusCallback(const std_msgs::Int8& msg);
	void targetPdfCallback(const medusa_slap_msg::TargetPDF &msg);
	void internalGammaDotCallback(const std_msgs::Float64& msg);
	bool updateDesiredGammaDot(medusa_slap_msg::SetFloat64::Request &req, 
	                           medusa_slap_msg::SetFloat64::Response &res);
	bool setFixedTargetMode(medusa_slap_msg::SetFixedTargetMode::Request &req, 
	                           medusa_slap_msg::SetFixedTargetMode::Response &res);
	bool setRotateTargetMode(medusa_slap_msg::SetRotateTargetMode::Request &req, 
	                           medusa_slap_msg::SetRotateTargetMode::Response &res);	
	bool setGamma(medusa_slap_msg::SetFloat64::Request &req, 
	                           medusa_slap_msg::SetFloat64::Response &res);						   						   
	// bool updateSetFloat64(medusa_slap_msg::SetFloat64::Request &req, 
	//                            medusa_slap_msg::SetFloat64::Response &res);						   


	bool received_target_pdf{false};
	bool received_internal_gamma_dot{false};
	Eigen::Vector3d target_pos_;
	Eigen::Vector3d target_vel_;
	double gamma_{0};
	double vd_bar_;
	double vd_bar_new_;
	double vd_bar_old_;

};
