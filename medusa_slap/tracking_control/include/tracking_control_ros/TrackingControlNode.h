/**
 * @brief This defines class for TrackingControlNode
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

/* Messages used to receive data from the vehicle and from the path */
 #include <auv_msgs/NavigationStatus.h>
 #include "StateStructs.h"
 #include <farol_msgs/mState.h>

/* To use wrap function in */ 
#include <farol_gimmicks_library/FarolGimmicks.h>

/* To use the tracking algorithm */
#include "TrackingControlAlgorithm.h"

/* to subscribe message containing target pdf */  
#include <medusa_slap_msg/TargetPDF.h>
#include <medusa_slap_msg/STCurve.h>
#include "medusa_slap_msg/StartStop.h"
#include "medusa_slap_msg/SetFloat64.h"
#include "medusa_slap_msg/SetTCGains.h"
#include "medusa_slap_msg/CPFinfo.h"





 class TrackingControlNode {
 public:
 	/* Constructor 	*/
		TrackingControlNode(ros::NodeHandle* nh, ros::NodeHandle *nh_p);
	
	/* Destructor	*/
 		~TrackingControlNode();

 	/* Public methods	*/
 	double nodeFrequency();

 private:
	/*	For node handle	*/
 		ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
 		ros::NodeHandle nh_p_; // we will need this, to pass between "main" and constructor

 	/* object of TrackingControlAlgorithm to be used 	*/
		TrackingControlAlgorithm tracking_algorithm_ = TrackingControlAlgorithm();

    /**
     * @brief Auxiliary variables to store the current vehicle state and path state, and target state 
     */
    	VehicleState vehicle_state_;
		PathPara path_para_;
		SpatialTemporalCurve st_curve_state_;
    /**
     * @brief Auxiliary variables to check whether we have received the first informaiton from the filter or not
     */
    	bool received_vehicle_state{false};
		bool received_vc{false};
		bool first_interation{false};
		bool received_st_curve{false};


 	/**
 	* @brief Subscribers
 	*/
		ros::Subscriber flag_sub_;					// for vehicle status
		ros::Subscriber state_sub_;					// for vehicle state
		ros::Subscriber cpf_info_sub_;					// for correction speed - from cooperative controller node 
		ros::Subscriber st_curve_sub_;					// for correction speed - from cooperative controller node 


	/**
	 * @brief Publishers
	 */
 		ros::Publisher wp_status_timer_pub;
		/* to inner loops */
		ros::Publisher surge_pub_;
		ros::Publisher yaw_rate_pub_;
		/* to cooperative controller */
		ros::Publisher gamma_dot_pub_;

    /**
	 * @brief Services
	 */
 		ros::ServiceServer start_tracking_srv_ ;
 		ros::ServiceServer stop_tracking_srv_ ;
		ros::ServiceServer set_tc_gains_srv_ ;

 	/**
 	* @brief Timer
 	*/
 		ros::Timer timer_;


	/**
     * @brief Use to store the time-elapsed to be used by the controller
     */
		ros::Time prev_time_; 
		ros::Time initial_time;

 	// ####################################################################################################################
 	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
 	// member variables will retain their values even as callbacks come and go
 	// ####################################################################################################################

 	// +.+ Parameters from Yaml
 	// always p_paraName -> Example: double p_ku;

 	// +.+ Handy variables
 	// Example double var_temp_x, var_temp_y;

 	// +.+ Problem variables
 	// Example: double x_state, y_state;
 	std_msgs::Int8 aux_int;
 	std_msgs::Bool aux_bool;

 	/**
	  * @brief Methods for initialize subcribers, publishers and timer
 	*/
 	void initializeSubscribers();
 	void initializePublishers();
 	void initializeTimer();
 	void loadParams();
    void initializeServices();
 	/**
	  * @brief Callback to update vehicle states, correction speed, estimated target state ...
 	*/
 	void timerIterCallback(const ros::TimerEvent& event);
	void wpStatusCallback(const std_msgs::Int8& msg);
	void vehicleStateCallback(const auv_msgs::NavigationStatus &msg);
	void cpf_infoCallback(const medusa_slap_msg::CPFinfo &msg);
	void stCurveCallback(const medusa_slap_msg::STCurve &msg);
	void flagCallback(const std_msgs::Int8 &msg);


    /**
	 * @brief methods using service to start and stop tracing algorithm
	 */
	bool startTracking(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);
	bool stopTracking(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);
	bool setTCGains(medusa_slap_msg::SetTCGains::Request &req, medusa_slap_msg::SetTCGains::Response &res);

    /**
	 *  @brief these two methods to update PathState and TargetState
	 */

  	farol_msgs::mState st_state_msg;
	double Ts_;										// sampling time


};
