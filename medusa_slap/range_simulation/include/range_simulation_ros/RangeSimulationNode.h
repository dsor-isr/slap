/**
 * @brief This defines a class for Rangesimulation
 * @author  DSOR team
 * @author  Persons in charges: Hung, Quintas, Cruz
 * @date    2021
 */
#pragma once

 //some generically useful stuff to include...
 #include <math.h>
 #include <stdlib.h>
 #include <std_msgs/String.h>
 #include <vector>

 #include <ros/ros.h> //ALWAYS need to include this 

 #include <random>

 #include <farol_msgs/mUSBLFix.h>
 #include <medusa_slap_msg/VehiclePosVel.h>
 #include <auv_msgs/NavigationStatus.h>

#include <farol_gimmicks_library/FarolGimmicks.h>


 class RangeSimulationNode {
 public:
 	/* Constructor */
 	RangeSimulationNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

	/* Destructor */
 	~RangeSimulationNode();

 	/* Public methods */
 	double nodeFrequency();

 private:
 	// put private member data here; "private" data will only be available to member functions of this class;
 	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
 	ros::NodeHandle nh_p_; // we will need this, to pass between "main" and constructor

	/** subscriber 
	 * @brief define subcribers for vehicle position and target position  
	 */

	ros::Subscriber target_pos_sub_;
	ros::Subscriber vehicle_state_sub_; 
 	
	/** publisher 
	 * @brief define publisher to publish range  
	 */
 	ros::Publisher target_range_pub_; 

	/**
	 * @brief Timer
	 */
 	ros::Timer timer_;


 	// #######################################################################################
 	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	// #######################################################################################
 	void initializeSubscribers();
 	void initializePublishers();
 	void initializeTimer();
 	void loadParams();

 	// #######################################################################################
 	// @.@ Callbacks declaration
 	// #######################################################################################

	void targetPositionCallback(const medusa_slap_msg::VehiclePosVel& msg);
	void vehicleStateCallback(const auv_msgs::NavigationStatus &msg);

 	void timerIterCallback(const ros::TimerEvent& event);

	Eigen::Vector2d vehicle_position_; 
	Eigen::Vector2d target_position_;
	double target_depth_;
	
    /* Simulate Gaussian random noise */

    const double mean{0.0};
    const double stddev{0.01};
    std::default_random_engine generator;
	std::normal_distribution<double> dist; 

	bool received_target_position{false};
	bool received_vehicle_state{false};
	
 };
