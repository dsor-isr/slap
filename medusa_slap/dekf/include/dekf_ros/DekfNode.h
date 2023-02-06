/**
 * @brief This defines class for dekf
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

 #include <std_msgs/Int8.h>

 #include <std_msgs/Bool.h>

/* class generated by message */
 #include <auv_msgs/NavigationStatus.h>
 #include <farol_msgs/mState.h>
 #include <farol_msgs/mUSBLFix.h>


 #include <medusa_slap_msg/TargetPDF.h>
 #include <medusa_slap_msg/VehiclePosVel.h>
 #include <medusa_slap_msg/StartStop.h>
 #include <medusa_slap_msg/SetTargetPDF.h>

 #include <medusa_slap_msg/SetMatricesQR.h>
 #include <medusa_slap_msg/SetETCParameter.h>

 #include <medusa_slap_msg/ETCInfo.h>


 #include <random>


 /* class dekf algorithm */
 #include "DekfAlgorithm.h"

 class DekfNode {
 public:
	/* Constructor */
 	DekfNode(ros::NodeHandle* nh, ros::NodeHandle *nh_p);
	
	/* Destructor */
 	~DekfNode();

 	/* Public methods */
 	double nodeFrequency();

 private:
 	// put private member data here; "private" data will only be available to member functions of this class;
 	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
	ros::NodeHandle nh_p_; // we will need this, to pass between "main" and constructor

 	/* object of Dekf to be used 	*/

		DekfAlgorithm dekf_algorithm_ = DekfAlgorithm();

	/** subscriber 
	 * @brief define subcribers for range measurement, and density function from other vehicles 
	 */
	ros::Subscriber target_range_sub_;
	ros::Subscriber target_pdf_from_neighbor_sub_;
	ros::Subscriber neighbor_pos_sub_;
	ros::Subscriber vehicle_state_sub_;

	/** publishers 
	 * @brief define publishers to publish target's pdf (state and covariance) to tracking_controller and communication 
	 */
 	ros::Publisher wp_status_timer_pub;
	ros::Publisher target_pdf_pub_; 									// publish target probability density function
	ros::Publisher target_pdf_to_neighbor_pub_; 						// publish target probability density function
	ros::Publisher estimated_target_to_console_pub_;
	ros::Publisher dekf_etc_info_pub_;


	/**
	 * @brief Services
	 */
	ros::ServiceServer start_dekf_srv_ ;
	ros::ServiceServer stop_dekf_srv_ ;
	ros::ServiceServer start_ekf_srv_ ;
	ros::ServiceServer stop_ekf_srv_ ; 
	ros::ServiceServer set_target_pdf_srv_ ;

	ros::ServiceServer set_matrices_QR_srv_ ; 
	ros::ServiceServer set_ETC_parameter_srv_ ; 


	/**
	 * @brief Timer
	 */
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


 	/**
 	*@brief Callbacks declaration
 	 */
 	void timerIterCallback(const ros::TimerEvent& event);
	void targetPdfFromNeighborCallback(const medusa_slap_msg::TargetPDF& msg);
	void targetRangeCallback(const farol_msgs::mUSBLFix& msg);
	void neighborPositionCallback(const medusa_slap_msg::VehiclePosVel& msg);
	void vehicleStateCallback(const auv_msgs::NavigationStatus &msg);

	/**
	 * @brief methods using service to start and stop dekf algorithms and set pdf for target
	 */
	bool startDekf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);
	bool stopDekf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);

	bool startEkf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);
	bool stopEkf(medusa_slap_msg::StartStop::Request &req, medusa_slap_msg::StartStop::Response &res);

   	bool setTargetPDF(medusa_slap_msg::SetTargetPDF::Request &req, medusa_slap_msg::SetTargetPDF::Response &res);
	
	bool setMatricesQR(medusa_slap_msg::SetMatricesQR::Request &req, medusa_slap_msg::SetMatricesQR::Response &res);

	bool setETCParameter(medusa_slap_msg::SetETCParameter::Request &req, medusa_slap_msg::SetETCParameter::Response &res);
	/**
	 * @brief auxilary methods
	 */    

    void smoothTargetState();
	bool checkOutlier(double range);


	/**
     * @brief Frame_id and seq for messages 
     */
    uint32_t seq_{0};
    std::string frame_id_;

	/**
     * @brief variables to check if received messages in callback methods
     */
	bool received_target_range{false};
	bool received_target_pdf_from_neighbor{false}; 
	bool received_neighbor_position{false}; 
	bool received_vehicle_state{false}; 

	/**
     * @brief variables to check if received messages in callback methods
     */

	int Veh_ID; 
	Eigen::VectorXi neighbor_set_;
	Eigen::Vector2d vehicle_position_{0.0,0.0}; 
	Eigen::Vector2d target_position_{0.0,0.0};
	Eigen::Vector2d target_velocity_{0.0,0.0};

	double target_depth_{0.0};

    /* Simulate Gaussian random noise */

    const double mean{0.0};
    const double stddev{0.1};
    std::default_random_engine generator;
	std::normal_distribution<double> dist;
	
	/**
     * @brief to store target pdf
     */
    TargetPDF internal_target_pdf_;
	TargetPDF neighbor_target_pdf_;

	/**
     * @brief store augmented target pdf
     */
	
	Eigen::MatrixXd augmented_target_info_vector_;
	Eigen::MatrixXd augmented_target_info_matrix_;

	/**
     * @brief store variable start and stop ekf and dekf mode
     */

	bool ekf_start_{false};
	bool dekf_mode_{false};
	
	/**
     * @brief auxilary variables
     */
	Eigen::Vector4d state_offset_vector_;
    Eigen::Vector2d vehicle2_position_;
	Eigen::Vector4d smoothed_target_state_;
	bool tracking_true_target_{false};
	ros::Time initial_time;
    
};