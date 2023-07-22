
/**
 * @brief This distributed extended Kalman filter (DEKF) is used for estimating target state in the context of target localization
 * and pursuit, given in the paper at: https://nt-hung.github.io/files/pdf/research/IEEE_TCST_preprint.pdf
 * 
 * The estimator output
 *      - estimated target information (state and covariance)
 * It requires inputs:
 *      - range to the target
 *      - estimated target informatoin from neighbor 
 *      - target_state     
 * External disturbance (e.g. ocean disturbance) can be easily incorporated but for the simplicity of 
 * exposition of the code, we excluded in this version.  
 * 
 * @author    DsorTeam
 * @author    Persons in charge: Hung
 * @version   1.0a
 * @date      2021
 * @copyright GPLv3
 */

#include "DekfNode.h"

/**
 * @brief constructor
 */
 DekfNode::DekfNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
                    :nh_(*nh), nh_p_(*nh_p) {
 	ROS_INFO("in class constructor of DekfNode");
 	loadParams();
 	initializeSubscribers();
 	initializePublishers();
 	initializeTimer();
	initializeServices();
//	initialize Gaussian noise

	std::normal_distribution<double> dist(mean,stddev);
    
 }

/**
 * @brief destructor
 */
 DekfNode::~DekfNode() {

 	// +.+ stop timer
 //	timer_.stop();

 	// +.+ shutdown node
 	nh_.shutdown();
 }

 /**
  * @brief method initialie subcribber 
 */
 void DekfNode::initializeSubscribers() {
 	ROS_INFO("Initializing Subscribers for DekfNode");
 	// ---> add subscribers here
	
	std::string target_pdf_from_neighbor_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_,"topics/subscribers/neighbor_target_pdf"); 
    std::string target_range_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_,"topics/subscribers/target_range");
	std::string neighbor_position_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/neighbor_pos");
	std::string vehicle_state_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/vehicle_state");

    target_range_sub_ = nh_.subscribe(target_range_topic_name, 10, &DekfNode::targetRangeCallback, this);
	target_pdf_from_neighbor_sub_ = nh_.subscribe(target_pdf_from_neighbor_topic_name, 10, &DekfNode::targetPdfFromNeighborCallback, this);
	neighbor_pos_sub_ = nh_.subscribe(neighbor_position_topic_name, 10, &DekfNode::neighborPositionCallback, this);;
	vehicle_state_sub_ = nh_.subscribe(vehicle_state_topic_name, 10, &DekfNode::vehicleStateCallback, this);
 }

 /**
  * @brief Initialize publishers
 */
 void DekfNode::initializePublishers() {
 	ROS_INFO("Initializing Publishers for DekfNode"); 	// ---> add publishers here

	/* These topics names later can be get from config file */  
	std::string target_pdf_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/target_pdf");
	std::string target_pdf_to_neighbor_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/target_pdf_to_neighbor");
	std::string estimated_target_to_console_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/estimated_target_to_console");
	std::string dekf_etc_info_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/dekf_etc_info");
	std::string target_pdf_absolute_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/target_pdf_absolute");
	std::string target_pdf_aux_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/target_pdf_aux");

	/* Initialize the publisher */
	
	target_pdf_pub_ = nh_.advertise<medusa_slap_msg::TargetPDF>(target_pdf_topic_name,1);  
	target_pdf_to_neighbor_pub_ = nh_.advertise<medusa_slap_msg::TargetPDF>(target_pdf_to_neighbor_topic_name,1);  
	estimated_target_to_console_pub_ = nh_.advertise<farol_msgs::mState>(estimated_target_to_console_topic_name,1);  
	dekf_etc_info_pub_ = nh_.advertise<medusa_slap_msg::ETCInfo>(dekf_etc_info_topic_name,1);  

	target_pdf_absolute_pub_ = nh_.advertise<medusa_slap_msg::TargetPDF>(target_pdf_absolute_topic_name,1);  
	target_pdf_aux_pub_ = nh_.advertise<medusa_slap_msg::TargetPDF>(target_pdf_aux_topic_name,1);  


 }

 /*
#######################################################################################################################
 @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
 #######################################################################################################################
 */
 void DekfNode::initializeTimer() {
 	timer_ =nh_.createTimer(ros::Duration(1.0/DekfNode::nodeFrequency()), &DekfNode::timerIterCallback, this);
 //	timer_.stop();
 }

 /**
  * @brief Set node frequency 
 */
 double DekfNode::nodeFrequency()
 {
 	double node_frequency;
 	nh_p_.param("node_frequency", node_frequency, 10.0);
 	ROS_INFO("Node will run at : %lf [hz]", node_frequency);
 	return node_frequency;
 }

 /**
  * @brief Load parameters from config files
 */
 void DekfNode::loadParams() {
 	ROS_INFO("Load the DekfNode parameters");
	std::vector<int> adj_matrix;
	std::vector<double> ini_target_state;
	std::vector<double> ini_target_cov;
	std::vector<double> state_offset;
	nh_p_.getParam("target_depth", target_depth_);
	nh_p_.getParam("Veh_ID", Veh_ID);
	nh_p_.getParam("adj_matrix", adj_matrix);
	nh_p_.getParam("ini_target_state", ini_target_state);
	nh_p_.getParam("state_offset", state_offset);
	nh_p_.getParam("ini_target_cov", ini_target_cov);

    /* get parameters of threshoud function for ETC */
	double c0, c1, c2 ;
    nh_p_.getParam("c0", c0);
	nh_p_.getParam("c1", c1);
	nh_p_.getParam("c2", c2);

	dekf_algorithm_.setETCParameters(c0, c1, c2);

	
	int num_vehicles = sqrt(adj_matrix.size());
    neighbor_set_.resize(num_vehicles);	 					// update neighbor set
	augmented_target_info_vector_.resize(4,num_vehicles);
	augmented_target_info_matrix_.resize(4,4*num_vehicles);

	/**  
	 * @brief Set initial internal target's state
	 */ 
    for (int i = 0; i < 4; i++){
	   target_pdf_local_init_.state[i] = ini_target_state[i];
	   state_offset_vector_[i] = state_offset[i];
	   for (int j = 0; j < 4; j++){
		   target_pdf_local_init_.cov(i,j) = ini_target_cov[4*i+j];
	   } 
    }  

	for (int i = 0; i < num_vehicles; i++){
		neighbor_set_[i] = adj_matrix[num_vehicles*Veh_ID + i];
	}
	

	dekf_algorithm_.setLocalTargetPDF(target_pdf_local_init_); 				// update vehicle internal target pdf
    dekf_algorithm_.setNeighborSet(neighbor_set_); 							// set neighbor set
    dekf_algorithm_.setVehicleID(Veh_ID); 									// set vehicle ID
	dekf_algorithm_.setTargetDepth(target_depth_);

	dekf_algorithm_.setAuxTargetPDF(); 				// for ETC

 }

 /**
  * @brief timer call back
  */ 
 void DekfNode::timerIterCallback(const ros::TimerEvent &event) {

	float time = abs((ros::Time::now()- initial_time).toSec());

	/** 
	 * @brief Keep predicting target's state at every sampling interval (1/node_frequency) 
	*/
	  if (ekf_enable_){
	 	dekf_algorithm_.localPrediction();	 				
	  } 
	
	/** 
	 * @brief Publish target pdf (to be used by the tracking controller) 
	*/
		TargetPDF target_pdf_local_ ;

		target_pdf_local_.state = dekf_algorithm_.getLocalTargetState();	
		target_pdf_local_.cov   = dekf_algorithm_.getLocalTargetCovariance();
		smoothTargetState(target_pdf_local_.state );

        Eigen::Vector4d  absolute_target_state;

		if (tracking_true_target_)
		{
		    absolute_target_state <<   	target_position_[0] , 
		                               	target_position_[1] ,
		 							    target_velocity_[0],
		 								target_velocity_[1];
		}
		else
		{
	   		absolute_target_state <<  smoothed_target_state_[0] + state_offset_vector_[0],
									  smoothed_target_state_[1] + state_offset_vector_[1],
									  smoothed_target_state_[2],
									  smoothed_target_state_[3];							  
		}							 	 
	 	medusa_slap_msg::TargetPDF msg_pdf_absolute;
		msg_pdf_absolute.Veh_ID = Veh_ID; 
		for(int i = 0; i < 4; i++) {
     		msg_pdf_absolute.state[i] = absolute_target_state[i];
         	msg_pdf_absolute.cov_row1[i] = target_pdf_local_.cov(0,i);
        	msg_pdf_absolute.cov_row2[i] = target_pdf_local_.cov(1,i); 
	 		msg_pdf_absolute.cov_row3[i] = target_pdf_local_.cov(2,i); 
        	msg_pdf_absolute.cov_row4[i] = target_pdf_local_.cov(3,i); 
			msg_pdf_absolute.ekf_enable  = ekf_enable_;
			msg_pdf_absolute.dekf_enable = dekf_enable_;
			msg_pdf_absolute.time = time;			 
        } 
		target_pdf_absolute_pub_.publish(msg_pdf_absolute) ;
 	
	 /** 
	  * @brief publish target pdf, to be used by the udp server 
	  *        for broadasting to neighbor - this depend on event-triggered mechanism
	  */

	 	if (dekf_enable_){
	
			EtcInfo etc_info_ = dekf_algorithm_.checkBroadcastSignal(time);


 			medusa_slap_msg::ETCInfo msg_dekf_etc;
			msg_dekf_etc.broadcast_signal = etc_info_.broadcast_signal;
			msg_dekf_etc.threshold = etc_info_.threshold;
 			msg_dekf_etc.error = etc_info_.kld;
			msg_dekf_etc.time  = time;
			dekf_etc_info_pub_.publish(msg_dekf_etc);
			if (etc_info_.broadcast_signal){
			/**
			 * @brief get information state and matrix to broadcast to neighbors
			 */
				Eigen::Vector4d info_vector = dekf_algorithm_.getLocalTargetInfoVector();				   	
				Eigen::Matrix4d info_matrix = dekf_algorithm_.getLocalTargetInfoMatrix();
			
				medusa_slap_msg::TargetPDF msg_pdf_to_neighbor_;
			/* Header for the message */
				msg_pdf_to_neighbor_.Veh_ID = Veh_ID;
				msg_pdf_to_neighbor_.header.seq = seq_;
				seq_++	;				 //Increment the sequence id
				msg_pdf_to_neighbor_.header.stamp = ros::Time::now();
				msg_pdf_to_neighbor_.header.frame_id = "mblack/target/PDF_frame";
				for(int i = 0; i < 4; i++) {
					msg_pdf_to_neighbor_.state[i] = info_vector[i];
					msg_pdf_to_neighbor_.cov_row1[i] = info_matrix(0,i);
					msg_pdf_to_neighbor_.cov_row2[i] = info_matrix(1,i); 
					msg_pdf_to_neighbor_.cov_row3[i] = info_matrix(2,i); 
					msg_pdf_to_neighbor_.cov_row4[i] = info_matrix(3,i); 
				} 
				msg_pdf_to_neighbor_.time = time;
				target_pdf_to_neighbor_pub_.publish(msg_pdf_to_neighbor_) ;
			}
		 }
/* Publish estimated target position to console */
		farol_msgs::mState estimated_target_msg; 
		
		estimated_target_msg.Y = target_pdf_local_.state[0] + state_offset_vector_[0];
		estimated_target_msg.X = target_pdf_local_.state[1] + state_offset_vector_[1];
		estimated_target_msg.Z = 0 ;
		double heading = atan2(target_pdf_local_.state[3],target_pdf_local_.state[2])* 180 /M_PI;
		estimated_target_msg.Yaw = (int(heading) + 360) % 360;
		estimated_target_to_console_pub_.publish(estimated_target_msg);


/* Publish relative target pdf (state and information matrix) for debug  */
	 	medusa_slap_msg::TargetPDF msg_target_pdf_local;
		msg_target_pdf_local.Veh_ID = Veh_ID; 
		for(int i = 0; i < 4; i++) {
     		msg_target_pdf_local.state[i]    = target_pdf_local_.state[i];
         	msg_target_pdf_local.cov_row1[i] = target_pdf_local_.cov(0,i);
        	msg_target_pdf_local.cov_row2[i] = target_pdf_local_.cov(1,i); 
	 		msg_target_pdf_local.cov_row3[i] = target_pdf_local_.cov(2,i); 
        	msg_target_pdf_local.cov_row4[i] = target_pdf_local_.cov(3,i); 
        } 
		msg_target_pdf_local.time = time;
		target_pdf_pub_.publish(msg_target_pdf_local) ;
/* Publish aux target pdf (state and information matrix) for debug  */		
		TargetPDF target_pdf_aux_;
		target_pdf_aux_.state = dekf_algorithm_.getAuxTargetState();	
		target_pdf_aux_.cov   = dekf_algorithm_.getAuxTargetCovariance();
		medusa_slap_msg::TargetPDF msg_target_pdf_aux;
		msg_target_pdf_aux.Veh_ID = Veh_ID; 
		for(int i = 0; i < 4; i++) {
     		msg_target_pdf_aux.state[i]    = target_pdf_aux_.state[i];
         	msg_target_pdf_aux.cov_row1[i] = target_pdf_aux_.cov(0,i);
        	msg_target_pdf_aux.cov_row2[i] = target_pdf_aux_.cov(1,i); 
	 		msg_target_pdf_aux.cov_row3[i] = target_pdf_aux_.cov(2,i); 
        	msg_target_pdf_aux.cov_row4[i] = target_pdf_aux_.cov(3,i); 
        } 
		msg_target_pdf_local.time = time;
		target_pdf_aux_pub_.publish(msg_target_pdf_aux) ;
 }

 /*
#######################################################################################################################
 @.@ Callbacks
 #######################################################################################################################
 */
 
 void DekfNode::targetPdfFromNeighborCallback(const medusa_slap_msg::TargetPDF& msg){
	if (!received_target_pdf_from_neighbor) {
		received_target_pdf_from_neighbor = true;
		ROS_INFO("dekf node has received the first target's pdf from neighbor"); 	
	}
	Eigen::Vector4d neighbor_target_info_vector;
	Eigen::Matrix4d neighbor_target_info_matrix;
	for(int i = 0; i < 4; i++) {
		neighbor_target_info_vector[i] = msg.state[i];
		neighbor_target_info_matrix(0,i) = msg.cov_row1[i];
		neighbor_target_info_matrix(1,i) = msg.cov_row2[i]; 
		neighbor_target_info_matrix(2,i) = msg.cov_row3[i]; 
		neighbor_target_info_matrix(3,i) = msg.cov_row4[i]; 
	}
	int neighbor_ID = msg.Veh_ID;
	// std::cout << "vector" << neighbor_target_info_vector << std::endl;
	// std::cout << "matrix" << neighbor_target_info_matrix << std::endl; 
	// std::cout << "augmented matrix" << augmented_target_info_matrix_ << std::endl;

	augmented_target_info_vector_.block<4,1>(0,neighbor_ID) = neighbor_target_info_vector; 
	augmented_target_info_matrix_.block<4,4>(0,4*neighbor_ID) = neighbor_target_info_matrix;

	 if (dekf_enable_) {
	  	dekf_algorithm_.fuseWithNeighbor(augmented_target_info_vector_, augmented_target_info_matrix_);
	 }
 }
 void DekfNode::targetRangeCallback(const farol_msgs::mUSBLFix& msg){

	double range_ = msg.range;
	std::cout << "range received is: " << range_ << std::endl;
	if (!received_target_range ) {
		received_target_range = true;	
		ROS_INFO("dekf node has received the first range to target"); 	
	}
	if (ekf_enable_) {	
		int msg_type = msg.type;
		if (msg_type == 0)
		{
			if (checkOutlier(range_)){
				ROS_INFO("measured range is outlier, discarded"); 					// a navier way to reject outliner (for test only), not standard method
			}
			else{
				dekf_algorithm_.localCorrection(range_, vehicle_position_);
			}
		}
	}

	}
 void DekfNode::neighborPositionCallback(const medusa_slap_msg::VehiclePosVel& msg){
	if (!received_neighbor_position ) {
		received_neighbor_position = true;
		ROS_INFO("dekf node has received the first position of neighbor"); 	
	}
	/* if neighbor is myellow (ID=2) the it plays the role of target */
	int neighbor_ID = msg.Veh_ID;

	if ((neighbor_ID!=100) && (neighbor_ID!=Veh_ID) ){
		// this is neighbor tracker
		vehicle2_position_[0] = msg.position[0];
		vehicle2_position_[1] = msg.position[1];
	}

	if (neighbor_ID == 100){
		// ID from 100 is used for targets
		target_position_[0] = msg.position[0];
		target_position_[1] = msg.position[1]; 
		target_velocity_[0] = msg.velocity[0];
		target_velocity_[1] = msg.velocity[1]; 
	}

 }
 void DekfNode::vehicleStateCallback(const auv_msgs::NavigationStatus &msg) {
	if ( !received_vehicle_state ){
		received_vehicle_state = true;
		ROS_INFO("Dekf node has received the first vehicle state"); 	

	}
	vehicle_position_ << msg.position.north - state_offset_vector_[0], msg.position.east - state_offset_vector_[1];
	
 }
void DekfNode::smoothTargetState(Eigen::Vector4d current_target_state) {
	 smoothed_target_state_ = 0.9*smoothed_target_state_ + 0.1*current_target_state;
} 
bool DekfNode::checkOutlier(double range) {
	double a1, a2, b0, threadshod;
	bool outlier;
	static double pre_range_old, pre_range_old_old;
	if (isnan(pre_range_old)){
		pre_range_old = range;
		pre_range_old_old = range;
	}

	a1 = 0.45; a2 = 0.1; b0 = 0.45; threadshod = 20;
    // predict range    
    double pre_range = a2*pre_range_old_old + a1*pre_range_old + b0*range;
//	std::cout << "predicted_range" << pre_range <<  std::endl; 
	if (abs(pre_range - range) > threadshod){
		a1 = 0.8; a2 = 0.199; b0 = 0.001;
        pre_range = a2*pre_range_old_old + a1*pre_range_old + b0*range ;  	
		outlier = true ;
	}
	else{
		outlier = false ;
	}

	pre_range_old_old  = pre_range_old;
	pre_range_old = pre_range;

return outlier;
} 

 /*
#######################################################################################################################
 @.@ Main
 #######################################################################################################################
 */
 int main(int argc, char** argv)
 {
 	// +.+ ROS set-ups:
 	ros::init(argc, argv, "dekf_node"); //node name
 	// +.+ create a node handle; need to pass this to the class constructor
 	ros::NodeHandle nh;

 	ros::NodeHandle nh_p("~");

 	ROS_INFO("main: instantiating an object of type DekfNode");

 	// +.+ instantiate an DekfNode class object and pass in pointer to nodehandle for constructor to use
 	DekfNode dekf(&nh,&nh_p);

 	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
 	ros::spin();

 	return 0;
 }

