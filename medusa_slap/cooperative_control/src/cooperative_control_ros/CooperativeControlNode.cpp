/**
 * @brief this code is for cooperative control node
 * @author  DSOR team
 * @author  Persons in charges: Hung
 * @date    2021
 */

#include "CooperativeControlNode.h"
#include "CooperativeControlAlgorithm.h"

/**
  * @brief Constructor, will be called when an object is instantiated
 */
 CooperativeControlNode::CooperativeControlNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
						:nh_(*nh), nh_p_(*nh_p) {
 	ROS_INFO("in class constructor of CooperativeControlNode");
 	loadParams();
 	initializeSubscribers();
 	initializePublishers();
 	initializeTimer();
	initializeServices(); 

 }

 /**
  * @brief Destructor
 */
 CooperativeControlNode::~CooperativeControlNode() {

 	// +.+ stop timer
 	timer_.stop();

 	// +.+ shutdown node
 	nh_.shutdown();
 }

 /**
  * @brief Initialize subscribers
  */
 void CooperativeControlNode::initializeSubscribers() {
 	ROS_INFO("Initializing Subscribers for CooperativeControlNode");

/* These topic names later can be gotten from config file */
	
	//std::string internal_gamma_topic_name =  "/mblack/slap/internal/gamma";
	std::string internal_gamma_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_,"topics/subscribers/internal_gamma");
	//std::string neighbor_gamma_topic_name =  "/mblack/slap/neighbor/gamma";
	std::string neighbor_gamma_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_,"topics/subscribers/neighbor_gamma");

	internal_gamma_sub_ = nh_.subscribe(internal_gamma_topic_name, 10, &CooperativeControlNode::internalGammaCallback, this);
	neighbor_gamma_sub_ = nh_.subscribe(neighbor_gamma_topic_name, 10, &CooperativeControlNode::neighborGammaCallback, this);

 }

 /**
  Member helper function to set up publishers;
 */
 void CooperativeControlNode::initializePublishers() {
 	ROS_INFO("Initializing Publishers for CooperativeControlNode"); 	// ---> add publishers here

	/* These topics names later can be get from config file */  
	//std::string vc_topic_name =  "/mblack/slap/internal/vc";
	std::string vc_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_,"topics/publishers/vc");
	//std::string gamma_to_neighbor_topic_name =  "/mblack/slap/gamma_to_neighbor";
	std::string gamma_to_neighbor_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_,"topics/publishers/to_neighbor_gamma"); 

	std::string cpf_etc_info_name = FarolGimmicks::getParameters<std::string>(nh_p_,"topics/publishers/cpf_etc_info"); 

	/* Initialize the publisher */
	
  	vc_pub_ = nh_.advertise<std_msgs::Float64>(vc_topic_name, 1);
  	to_neighbor_gamma_pub_ = nh_.advertise<farol_msgs::CPFGamma>(gamma_to_neighbor_topic_name, 1);
	cpf_etc_info_pub_ = nh_.advertise<medusa_slap_msg::ETCInfo>(cpf_etc_info_name, 1);

 }

 /*
#######################################################################################################################
 @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
 #######################################################################################################################
 */
 void CooperativeControlNode::initializeTimer() {
 	timer_ =nh_.createTimer(ros::Duration(1.0/CooperativeControlNode::nodeFrequency()), &CooperativeControlNode::timerIterCallback, this);
 	timer_.stop();
 }

 /*
#######################################################################################################################
 @.@ Set frequency of the node default is 2
 #######################################################################################################################
 */
 double CooperativeControlNode::nodeFrequency()
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
 void CooperativeControlNode::loadParams() {
 	ROS_INFO("Load the CooperativeControlNode parameters");
	
	std::vector<int> adj_matrix;
	double kc;
   	nh_p_.getParam("kc", kc);
    cooperative_control_algorithm_.updateGains(kc);


	nh_p_.getParam("Veh_ID", Veh_ID);
	nh_p_.getParam("adj_matrix", adj_matrix);

	int num_vehicles = sqrt(adj_matrix.size());
    neighbor_set_.resize(num_vehicles);	 
	gamma_vector_.resize(num_vehicles);
	gamma_desired_speed_vector_.resize(num_vehicles);

	cooperative_control_algorithm_.updateGammaVector(gamma_vector_);
	cooperative_control_algorithm_.updateGammaDesiredSpeedVector(gamma_desired_speed_vector_);


	for (int i = 0; i < num_vehicles; i++){
		neighbor_set_[i] = adj_matrix[num_vehicles*Veh_ID + i];
	}
			
	cooperative_control_algorithm_.updateNeighborSet(neighbor_set_);
	cooperative_control_algorithm_.updateVehicleID(Veh_ID);

	std::cout << "print gamma_vector: " << gamma_vector_ << std::endl;	
	cooperative_control_algorithm_.setEstGamma(gamma_vector_);

	// for(int i=0; i<3; i++){
	// 	neighbor_set[i] = adj_matrix[i+(Veh_ID-1)*num_vehicles];
	// }	
	// cooperative_control_algorithm_.updateNeighborSet(neighbor_set);
 }

/*
 	timmer callback acting as as a for loop - repeating every time interval  
 */
void CooperativeControlNode::timerIterCallback(const ros::TimerEvent &event) {
		// std::cout << "gamma vector_:" << gamma_vector_ << std::endl;
		// std::cout << "gamma desired speed vector_:" << gamma_desired_speed_vector_ << std::endl;

		cooperative_control_algorithm_.updateGammaVector(gamma_vector_);
		cooperative_control_algorithm_.updateGammaDesiredSpeedVector(gamma_desired_speed_vector_);
	 if (received_internal_gamma && received_neighbor_gamma){	 
 	/* Update internal and neighbor gamma */
		// cooperative_control_algorithm_.updateInternalGamma(internal_gamma_);
		// cooperative_control_algorithm_.updateNeighborGamma(neighbor_gamma_);

	/* Call cooperative controller */
		cooperative_control_algorithm_.callCooperativeController(0.1);
	/* Get the correction speed to be published */
		vc_ = cooperative_control_algorithm_.getCorrectionSpeed();
	
	/* Publish vc */
   		FarolGimmicks::publishValue<std_msgs::Float64, const double>(vc_pub_,vc_);
	 } 

	/* Broadcast the latest internal gamma to neighboring vehicle, this depends on triggering mechanism */

	double time = abs((ros::Time::now()- initial_time).toSec());
	// std::cout << "current time:" << time << std::endl;
	EtcInfo etc_info_ = cooperative_control_algorithm_.checkBroadcastSignal(time);
	medusa_slap_msg::ETCInfo msg_cpf_etc;
	msg_cpf_etc.broadcast_signal = etc_info_.broadcast_signal;
	msg_cpf_etc.threshold = etc_info_.threshold;
	msg_cpf_etc.error = etc_info_.est_error;
	cpf_etc_info_pub_.publish(msg_cpf_etc);
	// std::cout << "current time next:" << time << std::endl;

	if(etc_info_.broadcast_signal) {
			/* publish the latest internal gamma to the network */
			farol_msgs::CPFGamma msg;

			msg.header.seq = seq_;
			seq_++;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "";
			msg.gamma = internal_gamma_; 
			msg.vd = internal_gamma_desired_speed_; 
			to_neighbor_gamma_pub_.publish(msg);
 	}	
 }
 /*
#######################################################################################################################
 @.@ Callback 
 #######################################################################################################################
 */
 
void CooperativeControlNode::internalGammaCallback(const farol_msgs::CPFGamma& msg){
  if (!received_internal_gamma ) {
    received_internal_gamma = true;
	ROS_INFO("Cooperative controller node has received the first internal gamma"); 	
  }
	internal_gamma_ = msg.gamma;
	internal_gamma_desired_speed_ = msg.vd;
	gamma_vector_[Veh_ID] = internal_gamma_;
	gamma_desired_speed_vector_[Veh_ID] = internal_gamma_desired_speed_;
}
void CooperativeControlNode::neighborGammaCallback(const farol_msgs::CPFGamma& msg){
  if (!received_neighbor_gamma ) {
    received_neighbor_gamma = true;
	ROS_INFO("Cooperative controller node has received the first neighbor gamma"); 	
  }	
	double neighbor_gamma = msg.gamma;
	double neighbor_gamma_desired_speed = msg.vd ;
	int neighbor_ID = msg.ID;
    
	gamma_vector_[neighbor_ID] = neighbor_gamma;
	gamma_desired_speed_vector_[neighbor_ID] = neighbor_gamma_desired_speed;

	cooperative_control_algorithm_.resetNeighborEstGamma(gamma_vector_[neighbor_ID],neighbor_ID);

}



/*
#######################################################################################################################
 @.@ Main
 #######################################################################################################################
 */
 int main(int argc, char** argv)
 {
 	// +.+ ROS set-ups:
 	ros::init(argc, argv, "cooperative_control_node"); //node name
 	// +.+ create a node handle; need to pass this to the class constructor
 	ros::NodeHandle nh;

 	ros::NodeHandle nh_p("~");

 	ROS_INFO("main: instantiating an object of type CooperativeControlNode");

 	// +.+ instantiate an CooperativeControlNode class object and pass in pointer to nodehandle for constructor to use
 	CooperativeControlNode cooperativeControl(&nh,&nh_p);

 	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
 	ros::spin();

 	return 0;
 }

