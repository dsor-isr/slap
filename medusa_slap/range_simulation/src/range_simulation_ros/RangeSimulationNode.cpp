/**
 * @brief This is RangeSimulationNode 
 * @author  DSOR team
 * @author  Persons in charges: Hung, Quintas, Cruz
 * @date    2021
 */
 // this header incorporates all the necessary #include files and defines the class "RangeSimulationNode"
 #include "RangeSimulationNode.h"
 #include "RangeSimulationAlgorithm.h"

 /*
#######################################################################################################################
 @.@ CONSTRUCTOR: put all dirty work of initializations here
 Note the odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
 #######################################################################################################################
 */
 RangeSimulationNode::RangeSimulationNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_p_(*nodehandle_private) {
 	ROS_INFO("in class constructor of RangeSimulationNode");
 	loadParams();
 	initializeSubscribers();
 	initializePublishers();
 	initializeTimer();

	std::normal_distribution<double> dist(mean,stddev);

 }

 /*
#######################################################################################################################
 @.@ Destructor
 #######################################################################################################################
 */
 RangeSimulationNode::~RangeSimulationNode() {

 	// +.+ stop timer
 //	timer_.stop();

 	// +.+ shutdown node
 	nh_.shutdown();
 }

 void RangeSimulationNode::initializeSubscribers() {
 	ROS_INFO("Initializing Subscribers for RangeSimulationNode");
 	
	// NOTE: neighbor here means target's vehicle 
	std::string target_position_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/neighbor_pos");
	std::string vehicle_state_topic_name =  FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/vehicle_state");
 
 	target_pos_sub_ = nh_.subscribe(target_position_topic_name, 10, &RangeSimulationNode::targetPositionCallback, this);;
	vehicle_state_sub_ = nh_.subscribe(vehicle_state_topic_name, 10, &RangeSimulationNode::vehicleStateCallback, this);
 
 }

 /*
#######################################################################################################################
 @.@ Member helper function to set up publishers;
 #######################################################################################################################
 */
 void RangeSimulationNode::initializePublishers() {
 	ROS_INFO("Initializing Publishers for RangeSimulationNode"); 	// ---> add publishers here
    std::string target_range_topic_name = FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/target_range");
    target_range_pub_ = nh_.advertise<farol_msgs::mUSBLFix>(target_range_topic_name,10);  

  }
 
 void RangeSimulationNode::initializeTimer() {
 	timer_ =nh_.createTimer(ros::Duration(1.0/RangeSimulationNode::nodeFrequency()), &RangeSimulationNode::timerIterCallback, this);
 //	timer_.stop();
 }

 /*
#######################################################################################################################
 @.@ Set frequency of the node default is 2
 #######################################################################################################################
 */
 double RangeSimulationNode::nodeFrequency()
 {
 	double node_frequency;
 	nh_p_.param("node_frequency", node_frequency, 0.66);
 	ROS_INFO("Node will run at : %lf [hz]", node_frequency);
 	return node_frequency;
 }

 /*
#######################################################################################################################
 @.@ Load the parameters
 #######################################################################################################################
 */
 void RangeSimulationNode::loadParams() {
 	ROS_INFO("Load the RangeSimulationNode parameters");
 	//---> params here, always p_paramName
 	//Example: nh_.param("/RangeSimulationNode/Ku", p_ku, 0.5);
 }

 /*
#######################################################################################################################
 @.@ Callbacks Section / Methods
 #######################################################################################################################
 */

 void RangeSimulationNode::targetPositionCallback(const medusa_slap_msg::VehiclePosVel& msg){
	
	/* ID of target is assigned as 100 */
	int target_ID = msg.Veh_ID;

	if (!received_target_position && target_ID == 100 ) {
		received_target_position = true;
		ROS_INFO("range simulation node has received the first position of target"); 	
	}

	if (target_ID == 100){
		// ID from 100 is used for targets
		target_position_[0] = msg.position[0];
		target_position_[1] = msg.position[1]; 
	}
 }
 void RangeSimulationNode::vehicleStateCallback(const auv_msgs::NavigationStatus &msg) {

	if ( !received_vehicle_state ){
		received_vehicle_state = true;
		ROS_INFO("range simulation node has received the first vehicle state"); 	

	}
	vehicle_position_ << msg.position.north, msg.position.east;
 }


 /*
#######################################################################################################################
 @.@ Iteration via timer callback
 #######################################################################################################################
 */
 void RangeSimulationNode::timerIterCallback(const ros::TimerEvent &event) {
	 	
	if (!received_target_position ||  !received_vehicle_state)
	return;

	double tem = (vehicle_position_ - target_position_).norm(); 
	double simulated_range_  = sqrt(tem*tem + target_depth_*target_depth_) + 0.1*dist(generator);
	// simulate outlier
		if (dist(generator) > 140*stddev){
			simulated_range_=+ 500+100*dist(generator);			// just add a crazy number to the range
		}
	farol_msgs::mUSBLFix msg;
	msg.range = simulated_range_;
	msg.type = 0;
	ROS_INFO("Range to the target is: %f", simulated_range_);
	target_range_pub_.publish(msg); 


 }
 /*
#######################################################################################################################
 @.@ Callback Flag
 #######################################################################################################################
 */

 /*
#######################################################################################################################
 @.@ Main
 #######################################################################################################################
 */
 int main(int argc, char** argv)
 {
 	// +.+ ROS set-ups:
 	ros::init(argc, argv, "range_simulation_node"); //node name
 	// +.+ create a node handle; need to pass this to the class constructor
 	ros::NodeHandle nh;

 	ros::NodeHandle nh_p("~");

 	ROS_INFO("main: instantiating an object of type RangeSimulationNode");

 	// +.+ instantiate an RangeSimulationNode class object and pass in pointer to nodehandle for constructor to use
 	RangeSimulationNode rangeSimulation(&nh,&nh_p);

 	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
 	ros::spin();

 	return 0;
 }