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

 #include "DekfAlgorithm.h"

/* The constructor for the DekfAlgorithm Path Following Controller */
DekfAlgorithm::DekfAlgorithm() {
   Q_.diagonal() << 0.01, 0.01, 0.001, 0.001;
   R_.resize(1,1);
   R_ << 1;
   setMatricesWV(Q_,R_);

// Target model 
  double dt = 0.1;
  F_ << 1, 0, dt, 0,
	    0, 1, 0,  dt,
        0, 0, 1,  0,
        0, 0, 0,  1;
}


void DekfAlgorithm::setVehicleID(int veh_id){
	Veh_ID = veh_id;
}
void DekfAlgorithm::setInternalTargetPDF(TargetPDF internal_target_pdf){
	target_pdf_.state = internal_target_pdf.state;
	target_pdf_.cov = internal_target_pdf.cov;
	
	target_pdf_.info_matrix = target_pdf_.cov.inverse();
	target_pdf_.info_vector= target_pdf_.info_matrix*target_pdf_.state; 
}
void DekfAlgorithm::callCentralEKF(double range1, double range2, Eigen::Vector2d veh1_pos, Eigen::Vector2d veh2_pos){

	counter_ += 1;

   	if (!(counter_ % 10)){

	std::cout << "range1: " << range1 << std::endl;
	std::cout << "range2: " << range2 << std::endl;
	std::cout << "veh1_pos: " << veh1_pos << std::endl;
	std::cout << "veh2_pos: " << veh2_pos << std::endl;
		Eigen::Vector2d range_vector_;
		range_vector_ << range1, range2;
		Eigen::Vector2d target_position_hat_;
		Eigen::MatrixXd C_(2,4); 
		double range1_hat_;
		double range2_hat_;

		Eigen::Vector2d resid;
		Eigen::Vector2d range_hat_vector_;
		Eigen::Matrix2d V_;
		V_ << 1, 0,
		      0, 1;
		
		target_position_hat_ << target_pdf_.state[0], target_pdf_.state[1];

		double tem1 = (veh1_pos - target_position_hat_).norm();
		double tem2 = (veh2_pos - target_position_hat_).norm();
		range1_hat_ = sqrt(tem1*tem1 + target_depth_*target_depth_);
		range2_hat_ = sqrt(tem2*tem2 + target_depth_*target_depth_);
	    range_hat_vector_ << range1_hat_, range2_hat_;

		double c11 = (target_position_hat_[0] - veh1_pos[0])/range1_hat_;
		double c12 = (target_position_hat_[1] - veh1_pos[1])/range1_hat_;
		double c21 = (target_position_hat_[0] - veh2_pos[0])/range2_hat_;
		double c22 = (target_position_hat_[1] - veh2_pos[1])/range2_hat_;
		
		C_ << c11, c12, 0, 0,
		      c21, c22, 0, 0;

		resid = range_vector_ - range_hat_vector_ + C_*target_pdf_.state;

        target_pdf_.info_vector +=  C_.transpose()*V_*resid;
        target_pdf_.info_matrix +=  C_.transpose()*V_*C_;
 
	}
 
     // Prediction
    target_pdf_.state = target_pdf_.info_matrix.inverse()*target_pdf_.info_vector;
	
	target_pdf_.state = F_*target_pdf_.state;
	target_pdf_.info_matrix =  W_ - W_*F_*(target_pdf_.info_matrix + F_.transpose()*W_*F_).inverse()*F_.transpose()*W_;
    
    target_pdf_.info_vector = target_pdf_.info_matrix*target_pdf_.state;
	target_pdf_.cov = target_pdf_.info_matrix.inverse();			

}


Eigen::Vector4d DekfAlgorithm::getTargetInfoVector(){
 return target_pdf_.info_vector;
} 
Eigen::Matrix4d DekfAlgorithm::getTargetInfoMatrix(){
 return target_pdf_.info_matrix; 
}
Eigen::Vector4d DekfAlgorithm::getTargetState(){
 return target_pdf_.state;
} 
Eigen::Matrix4d DekfAlgorithm::getTargetCovariance(){
 return target_pdf_.cov; 
}
 
 
/* Method that will run in the first iteration of the algorithm */
void DekfAlgorithm::start() {
  
}

/* Method used to check whether we reached the end of the algorithm or not */
bool DekfAlgorithm::stop() {

  return false;
}
/* Method to reset all the algorithm data when the path following restarts */
bool DekfAlgorithm::reset() {
  
  
  return true;
}

EtcInfo DekfAlgorithm::checkBroadcastSignal(double time){
	/* check KLD between the two pdf */
	double n = 4; // dimension of the state (position and velocity in 2D)
//	std::cout << "c0: " << c0_ << std::endl;
	etc_info_.threshold = c0_ + c1_*std::exp(-c2_*time);			// compute threshold 
    etc_info_.kld = (est_target_pdf_.info_matrix*target_pdf_.cov).trace()+
	            (est_target_pdf_.state - target_pdf_.state).transpose()*est_target_pdf_.info_matrix*(est_target_pdf_.state - target_pdf_.state)
				 + std::log(target_pdf_.info_matrix.determinant()/est_target_pdf_.info_matrix.determinant())- n;

//	std::cout << "determinant of information matrix: " << target_pdf_.info_matrix.determinant() << std::endl;
//	std::cout << "determinant of estimation information matrix: " << est_target_pdf_.info_matrix.determinant() << std::endl;
			 
	if ((abs(etc_info_.kld) >= etc_info_.threshold ) || (isnan(etc_info_.kld))){
		etc_info_.broadcast_signal = true;
		resetEstTargetPDF();
	//	propagateEstTargetPDF();		
	}
	else{
		etc_info_.broadcast_signal = false;
	}
	propagateEstTargetPDF();		
//	std::cout << "etc_info.broadcast_sig: " << etc_info_.broadcast_signal << std::endl;

	return etc_info_;
}
void DekfAlgorithm::setNeighborSet(Eigen::VectorXi neighbor_set){
	neighbor_set_ = neighbor_set;
}
void DekfAlgorithm::setTargetDepth(double target_depth){
	target_depth_ = target_depth;
}

void DekfAlgorithm::localCorrection(double range, Eigen::Vector2d vehicle_position){
    // std::cout << " Before Local correction target state" << target_pdf_.state << std::endl;
	// std::cout << "Before Local correction target cov" << target_pdf_.cov << std::endl;
	// std::cout << "Before Local correction target info vector" << target_pdf_.info_vector << std::endl;
	// std::cout << "Before Local correction target info matrix" << target_pdf_.info_matrix << std::endl;
//		std::cout << " local correction" << std::endl; 
	    ROS_INFO("target depth is: %f", target_depth_);

		Eigen::Vector2d target_position_hat_;
		Eigen::MatrixXd C_(1,4); 
		double range_2D_hat_;
		double range_2D_meas;
		Eigen::MatrixXd resid(1,1);
		
		target_position_hat_ << target_pdf_.state[0], target_pdf_.state[1];

		range_2D_hat_ = (vehicle_position - target_position_hat_).norm();
		range_2D_meas = sqrt(range*range - target_depth_*target_depth_);
	    
		C_ << (target_position_hat_[0] - vehicle_position[0])/range_2D_hat_,
			  (target_position_hat_[1] - vehicle_position[1])/range_2D_hat_,
			   0,
			   0;
		resid << range_2D_meas - range_2D_hat_ + C_.row(0)*target_pdf_.state;

        target_pdf_.info_vector +=  C_.transpose()*V_*resid;
        target_pdf_.info_matrix +=  C_.transpose()*V_*C_;

		target_pdf_.cov = target_pdf_.info_matrix.inverse();	
		target_pdf_.state = target_pdf_.cov*target_pdf_.info_vector;


	// std::cout << "Local correction target state" << target_pdf_.state << std::endl;
	// std::cout << "Local correction target cov" << target_pdf_.cov << std::endl;
	// std::cout << "Local correction target info vector" << target_pdf_.info_vector << std::endl;
	// std::cout << "Local correction target info matrix" << target_pdf_.info_matrix << std::endl;


}
void DekfAlgorithm::localPrediction(){
//	std::cout << "local prediction" << std::endl;  
	target_pdf_.state = F_*target_pdf_.state;
	target_pdf_.info_matrix =  W_ - W_*F_*(target_pdf_.info_matrix + F_.transpose()*W_*F_).inverse()*F_.transpose()*W_;
    
    target_pdf_.info_vector = target_pdf_.info_matrix*target_pdf_.state;
	target_pdf_.cov = target_pdf_.info_matrix.inverse();	
}
void DekfAlgorithm::fuseWithNeighbor(Eigen::MatrixXd augmented_target_info_vector, Eigen::MatrixXd augmented_target_info_matrix){
	
//	std::cout << "fuse with neighbor" << std::endl;  

	Eigen::Vector4d sum_target_info_vector_;
	Eigen::Matrix4d sum_target_info_matrix_;
	sum_target_info_vector_ = target_pdf_.info_vector;
	sum_target_info_matrix_ = target_pdf_.info_matrix;

	std::cout << "local information vector: " << std::endl;
	std::cout << target_pdf_.info_vector<< std::endl;
    std::cout << "neighbor information vector: " << std::endl; 
	std::cout << augmented_target_info_vector<< std::endl;

	std::cout << "local information matrix:" << std::endl;
	std::cout << target_pdf_.info_matrix<< std::endl;
    std::cout << "neighbor information matrix:" << std::endl; 
	std::cout <<  augmented_target_info_matrix << std::endl;

	// std::cout << "matrix" << neighbor_target_info_matrix << std::endl; 
	// std::cout << "augmented matrix" << augmented_target_info_matrix_ << std::endl;
	
	for(int i=0; i < neighbor_set_.size(); i++){
		if (i!=Veh_ID) {
			sum_target_info_vector_ += neighbor_set_[i]*augmented_target_info_vector.col(i);
			sum_target_info_matrix_ += neighbor_set_[i]*augmented_target_info_matrix.block<4,4>(0,4*i);
		}
	}
	
	target_pdf_.info_vector = sum_target_info_vector_/(neighbor_set_.sum()+1);
	target_pdf_.info_matrix = sum_target_info_matrix_/(neighbor_set_.sum()+1);

	target_pdf_.state = target_pdf_.info_matrix.inverse()*target_pdf_.info_vector;
	target_pdf_.cov = target_pdf_.info_matrix.inverse();	

}
void DekfAlgorithm::setMatricesWV(Eigen::MatrixXd Q, Eigen::MatrixXd R){
   
	// double Q_scale_ = 1e-5;
    // double R_scale_ = 1e-4; 

	// new 

	double Q_scale_ = 1;
    double R_scale_ = 1;    

    Q_ = Q*Q_scale_;
	R_ = R*R_scale_; 

	W_.diagonal() <<  1/Q_(0,0), 1/Q_(1,1), 1/Q_(2,2), 1/Q_(3,3) ;
	V_.resize(1,1);
	V_ <<  1/R_(0,0);

//    std::cout << "matrix W " << W_ << std::endl;
//    std::cout << "matrix V " << V_ << std::endl; 
}

void DekfAlgorithm::resetEstTargetPDF(){
	est_target_pdf_.state = target_pdf_.state;
	est_target_pdf_.cov = target_pdf_.cov; 
	est_target_pdf_.info_matrix = target_pdf_.info_matrix;
}
void DekfAlgorithm::propagateEstTargetPDF(){
	est_target_pdf_.state = F_*est_target_pdf_.state;
	est_target_pdf_.cov = F_*est_target_pdf_.cov*F_.transpose() + Q_;
    est_target_pdf_.info_matrix = est_target_pdf_.cov.inverse();
	  
}

void DekfAlgorithm::setETCParameters(double c0, double c1, double c2){
	c0_ = c0;
	c1_ = c1;
	c2_ = c2;
	std::cout << "c0 in setETCParameters: " << c0_ << std::endl;

}

void DekfAlgorithm::setEstTargetPDF(){
		
	/* set estimate of internal target pdf - this is for ETC mechanism */ 
	est_target_pdf_.state = target_pdf_.state;
	est_target_pdf_.cov = target_pdf_.cov;
	
	est_target_pdf_.info_matrix = target_pdf_.info_matrix;

    // std::cout << "determinant of information matrix: " << target_pdf_.info_matrix.determinant() << std::endl;
	// std::cout << "determinant of estimation information matrix: " << est_target_pdf_.info_matrix.determinant() << std::endl;

}

// void DekfAlgorithm::setRangeToTarget(double range){
// range_ = range;
// }
// void DekfAlgorithm::setDekfMode(bool dekf_mode){
// 	dekf_mode_ = dekf_mode;
// }
// bool DekfAlgorithm::getDekfMode(){
// 	return dekf_mode_;
// }
// void DekfAlgorithm::updateAugmentedTargetInfoVector(Eigen::MatrixXd target_info_vector){
// 	augmented_target_info_vector_ = target_info_vector;
// }
// void DekfAlgorithm::updateAugmentedTargetInfoMatrix(Eigen::MatrixXd target_info_matrix){
// 	augmented_target_info_matrix_ = target_info_matrix;
// }
// void DekfAlgorithm::updateVehiclePosition(Eigen::Vector2d vehicle_position){
// 	vehicle_position_ = vehicle_position;
// }