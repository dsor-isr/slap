/**
 * @brief this code is for cooperative control algorithm
 * @author  DSOR team
 * @author  Persons in charges: Hung
 * @date    2021
 */

#include "CooperativeControlAlgorithm.h"

/**
* @brief  Constructor  
*/
CooperativeControlAlgorithm::CooperativeControlAlgorithm(){
    etc_info_.broadcast_signal = true;
}
/**
* @brief  Update the vehicle ID  
*/
void CooperativeControlAlgorithm::updateVehicleID(int veh_id){
    Veh_ID_=veh_id;
}
/**
* @brief  Update gamma of the vehicle  
*/
void CooperativeControlAlgorithm::updateInternalGamma(double internal_gamma){
    internal_gamma_ = internal_gamma;
}
/**
* @brief  Update gamma from neighbor vehicles 
*/
void CooperativeControlAlgorithm::updateNeighborGamma(double neighbor_gamma){
    neighbor_gamma_ = neighbor_gamma;
}
/**
* @brief  Update neighbor set
*/
void CooperativeControlAlgorithm::updateNeighborSet(Eigen::VectorXi neighbor_set){
    neighbor_set_ = neighbor_set;
}
void CooperativeControlAlgorithm::updateGammaVector(Eigen::VectorXd gamma_vector){
    gamma_vector_ = gamma_vector;
}

void CooperativeControlAlgorithm::updateGammaDesiredSpeedVector(Eigen::VectorXd gamma_desired_speed_vector){
    gamma_desired_speed_vector_ = gamma_desired_speed_vector;
}

/**
* @brief  Update cooperative control gain
*/
void CooperativeControlAlgorithm::updateGains(double kc){
    kc_ = kc;
}

/**
* @brief  Compute correction speed
*/
void CooperativeControlAlgorithm::callCooperativeController(double dt){
    // vc_ = - kc_*(internal_gamma_ - neighbor_gamma_);
    if (gain_scale_ <=1){ 
      gain_scale_ += 0.001;      // just a simple technique to increase the gains of the controller slowly, avoid vehicle over-reacts when it start
    }
    double consensus_error_{0};
    for(int i=0; i < neighbor_set_.size(); i++){
        consensus_error_ += neighbor_set_[i]*(gamma_vector_[Veh_ID_] - gamma_vector_est_[i] );
    }
    vc_ = -gain_scale_*kc_*tanh(consensus_error_);   
}
/**
* @brief   Get correction speed to be published
*/
double CooperativeControlAlgorithm::getCorrectionSpeed(){
    return vc_;
}
EtcInfo CooperativeControlAlgorithm::checkBroadcastSignal(double time){
    // std::cout << "print estimation error: " << etc_info_.est_error << std::endl; 
    // std::cout << "print gamma_vector: " << gamma_vector_ << std::endl; 
    // std::cout << "print gamma_vector_est: " << gamma_vector_est_ << std::endl; 
    // std::cout << "print gamma_desired_speed_vector_est: " << gamma_desired_speed_vector_ << std::endl; 
    etc_info_.threshold = c0_ + c1_*std::exp(-c2_*time);			// compute threshold 
    etc_info_.est_error = abs(gamma_vector_[Veh_ID_] - gamma_vector_est_[Veh_ID_]); 
    if ((etc_info_.est_error >= etc_info_.threshold ) || isnan(etc_info_.est_error)){
		etc_info_.broadcast_signal = true;
		resetInternalEstGamma();
	}
	else{
		etc_info_.broadcast_signal = false;
	}
    predictEstGamma(0.1);		
//	propagateEstTargetPDF();		
//	std::cout << "etc_info.broadcast_sig: " << etc_info_.broadcast_signal << std::endl;

    return etc_info_ ;
};  
bool CooperativeControlAlgorithm::reset(){
    gain_scale_ = 0.0;
    vc_ = 0.0;
};
void CooperativeControlAlgorithm::setETCParameters(double c0, double c1, double c2){
	c0_ = c0;
	c1_ = c1;
	c2_ = c2;
	std::cout << "c0 in setETCParameters: " << c0_ << std::endl;
}
void CooperativeControlAlgorithm::resetInternalEstGamma(){
	gamma_vector_est_[Veh_ID_] = gamma_vector_[Veh_ID_];
}
void CooperativeControlAlgorithm::resetNeighborEstGamma(double neighbor_gamma, int neighbor_id){
	gamma_vector_est_[neighbor_id] = neighbor_gamma;
}

void CooperativeControlAlgorithm::predictEstGamma(double dt){
	for(int i=0; i < neighbor_set_.size(); i++){
        gamma_vector_est_[i] += dt*gamma_desired_speed_vector_[i];
    }
    
}

void CooperativeControlAlgorithm::setEstGamma(Eigen::VectorXd gamma_vector){
        gamma_vector_est_ = gamma_vector;    
}

