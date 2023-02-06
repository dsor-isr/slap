
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
 * @author    Persons in charge: Hung, Quintas, Cruz
 * @version   1.0a
 * @date      2021
 * @copyright GPLv3
 */

#pragma once
/* ROS includes for publishing the values and setting the mode of operation */
#include "ros/ros.h"
#include <std_msgs/Float64.h>

/* Contains auxiliary functions for angle wrap-up and others */
#include <farol_gimmicks_library/FarolGimmicks.h>

struct TargetPDF {
  Eigen::Vector4d state;          
	Eigen::Matrix4d cov;
  Eigen::Vector4d info_vector;
  Eigen::Matrix4d info_matrix;
};
struct EtcInfo {
  double kld;               // Kullback–Leibler divergence        
	double threshold;
  bool broadcast_signal;
};

class DekfAlgorithm {

  public:

    /**
     * @brief  Constructor method for the class
     */
    DekfAlgorithm();

    /**
     *  @brief Methods to update vehicle, path, and target states
    */
      void setInternalTargetPDF(TargetPDF internal_target_pdf);
      void setVehicleID(int veh_id);
      void setTargetDepth(double depth);
      void setNeighborSet(Eigen::VectorXi neighbor_set);
      void localCorrection(double range, Eigen::Vector2d vehicle_position);
      void fuseWithNeighbor(Eigen::MatrixXd augmented_target_info_vector, Eigen::MatrixXd augmented_target_info_matrix);
      void localPrediction();
      void setMatricesWV(Eigen::MatrixXd Q, Eigen::MatrixXd R);

      void setETCParameters(double c0, double c1, double c2);
      void resetEstTargetPDF();
      void propagateEstTargetPDF();
      void setEstTargetPDF();



      // void updateVehiclePosition(Eigen::Vector2d vehicle_position);
      // void updateAugmentedTargetInfoVector(Eigen::MatrixXd target_info_vector);
      // void updateAugmentedTargetInfoMatrix(Eigen::MatrixXd target_info_matrix);
      // void updateRangeToTarget(double range);
      // void setDekfMode(bool dekf_mode);
      // bool getDekfMode();

    /**
     * @brief  Method to call main algorithm
     */
      void callCentralEKF(double range1, double range2, Eigen::Vector2d veh1_pos, Eigen::Vector2d veh2_pos);


    /**
     * @brief Methods to get estimated state and covariance
     */
      Eigen::Vector4d getTargetState(); 
      Eigen::Matrix4d getTargetCovariance();
      Eigen::Vector4d getTargetInfoVector(); 
      Eigen::Matrix4d getTargetInfoMatrix();
      EtcInfo checkBroadcastSignal(double time);

    /**
     * @brief  Method used in the first run to do initial setup
     */
    void start();

    /**
     * @brief  Method used to check whether we reached the end of the algorithm or not
     *
     * @return The success of the operation
     */
    bool stop();

    /**
     * @brief  Method used to reset the algorithm control parameters 
     * when running the algorithm more than once
     *
     * @return  Whether the reset was made successfully or not
     */
    bool reset();

  private:
    /**
     * @brief ETC parameters
     */ 

    double c0_, c1_, c2_;
	
    /**
     * @brief Auxilary variable and method
     */
    bool first_iteration{false};
protected:
    /**
     * @brief Variables to store the states of the vehicle, the path, and the target 
     */

    TargetPDF target_pdf_;
    // the following pdf is used for event-triggered communication
    TargetPDF est_target_pdf_;
    TargetPDF neighbor_est_target_pdf_;

    EtcInfo etc_info_;
    int Veh_ID;

    Eigen::MatrixXd augmented_target_info_vector_;
    Eigen::MatrixXd augmented_target_info_matrix_;
    Eigen::Vector2d vehicle_position_;
    double target_depth_;
    double range_;

    Eigen::VectorXi neighbor_set_;

    const double x_offset = 	4290797.0;
    const double y_offset =   491936.0;

    bool received_target_pdf_from_neighbors_{false};
    bool dekf_mode_{false};
    int counter_{0};

    // Tuning parameters
   Eigen::Matrix4d Q_;
   Eigen::Matrix4d W_;
   Eigen::MatrixXd R_;
   Eigen::MatrixXd V_;
   // target model
   Eigen::Matrix4d F_;

};

