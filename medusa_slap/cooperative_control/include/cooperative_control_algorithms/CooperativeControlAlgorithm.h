/**
 * @brief This cooperative control algorithm for cooperative tracking task
 *        given in the paper at: https://www.dropbox.com/s/ojrdsdltdbi7usq/Mainv9.pdf?dl=0
 *        It borrows the concept of cooperative path following in many previous work, see the reference therein the paper
 * The controller outputs
 *      - correction speed
 * It requires inputs:
 *      - coordination state from vehicle neighbor - in this context is the path parameter gamma
 *      - coordination state of the vehicle its self - in this context is the path parameter gamma of the vehicle
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

struct EtcInfo {
  double est_error;               // estimation error of gamma    
	double threshold;
  bool broadcast_signal;
};

class CooperativeControlAlgorithm {

  public:

    /**
     * @brief  Constructor method for the class
     */
   CooperativeControlAlgorithm();

    /**
     *  @brief Methods to update gamma of the vehicle its self and gamma from the neighbor
    */
      void updateInternalGamma(double internal_gamma);
      void updateNeighborGamma(double neighbor_gamma);
      void updateNeighborSet(Eigen::VectorXi neighbor_set);
      void updateGammaVector(Eigen::VectorXd gamma_vector);
      void updateVehicleID(int veh_id);
      void updateGains(double kc);

      void setETCParameters(double c0, double c1, double c2);
      void resetInternalEstGamma();
      void resetNeighborEstGamma(double neighbor_gamma, int neighbor_id);
      void predictEstGamma(double dt);
		  void updateGammaDesiredSpeedVector(Eigen::VectorXd gamma_desired_speed_vector);

      void setEstGamma(Eigen::VectorXd gamma_vector);



    /**
     * @brief  Method to call main algorithm
     */
      void callCooperativeController(double dt);

    /**
     * @brief Methods to get desired surge, yaw_rate, gamma
     */
      double getCorrectionSpeed(); 
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

    /**
     * @param gains A vector of gains for the controller
     */
    bool setPFGains(std::vector<double> gains);

  private:

    /**
     * @brief Controller parameters for the cooperative algorithm
     */

    double kc_{0.0};

    /** 
     * @brief The desired correction speed outputted from the controller
     */
    double vc_{0.0};

    /**
     * @brief Auxilary variable
     */
    bool first_iteration_{true};
    Eigen::VectorXi neighbor_set_;
    Eigen::VectorXd gamma_vector_;
    Eigen::VectorXd gamma_vector_est_;
    Eigen::VectorXd gamma_desired_speed_vector_;


    double c0_, c1_, c2_;

protected:
    /**
     * @brief Variables to store the states of the vehicle, the path, and the target 
     */
    double internal_gamma_;
    double neighbor_gamma_; 
    EtcInfo etc_info_;
    int Veh_ID_;
    double gain_scale_{0};

};

