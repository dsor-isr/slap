/**
 * @brief This controller is used for tracking spatial-temporal (S-T) curves in the context of target localization
 * and pursuit, given in the paper at: https://www.dropbox.com/s/ojrdsdltdbi7usq/Mainv9.pdf?dl=0
 * 
 * The controller outputs
 *      - yaw-rate
 *      - surge
        - gamma_dot or gamma_ddot: to control evolution of the "reference point on the path"
 * It requires inputs:
 *      - vehicle state
 *      - path state
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
/* Auxiliary definitions for the path state and vehicle state */
#include "StateStructs.h"

/* ROS includes for publishing the values and setting the mode of operation */
#include "ros/ros.h"
#include <std_msgs/Float64.h>

/* Contains auxiliary functions for angle wrap-up and others */
#include <farol_gimmicks_library/FarolGimmicks.h>

class TrackingControlAlgorithm {

  public:

    /**
     * @brief  Constructor method for the class
     */
   TrackingControlAlgorithm();

    /**
     *  @brief Methods to update vehicle, path, and target states
    */
      void updateVehicleState(VehicleState vehicle_state);
      void updateSTCurveState(STCurveState st_curve_state);
      bool reset();

    /**
     * @brief  Method to call main algorithm
     */
      void callTrackingControl(double dt);

    /**
     * @brief Methods to get desired surge, yaw_rate, gamma
     */
      double getDesiredSurge(); 
      double getDesiredYawRate();
      double getGammaDot();

     
    /**
     * @param set gains for the tracking controller
     */
      void setTCGains(double delta,double kx, double ky, double kz); 

  private:

    /**
     * @brief Controller parameters for the tracking algorithm
     */
    Eigen::Vector2d epsilon_;
    Eigen::Matrix2d Delta_;
    Eigen::Matrix2d Delta_inv_;
    Eigen::Matrix2d Kp_;
    double kz_;

    /** 
     * @brief The desired references outputted from the controller
     */
    double desired_surge_{0.0};
    double desired_yaw_rate_{0.0};    
    /** 
     * @brief The values for the dynamics of the gamma (virtual reference point) 
     */
    double gamma_ddot_{0.0};
    double gamma_dot_{0.0};
    double gamma_{0.0};

    /**
     * @brief Auxilary variable
     */
    bool first_iteration_{true};

protected:
    /**
     * @brief Variables to store the states of the vehicle, the path, and the target 
     */
    VehicleState vehicle_state_;
    STCurveState st_curve_state_;
    double gain_scale_{0.0};

};

