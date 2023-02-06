/**
 * @brief This algorithm is used for generating an spatial-temporal (S-T) curve for the vehicle to track
 *  https://www.dropbox.com/s/ojrdsdltdbi7usq/Mainv9.pdf?dl=0
 * 
 * The algorithm outputs
 *      - a desired Spatila-Temporal curve for vehicle to track
 * It requires inputs
 *      -  Trajectories  (e.g. targets's trajectories) 
 *      -  Paths         (e.g. a circumference)   
 * 
 * @author    DsorTeam
 * @author    Persons in charge: Hung, Quintas, Cruz
 * @version   1.0a
 * @date      2021
 * @copyright GPLv3
 */
#pragma once

/* Contains auxiliary functions for angle wrap-up and others */
#include <farol_gimmicks_library/FarolGimmicks.h>


class TrajectoryPlanningAlgorithm {

  public:

    /**
     * @brief  Constructor method for the class
     */
   TrajectoryPlanningAlgorithm();

    /**
     *  @brief Methods to update (target) trajectory and parameter of the path
    */
      void updateTargetTrajectory(Eigen::Vector3d target_pos, Eigen::Vector3d target_vel);  
      void updateGamma(double gamma);
      void updatePathParameter(double rx, double ry, double rz, double formation_offset);
      void updateRotateTargetFormationParameter(double desired_distance);
      void updateFixedTargetFormationParameter(double desired_distance, double desired_angular);

    /**
     * @brief  Method to call main algorithm that generate desired S-T curve
     */
      void callTrajectoryPlanningAlgorithm(double dt);

    /**
     * @brief Methods to get data of the S-T curve to be published 
     */
     Eigen::Vector3d getTrajPos();
     Eigen::Vector3d getTrajVel();
     Eigen::Vector3d getPathPos();
     Eigen::Vector3d getPathVelGamma();
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
     * @brief Auxilary variable
     */
    bool first_iteration_{true};

    void updatePath();
    Eigen::Vector3d target_pos_;
    Eigen::Vector3d target_vel_;
    Eigen::Vector3d traj_pos_ ;
    Eigen::Vector3d traj_vel_{0.0,0.0,0.0} ;
    Eigen::Vector3d path_pos_ ;
    Eigen::Vector3d path_vel_gamma_;
    double gamma_{0};
    
protected:
 
  double vd_bar_{0.0};            // desired norminal speed for gamma
  double rx_{0};
	double ry_{0};
  double rz_{0};
	double phi_{0};
  double alpha_{3*M_PI/4}; 


  std::string pursuit_mode_{"rotate"};	  

};

