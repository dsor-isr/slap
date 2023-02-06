#pragma once

#include <Eigen/Core>
#include <limits>
/**
 * @brief This defines structures to store state of the vehicle, the path, and the target
 * @author  DSOR team
 * @author  Person in charges: Hung, Quintas, Cruz
 * @date    2021
 */
typedef struct VehicleState {
  /**
   * @brief Vehicle position and orientation in the inertial 
   */
  Eigen::Vector3d pos{0.0, 0.0, 0.0}; 
  Eigen::Vector3d orientation{0.0, 0.0, 0.0} ; 

  /**
   * @brief Vehicle linear and angular velocity vector in Body 
   */
  Eigen::Vector3d linear_vel{0.0, 0.0, 0.0};     
  Eigen::Vector3d angular_vel{0.0, 0.0, 0.0};     

 }VehicleState ;

typedef struct PathState {
  
  /**
   * @brief The path parameter gamma
   */
  double gamma{0.0};
  
  /**
   * @brief Position of the reference point on the path and its partial derivative respect to gamma
   */
  Eigen::Vector3d pos{0.0, 0.0, 0.0};
  Eigen::Vector3d vel_gamma{0.0, 0.0, 0.0};

  /**
   * brief desired nominal and correction speed for the derivative of gamma
   */
  double vd_bar{0.0};
  double vc{0.0};
   
  /**
   * @brief Other properties of the path such as the psi, curvature and tangent_norm 
   */
}PathState;


typedef struct PathPara {
   /**
    * @brief struct to store parameters of a path - in this case is the parameter of circles  
    */ 
  double phi;         // offset of the path parameter, use to configure desired formation
  double rx;          // radius in x
  double ry;
  double rz; 
} PathPara; 


/**
 * @brief  A structure to hold the data of the target state
 */
typedef struct TargetState {
 
  /**
  * @brief target position and velocity in the inertial
  */

  Eigen::Vector3d pos{0.0, 0.0, 0.0};
  Eigen::Vector3d vel{0.0, 0.0, 0.0};

} TargetState; 

/**
 * @brief  A structure to hold the data of a spatial temporal curve that combines a 
 *         trajectory (e.g. target) and a spatial path (e.g. a circumference about the target)
 */

typedef struct STCurveState {

  /**
   * @brief Spatial path component
   */

  double gamma{0.0};                          // the path parameter
  Eigen::Vector3d path_pos{0.0, 0.0, 0.0};    // position of the path given each gamma
  Eigen::Vector3d path_vel_gamma{0.0, 0.0, 0.0};   // velocity of the path respect to gamma
  double vd_bar{0.0};                         // norminal desired speed for gamma_dot
  double vc{0.0};                             // correction desired speed for gamma_dot
  /**
   * @brief Other properties of the path component such as tangent, curvature and tangent_norm 
   */


  /**
   * @brief Temporal (trajectory) component
   */
  Eigen::Vector3d traj_pos{0.0, 0.0, 0.0};
  Eigen::Vector3d traj_vel{0.0, 0.0, 0.0};
} SpatialTemporalCurve ;