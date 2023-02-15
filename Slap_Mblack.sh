#!/bin/bash
echo Welcome to Medusa SLAP package !
echo 

read -p 'Enter SLAP command: ' mode

if [ $mode == set_target_pdf ]
then
echo 'Target state:'
 read -a state
echo 'Covariance (diagonal entries):'
 read -a cov

  if [ -z "$cov" ]
  then
    rosservice call /mblack0/slap/set_target_pdf "{pos_x: ${state[0]}, pos_y: ${state[1]}, vel_x: ${state[2]}, vel_y: ${state[3]}, var_pos_x: 10, var_pos_y: 10, var_vel_x: 0.01,
     var_vel_y: 0.01}"  
  else 
    rosservice call /mblack0/slap/set_target_pdf "{pos_x: ${state[0]}, pos_y: ${state[1]}, vel_x: ${state[2]}, vel_y: ${state[3]}, var_pos_x: ${cov[0]}, var_pos_y: ${cov[1]}, var_vel_x: ${cov[2]},
     var_vel_y: ${cov[3]}}"  
  fi 
fi


if [ $mode == launch_vehicle ]
then

 read -p 'Vehicle name: ' name
 read -p 'Mission: ' mission 
 
 roslaunch medusa_bringup medusa_bringup.launch name:=$name mission:=$mission
fi


if [ $mode == launch_slap ]
then

read -p 'Vehicle role: ' role

 if [ $role == tracker ]
 then
  roslaunch launch_slap launch_slap_tracker.launch name:=$name
 fi
 if [ $role == target ]
 then
  roslaunch launch_slap launch_slap_target.launch name:=$name
 fi
fi 

if [ $mode == set_dekf_gains ]
then
  # 
echo 'Q matrix (diagonal entries):'
 read -a Q
echo 'R matrix (diagonal entries):'
 read -a R
  
    rosservice call /mblack0/slap/set_matrices_QR "{Q11: ${Q[0]}, Q22: ${Q[1]}, Q33: ${Q[2]}, Q44: ${Q[3]}, R: ${R[0]}}"
fi

if [ $mode == set_tc_gains ]
then
  # 
    read -p 'delta: ' delta
    read -p 'kx: ' kx
    read -p 'ky: ' ky
    read -p 'kz: ' kz
  
    rosservice call /mblack0/slap/set_tc_gains "{delta: $delta, kx: $kx, ky: $ky, kz: $kz}"
fi


if [ $mode == set_etc_cpf ]
then
  # 
    read -p 'c0: ' c0
    read -p 'c1: ' c1
    read -p 'c2: ' c2
    rosservice call /mblack0/slap/cpf/set_ETC_parameter "{c0: $c0, c1: $c1, c2: $c2}" 
fi
if [ $mode == set_etc_dekf ]
then
  # 
    read -p 'c0: ' c0
    read -p 'c1: ' c1
    read -p 'c2: ' c2
    rosservice call /mblack0/slap/dekf/set_ETC_parameter "{c0: $c0, c1: $c1, c2: $c2}" 
fi

if [ $mode == start ]
then
  # rosservice call /slap/start_dekf
  echo  start ekf
  rosservice call /mblack0/slap/start_ekf "{}"

  echo start tracking
  rosservice call /mblack0/slap/start_tracking "{}"
fi
if [ $mode == start_dekf ]
then

  echo  start dekf
  rosservice call /mblack0/slap/start_dekf "{}"
fi  

if [ $mode == stop_dekf ]
then

  echo  stop dekf
  rosservice call /mblack0/slap/stop_dekf
fi  

if [ $mode == start_cpf ]
then

  echo  start cooperative control
  rosservice call /mblack0/slap/start_cooperative_control "{}"
fi  


if [ $mode == stop_cpf ]
then

  echo  stop cooperative control
  rosservice call /mblack0/slap/stop_cooperative_control
fi  

if [ $mode == stop ]
then 
  echo  stop ekf
  rosservice call /mblack0/slap/stop_ekf
  echo  stop tracking
  rosservice call /mblack0/slap/stop_tracking  
  echo  stop dekf
  rosservice call /mblack0/slap/stop_dekf  
  echo  stop cooperative_control
  rosservice call /mblack0/slap/stop_cooperative_control 
fi  

if [ $mode == reset ]
then
  rosservice call /mblack0/slap/nominal_desired_gamma_dot "data: 0.03" 
  echo "Reset desired speed profile for gamma to 0.03rad/s"
  rosservice call /mblack0/slap/set_gamma "data: 0.0" 
  echo "Reset gamma to 0"
  rosservice call /mblack0/slap/set_target_pdf "{pos_x: 10.0, pos_y: -5.0, vel_x: 0.0, vel_y: 0.0, var_pos_x: 10.0, var_pos_y: 10.0, var_vel_x: 0.01,
    var_vel_y: 0.01}"
  echo "Reset target pdf to default orignal value"    
fi

if [ $mode == set_formation ]
then
  read -p 'Please enter desired formation: ' desired_formation

  if [ $desired_formation == encircle_target ]
  then
    read -p 'Set desired distance to target(m): ' desired_distance
    read -p 'Set desired angular rate(rad/s): ' desired_angular_rate
    echo  switch to encircle target mode:
    rosservice call /mblack0/slap/rotate_target_mode "{desired_distance: $desired_distance,  desired_angular_rate: $desired_angular_rate}" 
  fi

  if [ $desired_formation == behind_target ]
  then
    read -p 'Set desired distance to target(m): ' desired_distance
    read -p 'Set desired angle(degree): ' desired_angle
    echo  switch to behind target mode:
    rosservice call /mblack0/slap/fixed_target_mode "{desired_distance: $desired_distance,  desired_angle: $desired_angle}" 
  fi

  if [ $desired_formation == front_target ]
  then
    read -p 'Set desired distance to target(m): ' desired_distance
    read -p 'Set desired angle(degree): ' desired_angle
    a=`expr $desired_angle - 180`
    echo  switch to behind target mode:
    rosservice call /mblack0/slap/fixed_target_mode "{desired_distance: $desired_distance,  desired_angle: $desired_angle}" 
  fi
fi

if [ $mode == set_target_depth ]
then
  # 
    read -p 'target_depth: ' target_depth
  
    rosservice call /mblack0/slap/dekf/set_target_depth "{data: $target_depth}"
fi
