# SLAP (Simultaneous Localization And Pursuit)
This repository contains the ROS code USED to generate the results in the paper entitled **Target localization and pursuit with networked robotic vehicles: theory and experiments**.

Cooperative simultaneous target localization and pursuit (cooperative SLAP) refers to the problem of using one or more autonomous robots/vehicles, acting as mobile sensors, to estimate the state of single or multiple targets with unknown trajectories, while staying in a desired vicinity of the targets. The state of each target typically includes the target´s position, velocity, and even possibly acceleration, depending on the model adopted for the target. The information about each target is given in terms of ranges, bearing, or both, depending on the sensing capability of the trackers.

In order to test the SLAP algorithm in a simulation environment with autonomous underwater vehicles (AUVs), please refer to the [SLAP Simulation](https://github.com/dsor-isr/slap_simulation) repository.

### Contributions
We propose TWO approaches to SOLVE range-based SLAP problems:
- A centralized approach using MPC and the BAYESIAN Cramer Rao Lower Bound.
- A distributed approach using tools from nonlinear control, distributed control, and distributed estimation.

### Reference
If you find the code useful and would like to cite it, please REFERENCE the following paper:

**Target localization and pursuit with networked robotic vehicles: theory and experiments**, 

Authors: Nguyen Hung, Eduardo Cunha, Francisco Branco, Antonio Pascoal, Institute for Systems and Robotics, IST, Lisbon

The paper was accepted for publication at the Journal of Field Robotics

More information:
https://nt-hung.github.io/research/Range-based-target-localization/