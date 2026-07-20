<p align="center">
  <img src="assets/farlio-banner.png" alt="FAR-LIO" width="660">
</p>

# TAM State Estimation

Welcome to the TAM State Estimation documentation.  
This repository is responsible for estimating the vehicle position and orientation with respect to
the local cartesian frame (ENU), as well as the vehicle state (linear/angular velocity and linear
acceleration) in the vehicle coordinate frame.
![High level state estimation concept](images/repository_structure.png)

- the state estimation provides a versatile and modular interface for N localization inputs
  (GNSS or SLAM), N velocity inputs (wheelspeed sensors, slip angle sensor) and N IMUs
- all classes are templated with configurations using structs
  ([state_estimation_constants](lib_cpp/state_estimation_constants.md)), which enables the
  utilization of various Kalman filters (such as 2D EKF, 3D EKF or UKF) within the same
  [node](ros_packages/tam_state_estimation_node.md)
- in addition to the state estimation, this repository contains a dedicated side slip angle
  estimation ([ssa_estimation_cpp](lib_cpp/ssa_estimation_cpp.md)) and tf2 state publishers
  providing static transformations within the software stack

## Acknowledgements

The implementations are mainly based on the following paper:

- [Vehicle Dynamics State Estimation and Localization for High Performance Race Cars](https://www.sciencedirect.com/science/article/pii/S2405896319303957)
