# State Estimation Constants Library

This library defines the compile-time configurations for the
[tam_state_estimation](tam_state_estimation.md) library. Each configuration is a struct that
fixes the state, input and measurement layout of a filter variant at compile time and is passed
to the templated classes as a template parameter.

<!-- markdownlint-disable MD013 -->
| Config | Description |
| ------ | ----------- |
| `KF_BASE` | Shared base constants (number of sensors per modality, vector layouts) all filter configs build upon. |
| `EKF_2D` | 2D EKF: state vector $[X_m, Y_m, \psi, v_x, v_y]$, IMU input vector $[\omega_z, a_x, a_y]$. |
| `EKF_3D` | 3D EKF: state vector $[X_m, Y_m, Z_m, \phi, \theta, \psi, v_x, v_y, v_z]$, IMU input vector $[\omega_x, \omega_y, \omega_z, a_x, a_y, a_z]$. |
<!-- markdownlint-enable MD013 -->
