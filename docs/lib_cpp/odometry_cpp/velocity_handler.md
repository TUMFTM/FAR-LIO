# Velocity Handler Library

TUM Autonomous Motorsport package for modular estimation of the body-frame twist
(linear/angular velocity) of the vehicle.

The module estimates the velocity for the current frame and registered pose and returns it
as a stamped tangent vector $[v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]$ (SE(3) twist
convention). Each implementation inherits from the base handler `VelocityHandler` and overrides
the virtual interface:

```cpp
virtual types::TangentStamped get_tangent(
  const std::vector<types::Point<TConfig>> & frame,
  const types::PoseStamped & pose_registered) = 0;
```

Configuration and debug signals are defined in
[odometry_types](odometry_types.md) (`VelocityConfig`, `VelocityDebug`). The base handler logs
`velocity/velocity_time` (compute time in ms) and `velocity/conditional` (a map of
implementation-specific conditional debug signals).

## Implemented Formats

1. **Derivative**
    - estimates the full 6-DoF twist purely from the registered pose history, independent of the
    sensor frame (the `frame` argument is unused)
    - velocity is the logarithmic map of the relative SE(3) transform between the previous and the
    current pose, divided by the time delta:

    $$
    \vec{\xi}_t = \frac{\log\!\left(\vec{T}_{t-1}^{-1}\,\vec{T}_t\right)}{\Delta t}
    $$

    - the result is sanitized: it returns zero on the first call (uninitialized), on an invalid
    time delta ($\Delta t$ non-finite, $\approx 0$, or $> 1\,\text{s}$), and clamps unreasonable
    linear ($> 100\,\text{m/s}$) or angular ($> 10\,\text{rad/s}$) velocities to zero
    - logs the conditional signal `vel_dt`
    - works with any `TConfig`; the default choice when no Doppler sensor is available
