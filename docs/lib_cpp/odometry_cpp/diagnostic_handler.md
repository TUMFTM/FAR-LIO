# Diagnostic Handler Library

TUM Autonomous Motorsport package for modular handling of health checks for odometry algorithm

## Diagnostic Checks

The following table lists the available diagnostic check that may be conducted in the listed order.
It consists of three stages (`Registration Status`, `Input Status`, and `Pose Validity`).
Apart from the `Registration Status` checks, each single test may be deactivated in the configuration.

<!-- markdownlint-disable MD013 -->
| Registration Status Check | Description |
| ---------------- | ----------- |
| Convergence | Returns `error`, if registration did not converge, `warn` if the registration time exceeded the limit, and `ok` otherwise |
<!-- markdownlint-enable MD013 -->

<!-- markdownlint-disable MD013 -->
| Input Status Check | Description |
| ---------------- | ----------- |
| Input Status | Returns `error`, if the input status is `error` or no input status was received, and `ok` otherwise. (Ignoring `warn`) |
<!-- markdownlint-enable MD013 -->

<!-- markdownlint-disable MD013 -->
| Pose Validity Check | Description |
| ---------------- | ----------- |
| Forward | Returns `error`, if the registered pose is not in front of the previous pose (if above a `min_motion_threshold`), and `ok` otherwise |
| Ellipsis | Returns `error`, if the registered pose is outside an ellipsis (size configurable) around the initial guess (i.e. unrealistically far apart), and `ok` otherwise |
| Time Diff | Returns `error`, if the difference between the time stamps of the initial guess and the input pointcloud is too high, and `ok` otherwise |
| Velocity | **Note:** This check only evaluates the computed velocity in the velocity handler, but does not consider the registered pose. As the EKF in the TAM software stack does neglect the velocity of an odometry-msg if its diagnostic status is set to `warn`, this check only sets the status to warn, in case any check is not passed. Returns `warn`, if the side slip angle estimation (ssa) is disabled. Returns `warn` if the vehicle is below the configured `vel_motion_threshold` (vel numerically unstable). Returns `warn`, if the computed velocity is invalid (e.g. zero, although the vehicle is above the motion threshold). Returns `ok` otherwise |
<!-- markdownlint-enable MD013 -->
