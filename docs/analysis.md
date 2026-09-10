# Analysis

FAR-LIO integrates with the standard ROS 2 tooling for inspecting, plotting and debugging its
output.

## Plotting with PlotJuggler

[PlotJuggler](https://github.com/facontidavide/PlotJuggler) is the recommended tool for inspecting
FAR-LIO's numeric output. It natively decodes the [tsl](https://github.com/TUMFTM/tsl) debug
messages that FAR-LIO uses extensively, and it also provides a convenient interface for plotting
and illustrating regular ROS 2 messages.

The debug messages expose all details about the running system, including:

- the inputs actually used (LiDAR, IMU, …),
- measurement and processing delays,
- the computation time of each individual module,
- and much more.

This makes it straightforward to profile the pipeline, spot bottlenecks, and verify that inputs
arrive as expected.

## Visualizing odometry and the local submap

To inspect the odometry and mapping geometrically, enable **`debug_mode`** in the configuration
(`pipeline.debug_mode: true` in `config/far-lio.yml` — see [Configuration](configuration.md)).
With it enabled, `LidarOdometry` publishes:

- the point cloud used for registration, and
- the current local submap.

Both can be displayed in **RViz2**, for example to check alignment quality and map consistency.

!!! warning
    Publishing the whole local submap is expensive. Enabling `debug_mode` can noticeably increase
    computation times, so use it for analysis rather than during performance-critical runs.
