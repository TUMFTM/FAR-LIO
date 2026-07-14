# Distortion Handler Library

TUM Autonomous Motorsport package for point cloud undistortion

## Implemented Formats

1. POLYNOM/CUDA_POLYNOM
    - stores a pose history (usually set from the tf transform `odom_frame` -> `child_frame` in the node's cloud callback)
    - stores the most recent `0.2s` of pose history (`Sophus::SE3f`)
    - fits a polynom of configurable degree (compile-time parameter) to the **log-poses**
    `xi_i = log(T_ref^-1 * T_i)`, i.e. the se(3) twist coordinates of each history pose
    relative to a reference pose `T_ref`. `T_ref` is the SLERP-interpolated pose at
    the frame stamp and cancels out per point, so evaluating and exponentiating the
    polynom yields the deskewing correction directly
    - performs in-place undistortion by evaluating the polynom for each point timestamp
    and applying `point.pos = exp(poly(t)) * point.pos`
    - skips undistortion when fewer than two poses are available or the estimated velocity
    is below `vel_threshold`
