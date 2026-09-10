# Model Handler Library

TUM Autonomous Motorsport package for modular handling of prediction models for lidar odometry algorithm

## Implemented Formats

1. ExternalGuess
    - to be used in combination with an external filter (e.g. EKF)
    - initial guess needs to be set to the pipeline before the registration via the
    pipeline API

2. Constant Velocity
    - generates initial guess based on constant-velocity assumption
    (actually constant distance assumption, as only leverage previous
    poses)

    $$
    \vec{T}_t = \vec{T}_{t-1}\,\vec{T}_{t-2}^{-1}\,\vec{T}_{t-1}
    $$
