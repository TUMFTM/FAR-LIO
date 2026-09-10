# Correspondence Threshold Library

TUM Autonomous Motorsport package for modular handling of correspondence threshold models

The correspondence threshold bounds the maximum distance at which a frame point may be matched to a
map point during [registration](registration_handler.md). It is queried once per registration and
returned as a single distance.

## Implemented Formats

1. **Fixed Threshold**
    - returns a constant threshold taken from the configuration (`threshold.initial_threshold`);
    predictable and free of any warm-up
2. **Adaptive Threshold**
    - adapts the threshold at runtime from the observed model deviation (the discrepancy between
    initial guess and registration result), so it tightens as the odometry stabilizes, from
    [KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Threshold.hpp)
