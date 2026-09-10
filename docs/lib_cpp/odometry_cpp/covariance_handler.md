# Covariance Computation Library

TUM Autonomous Motorsport package for modular computation of scan registration covariance

## Implemented Formats

Each format returns the 6x6 pose covariance of the registration result (row-major 36-element
array). All formats are bound by minimum covariances as part of the configuration. This ensures
that the published covariance may not be unrealistically low.

1. **Constant Covariance**
    - returns fixed covariance values taken directly from the configuration, independent of the
    registration; the cheapest option and a safe default when a calibrated model is unavailable