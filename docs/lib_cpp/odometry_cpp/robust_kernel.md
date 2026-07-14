# Robust Kernels Library

TUM Autonomous Motorsport package for robust kernels for scan registration

Robust kernels down-weight correspondences with large residuals so that outliers (dynamic objects,
wrong matches) do not dominate the registration. The kernel is a compile-time property of the config
(`TConfig::KERNEL`) and returns a scalar weight `w ∈ [0, 1]` for a residual, given a
`kernel_scale`. Implementations are based on
[small_gicp](https://github.com/koide3/small_gicp/blob/master/include/small_gicp/factors/robust_kernel.hpp)
and [KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Registration.cpp).

## Available Kernels

| Kernel | Weight `w(r)` | Behavior |
| ------ | ------------- | -------- |
| `NONE` | `1` | No down-weighting; plain least squares. |
| `HUBER` | `1` if `‖r‖² < s`, else `s / ‖r‖²` | Quadratic for inliers, linear for outliers; mild down-weighting. |
| `CAUCHY` | `1 / (1 + ‖r/s‖²)` | Smooth, heavier down-weighting than Huber; never reaches zero. |
| `GEMANMCCLURE` | `s² / (s + ‖r‖²)²` | Strong outlier rejection, weight → 0 for large residuals; KISS-ICP default. |
| `WELSCH` | `exp(-‖r/s‖²)` | Exponential cut-off; the most aggressive, effectively hard-rejects far residuals. |

Here `r` is the correspondence residual and `s` the `kernel_scale`.
