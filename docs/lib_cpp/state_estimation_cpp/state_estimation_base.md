# State Estimation Base Library

This library defines the abstract base class `StateEstimationBase` for the
[tam_state_estimation](tam_state_estimation.md) library:

- pure-virtual interface (`step`, `set_initial_state`, `set_input_*`, output getters) that every
state estimation implementation has to provide
- shared types for the interface (`types.hpp`)
- allows the [tam_state_estimation_node](../../ros_packages/tam_state_estimation_node.md) to interact
with any filter implementation through a common, ROS-free interface
