# Odometry Library

A templated **C++-only** library without any ROS2 dependencies.
It contains all core functionalities of the localization and may be used for low-level developments. It also defines the
built executables that may later be used in the ROS wrappers. Read the documentation to understand the software
architecture.

## Design

- the folder base defines a base [class](odometry_cpp/odometry_base.md) and base [types](odometry_cpp/odometry_types.md)
which all other modules are built on:
  - the base class holds a **config** and **debug** object (custom structs) which specify the values necessary
  for configuration of the module and the values that are to be logged for debugging
  - the base types holds the definition of the structs for each class (config and debug) and further specify
  custom, templated variables for interfaces between the modules (diagnostic status, point type)
  - the base types also hold the [config](https://github.com/TUMFTM/FAR-LIO/blob/main/lib_cpp/odometry_cpp/base/odometry_types/include/odometry_types/odometry_config.hpp)
  with the template arguments for the construction of the single modules/pipelines/nodes
- the folder modules implements the singles modules necessary for the whole odometry node
  - each module consists of a base module that specifies the config and debug variables of it and the interfaces of
  the modules
  with other modules (inherited of [odometry_base](https://github.com/TUMFTM/FAR-LIO/tree/main/lib_cpp/odometry_cpp/base/odometry_base))
  - implementations of specific modules (e.g. ICP) inherit from its base class (e.g. RegistationHandler) and
  implement overrides for the virtual interface functions of the module base class
  - each module is a header-only library
  - for each specific implementation, there is an exemplary executable in the folder `example/module.cpp` that can
  be used to see how to use it
  - for each specific implementation, there are unit tests to test correct functionality, leveraging
  [GoogleTest](http://google.github.io/googletest/quickstart-cmake.html)
  - if you add a new implementation for a module, make sure to also add unit tests and and example
- the folder [pipeline](odometry_cpp/odometry_pipeline.md) combines all modules to a full odometry pipeline as a
shared library and holds the single modules
as member variables. It also holds an example and several unit tests

## Develop

- the library is a native C++ library without any ROS2 deps.
- currently, it depends on the following libraries:
  1. Eigen3 (`sudo apt install libeigen3-dev`)
  2. [Sophus](https://github.com/strasdat/Sophus)
  3. [robinmap](https://github.com/Tessil/robin-map) (MapHandler)
  4. [tbb](https://github.com/oneapi-src/oneTBB?tab=readme-ov-file) (CPU Multithreading)
  - robinmap, tbb and Sophus are automatically fetched via CMake with a specific tag
- generate the build files using the following commands:

```bash
cmake -S path/to/odometry_cpp -B path/to/build_folder -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

- then build the library:

```bash
cmake --build path/to/build_folder
```

- execute the example executables to debug (exemplary):

```bash
path/to/build_folder/modules/registration_handler/icp_example
```

- run the unit tests before creating merge requests:

```bash
ctest --test-dir path/to/build_folder
```

- add the flag `--verbose` to see additional information if a test fails

## CUDA

- building the library with CUDA support involves the following additional dependencies:
  - [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)
  (installation of nvcc compiler and cuda toolkit)
  - [cuCollections](https://github.com/NVIDIA/cuCollections) (automatically fetched
  via Cmake with a specific tag)
- the library support CUDA implementations for the following modules right now:
  - `PolynomUndistortion` ([DistortionHandler](odometry_cpp/distortion_handler.md))
  - `VoxelHashMap` ([MapHandler](odometry_cpp/map_handler.md))
  - `ICP`, `GICP` ([RegistrationHandler](odometry_cpp/registration_handler.md))
  - `voxel_downsampling` ([VoxelTools](odometry_cpp/voxel_tools.md))
- to build the library with CUDA-capability, check the architecture of your target
device in the NVCC [GPU Feature List](https://docs.nvidia.com/cuda/cuda-compiler-driver-nvcc/).
- set the architecture of your GPU as a build-argument:

```bash
cmake -S path/to/odometry_cpp -B path/to/build_folder -DBUILD_TESTING=ON -DBUILD_CUDA=ON -DCUDA_ARCHITECTURES=<num>
```

## Testing with Example Data

### Example

See this [example](https://github.com/TUMFTM/FAR-LIO/blob/main/lib_cpp/odometry_cpp/modules/registration_handler/example/icp.cpp)
on how to use the test data for registration testing.
To register a single point cloud to the map, call:

```sh
./icp_example <path/to/map.bin> <path/to/frame.bin> <path/to/init_guess.txt>
```

If using the test data, select one frame and the odometry file with the closest timestamp
(from the timestamp).
This will give you output on the iterations and the registration time. However, feel free to
customize it.

## Extensibility

### Adding an Executable

To add an executable to the library, that can be used in the [Examples](#example) or within the ROS2 node,
follow these steps:

- add a new config struct to the [odometry_config.hpp](https://github.com/TUMFTM/FAR-LIO/blob/main/lib_cpp/odometry_cpp/base/odometry_types/include/odometry_types/odometry_config.hpp)
- add your config-struct to the list of [template-instantiations](https://github.com/TUMFTM/FAR-LIO/blob/main/lib_cpp/odometry_cpp/odometry_pipeline/include/odometry_pipeline/odometry_pipeline.hpp)
  (bottom of the file, also adjust the respective `.cpp`-file)
- for usage in the ROS2 node, also adjust the `.cpp`-file and `CMakeLists.txt` of
  [tam_odometry](../ros_packages/tam_odometry.md)

### Adding a Module

To add a new implementation for one of the modules, follow these steps:

- in the respective module, open its corresponding `<module>_handler_base.hpp`-file
- identify the module's `config`- and `debug`-signals
- following existing implementations, create a new `.hpp`-file with your implementation that
  inherits from the base-handler and overrides its virtual interface functions
- add a unit test under `test/` covering at least construction from both constructors
  (param manager & logger, and config & debug) and register it in the module's `CMakeLists.txt`
  by adding its name to the `TESTS` list
- add an example under `example/` and register it in the `EXAMPLES` list of the same `CMakeLists.txt`
  (both lists are iterated over, so no further CMake changes are required)

## Visualization

For visualization, the cmake-argument `VISUALIZATION` is provided that installs
[rerun](https://rerun.io/). It is fetched from github via cmake, but will take some time
to build so it is highly recommended to use some additional cores when building the library with
visualization:

```bash
cmake -S path/to/odometry_cpp -B path/to/build_folder -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DVISUALIZATION=ON
cmake --build path/to/build_folder -j <num_threads>
```

Rerun is based on a streamer-viewer concept: In your cpp-code, you can then stream data via TCP,
when using Rerun. The streamed data can then be visualized in the viewer, which needs to installed via

```bash
pip3 install rerun-sdk
```

If you prefer a different installation method, refer to the
[documentation](https://rerun.io/docs/getting-started/installing-viewer#installing-the-viewer).

To use rerun within your code, the compiler definition `USE_VISUALIZATION` is provided (if `DVISUALIZATION=ON`).
It provides the option to compile code in your example file, depending on the `VISUALIZATION` and may be used
with the following makro:

```C++

#ifdef USE_VISUALIZATION
#include <rerun.hpp>
<your code>
#endif
```

For an example on the basic visualization of a point cloud, refer to the
[voxel_hash_map_example](https://github.com/TUMFTM/FAR-LIO/blob/main/lib_cpp/odometry_cpp/modules/map_handler/example/voxel_hash_map.cpp)
