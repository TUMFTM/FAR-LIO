# Preprocessing Library

TUM Autonomous Motorsport package for preprocessing

## Implemented Preprocessing Formats

1. General Preprocessing Functionalities:

    - **Crop Filter**: Apply spatial cropping based on the distance of a point from the origin
    given a minimum and maximum range

    - **Crop Footprint**: Apply spatial cropping based on the x- and y-coordinate of a point
    given the longitudinal and lateral size of the vehicle footprint

2. LiDAR Preprocessing:

    - the default preprocessing for dense LiDAR scans
    - applies only the general functionalities above (range crop, then vehicle-footprint crop)
    and requires no sensor-specific point attributes
    - available as a CPU implementation and a CUDA variant (`CUDA_LIDAR`) that runs the same crops
    on the GPU

