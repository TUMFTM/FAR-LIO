/*
 * Copyright 2026 Maximilian Leitenstern, Marcel Weinmann
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include "covariance.hpp"
#include "diagnostic.hpp"
#include "distortion.hpp"
#include "kernel.hpp"
#include "map.hpp"
#include "model.hpp"
#include "preprocessing.hpp"
#include "registration.hpp"
#include "threshold.hpp"
#include "velocity.hpp"

namespace tam::core::state::types {
/**
 * @brief Vehicle speed constraints
 * @note Currently used within DCR to prune infeasible aliasing hypotheses based on the physical
 * vehicle envelope.
 */
static constexpr float VXMIN = 0.0f;
static constexpr float VXMAX = 90.0f;
static constexpr float VYMAX = 10.0f;
/**
 * @brief Maximum size of map (allocated during instantiation)
 * @note Should be power of 2 for optimal performance
 */
static constexpr std::size_t MAX_MAP_SIZE = 1 << 23;  // 2^23 = 8388608
static constexpr double MAX_LOAD_FACTOR = 0.3;        // Max load factor for hashmap
/**
 * @brief Degree for polynomial fitting of motion distortion
 */
static constexpr uint8_t POLYNOM_DEGREE = 1;

/**
 * @brief Configuration for the TAM-ICP algorithm with external initial guess
 */
struct ICP_EXT {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = false;
  static constexpr bool COV = false;
  static constexpr bool SEG = false;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 20;
  static constexpr int16_t NUM_NEIGHBORS = 5;
  static constexpr CovarianceType COVARIANCE = CovarianceType::CONSTANT;
  static constexpr DiagnosticType DIAGNOSTIC = DiagnosticType::BASE;
  static constexpr DistortionType DISTORTION = DistortionType::POLYNOM;
  static constexpr RobustKernelType KERNEL = RobustKernelType::GEMANMCCLURE;
  static constexpr MapType MAP = MapType::VOXELHASHMAP;
  static constexpr ModelType MODEL = ModelType::EXTERNALGUESS;
  static constexpr PreprocessingType PREPROCESSING = PreprocessingType::LIDAR;
  static constexpr RegistrationType REGISTRATION = RegistrationType::ICP;
  static constexpr ThresholdType THRESHOLD = ThresholdType::ADAPTIVETHRESHOLD;
  static constexpr VelocityType VELOCITY = VelocityType::DERIVATIVE;
};

/**
 * @brief Configuration for the TAM-ICP algorithm with constant velocity model
 */
struct ICP_CV {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = false;
  static constexpr bool COV = false;
  static constexpr bool SEG = false;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 20;
  static constexpr int16_t NUM_NEIGHBORS = 5;
  static constexpr CovarianceType COVARIANCE = CovarianceType::CONSTANT;
  static constexpr DiagnosticType DIAGNOSTIC = DiagnosticType::BASE;
  static constexpr DistortionType DISTORTION = DistortionType::POLYNOM;
  static constexpr RobustKernelType KERNEL = RobustKernelType::GEMANMCCLURE;
  static constexpr MapType MAP = MapType::VOXELHASHMAP;
  static constexpr ModelType MODEL = ModelType::CONSTANTVELOCITY;
  static constexpr PreprocessingType PREPROCESSING = PreprocessingType::LIDAR;
  static constexpr RegistrationType REGISTRATION = RegistrationType::ICP;
  static constexpr ThresholdType THRESHOLD = ThresholdType::ADAPTIVETHRESHOLD;
  static constexpr VelocityType VELOCITY = VelocityType::DERIVATIVE;
};

/**
 * @brief Configuration for radar-based KISS-ICP with external initial guess
 * @note Includes radar-specific attributes: RCS, SNR, Confidence, and velocity interval
 */
struct RADAR_ICP_EXT {
  static constexpr bool SPHERICAL = true;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = false;
  static constexpr bool COV = false;
  static constexpr bool SEG = false;
  static constexpr bool VEL = true;
  static constexpr bool RCS = true;
  static constexpr bool SNR = true;
  static constexpr bool CONFIDENCE = true;
  static constexpr bool VEL_INTERVAL = true;
  static constexpr bool SENSOR_ID = true;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 20;
  static constexpr int16_t NUM_NEIGHBORS = 5;
  static constexpr CovarianceType COVARIANCE = CovarianceType::CONSTANT;
  static constexpr DiagnosticType DIAGNOSTIC = DiagnosticType::BASE;
  static constexpr DistortionType DISTORTION = DistortionType::POLYNOM;
  static constexpr RobustKernelType KERNEL = RobustKernelType::GEMANMCCLURE;
  static constexpr MapType MAP = MapType::VOXELHASHMAP;
  static constexpr ModelType MODEL = ModelType::EXTERNALGUESS;
  static constexpr PreprocessingType PREPROCESSING = PreprocessingType::RADAR;
  static constexpr RegistrationType REGISTRATION = RegistrationType::ICP;
  static constexpr ThresholdType THRESHOLD = ThresholdType::ADAPTIVETHRESHOLD;
  static constexpr VelocityType VELOCITY = VelocityType::DCR;
};

/**
 * @brief Configuration for the CUDA KISS-ICP algorithm
 */
struct CUDA_ICP_EXT {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = false;
  static constexpr bool COV = false;
  static constexpr bool SEG = false;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 20;
  static constexpr int16_t NUM_NEIGHBORS = 5;
  static constexpr CovarianceType COVARIANCE = CovarianceType::CONSTANT;
  static constexpr DiagnosticType DIAGNOSTIC = DiagnosticType::BASE;
  static constexpr DistortionType DISTORTION = DistortionType::CUDA_POLYNOM;
  static constexpr RobustKernelType KERNEL = RobustKernelType::GEMANMCCLURE;
  static constexpr MapType MAP = MapType::CUDA_VOXELHASHMAP;
  static constexpr ModelType MODEL = ModelType::EXTERNALGUESS;
  static constexpr PreprocessingType PREPROCESSING = PreprocessingType::CUDA_LIDAR;
  static constexpr RegistrationType REGISTRATION = RegistrationType::CUDA_ICP;
  static constexpr ThresholdType THRESHOLD = ThresholdType::ADAPTIVETHRESHOLD;
  static constexpr VelocityType VELOCITY = VelocityType::DERIVATIVE;
};

/**
 * @brief Configuration for the TAM-GICP algorithm with external initial guess
 */
struct GICP_EXT {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = true;
  static constexpr bool COV = true;
  static constexpr bool SEG = false;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 40;
  static constexpr int16_t NUM_NEIGHBORS = 10;
  static constexpr CovarianceType COVARIANCE = CovarianceType::CONSTANT;
  static constexpr DiagnosticType DIAGNOSTIC = DiagnosticType::BASE;
  static constexpr DistortionType DISTORTION = DistortionType::POLYNOM;
  static constexpr RobustKernelType KERNEL = RobustKernelType::CAUCHY;
  static constexpr MapType MAP = MapType::VOXELHASHMAP;
  static constexpr ModelType MODEL = ModelType::EXTERNALGUESS;
  static constexpr PreprocessingType PREPROCESSING = PreprocessingType::LIDAR;
  static constexpr RegistrationType REGISTRATION = RegistrationType::GICP;
  static constexpr ThresholdType THRESHOLD = ThresholdType::ADAPTIVETHRESHOLD;
  static constexpr VelocityType VELOCITY = VelocityType::DERIVATIVE;
};

/**
 * @brief Configuration for the TAM-CUDA-GICP algorithm with external initial guess
 */
struct CUDA_GICP_EXT {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = true;
  static constexpr bool COV = true;
  static constexpr bool SEG = false;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 40;
  static constexpr int16_t NUM_NEIGHBORS = 10;
  static constexpr CovarianceType COVARIANCE = CovarianceType::CONSTANT;
  static constexpr DiagnosticType DIAGNOSTIC = DiagnosticType::BASE;
  static constexpr DistortionType DISTORTION = DistortionType::CUDA_POLYNOM;
  static constexpr RobustKernelType KERNEL = RobustKernelType::CAUCHY;
  static constexpr MapType MAP = MapType::CUDA_VOXELHASHMAP;
  static constexpr ModelType MODEL = ModelType::EXTERNALGUESS;
  static constexpr PreprocessingType PREPROCESSING = PreprocessingType::CUDA_LIDAR;
  static constexpr RegistrationType REGISTRATION = RegistrationType::CUDA_GICP;
  static constexpr ThresholdType THRESHOLD = ThresholdType::ADAPTIVETHRESHOLD;
  static constexpr VelocityType VELOCITY = VelocityType::DERIVATIVE;
};

/**
 * @brief Configuration for an XYZ point type (i.e. from the offline map)
 */
struct Point_XYZ {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = false;
  static constexpr bool COV = false;
  static constexpr bool SEG = false;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
};

/**
 * @brief Configuration for testing of map functionality
 * @note This configuration is used for testing purposes only and should not be used in production.
 */
struct POINT_NORMAL {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = true;
  static constexpr bool COV = true;
  static constexpr bool SEG = true;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 20;
  static constexpr int16_t NUM_NEIGHBORS = 5;
  static constexpr RobustKernelType KERNEL = RobustKernelType::GEMANMCCLURE;
};

/**
 * @brief Define concept for normal and covariance
 */
template <typename TConfig>
concept HASNORMALCOV = TConfig::NORMALS && TConfig::COV;

/**
 * @brief Define concept for segmentation
 */
template <typename TConfig>
concept HASSEG = TConfig::SEG;
/**
 * @brief Define concept for frame map instance
 */
template <typename TConfig>
concept FRAMEMAP =
  TConfig::REGISTRATION == RegistrationType::GICP || TConfig::REGISTRATION == RegistrationType::CUDA_GICP;
/**
 * @brief Define concept for radar configurations with radar-specific attributes
 */
template <typename TConfig>
concept HASRADAR =
  (TConfig::VEL) && (TConfig::RCS) && (TConfig::SNR) && (TConfig::CONFIDENCE) && (TConfig::VEL_INTERVAL);
/**
 * @brief Define concept for the DCR velocity handler. DCR needs per-point Doppler
 *        (VEL), velocity interval (VEL_INTERVAL) and the original sensor-frame
 *        spherical coordinates (SPHERICAL) so it can use the true line-of-sight
 *        azimuth instead of one derived from body-frame Cartesian coordinates.
 */
template <typename TConfig>
concept VELOCITY_DCR = TConfig::VEL && TConfig::VEL_INTERVAL && TConfig::SPHERICAL;
}  // namespace tam::core::state::types
