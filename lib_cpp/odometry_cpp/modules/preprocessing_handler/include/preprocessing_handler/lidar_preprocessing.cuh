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

#include <eigen3/Eigen/Core>
#include <memory>
#include <vector>

#include "preprocessing_handler/preprocessing_handler_base.hpp"

//
namespace tam::core::state::cuda {
/**
 * @brief Lidar preprocessing pipeline: spatial cropping → vehicle footprint filtering.
 */
template <typename TConfig>
class LidarPreprocessing : public PreprocessingHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<PreprocessingHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
  {
    std::unique_ptr<LidarPreprocessing<TConfig>> ph =
      std::unique_ptr<LidarPreprocessing<TConfig>>(new LidarPreprocessing<TConfig>(pmg, logger));
    return ph;
  }

  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<LidarPreprocessing<TConfig>> from_config(
    const types::PreprocessingConfig& config, const types::PreprocessingDebug& debug)
  {
    std::unique_ptr<LidarPreprocessing<TConfig>> ph =
      std::unique_ptr<LidarPreprocessing<TConfig>>(new LidarPreprocessing<TConfig>(config, debug));
    return ph;
  }

  __host__ bool preprocess(thrust::device_vector<types::Point<TConfig>>& frame, ::cuda::stream_ref stream = {}) override
  {
    // Crop points to range in place
    if (this->config_.crop_range.size() == 2) {
      this->crop_points(frame, stream);
    }
    if (frame.empty()) return false;
    // Vehicle footprint filtering if configured and point type has radar attributes
    if (this->config_.crop_footprint.size() == 2) {
      this->crop_footprint(frame, stream);
    }
    return frame.size() > 0;
  }

  /**
   * @brief  Init parameters
   */
  void init() override
  {
    // No additional initialization needed for lidar preprocessing
  }

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  LidarPreprocessing(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : PreprocessingHandler<TConfig>(pmg, logger)
  {
  }

  // Inherit constructor from ModelHandler for config and debug object
  LidarPreprocessing(const types::PreprocessingConfig& config, const types::PreprocessingDebug& debug)
      : PreprocessingHandler<TConfig>(config, debug)
  {
  }
};
}  // namespace tam::core::state::cuda
