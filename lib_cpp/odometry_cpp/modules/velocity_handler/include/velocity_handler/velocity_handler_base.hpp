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
#include <sophus/se3.hpp>
#include <vector>

#include "odometry_base/odometry_base.hpp"
#include "odometry_types/odometry_types.hpp"
#include "odometry_types/point_types.hpp"
#include "odometry_types/velocity.hpp"
namespace tam::core::state
{
/**
 * @brief Base class for velocity (twist) estimation modules.
 *
 */
template <typename TConfig>
class VelocityHandler : public OdometryBase<TConfig, types::VelocityConfig, types::VelocityDebug>
{
public:
  /**
   * @brief Estimate the body-frame twist for the current frame and registered pose.
   * @param[in] frame                    The input frame of points to estimate velocity from.
   * @param[in] pose_registered          The current registered pose, used for timestamping the result.
   * @return The estimated twist as a tangent vector, stamped with the input pose's timestamp.
   */
  virtual types::TangentStamped get_tangent(
    [[maybe_unused]] const std::vector<types::Point<TConfig>> & frame,
    const types::PoseStamped & pose_registered) = 0;

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  VelocityHandler(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : OdometryBase<TConfig, types::VelocityConfig, types::VelocityDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
  }
  // Inherit constructor from OdometryBase for config and debug object
  VelocityHandler(const types::VelocityConfig & config, const types::VelocityDebug & debug)
  : OdometryBase<TConfig, types::VelocityConfig, types::VelocityDebug>(config, debug)
  {
  }
  /**
   * @brief Declare the velocity-handler config parameters via the param manager.
   */
  void set_config([[maybe_unused]] tam::pmg::ParamReferenceManager * pmg) override {}
  /**
   * @brief Register debug variables with the logger.
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    logger->log("velocity/velocity_time", &this->debug_.velocity_time);
    logger->log("velocity/conditional", &this->debug_.conditional);
  }
};
}  // namespace tam::core::state
