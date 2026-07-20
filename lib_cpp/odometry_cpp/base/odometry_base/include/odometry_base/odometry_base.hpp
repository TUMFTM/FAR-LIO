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

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "odometry_types/odometry_config.hpp"
#include "odometry_types/odometry_types.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

namespace tam::core::state {
template <typename TConfig, typename CONFIG, typename DEBUG>
class OdometryBase
{
public:
  // In the final class, create the following, static constructors
  // static std::unique_ptr<BaseClass<TEMPLATE_PARAM>> from_config(
  //   tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  // {
  //   std::unique_ptr<Module<TEMPLATE_PARAM>> mh =
  //     std::unique_ptr<Module<TEMPLATE_PARAM>>(
  //       new Module<TEMPLATE_PARAM>(pmg, logger));
  //   return mh;
  // }
  // static std::unique_ptr<BaseClass<TEMPLATE_PARAM>> from_config(
  //   const types::Config & config, const types::Debug & debug)
  // {
  //   std::unique_ptr<Module<TEMPLATE_PARAM>> mh =
  //     std::unique_ptr<Module<TEMPLATE_PARAM>>(
  //       new Module<TEMPLATE_PARAM>(config, debug));
  //   return mh;
  // }
  /**
   * @brief Get the param object
   */
  CONFIG& get_config() { return config_; }

  /**
   * @brief Get the debug object
   */
  DEBUG get_debug() const { return debug_; }

protected:
  /**
   * @brief Constructor from param manager and logger
   */
  OdometryBase(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
  {
    // Inherit this constructor and call the set functions for params and logging
    // e.g.
    // Module(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
    // : OdometryBase<types::Config, types::Debug>(pmg, logger)
    // {
    //   setup_config(pmg);
    //   setup_logging(logger);
    //   // Additional initialization
    // }
    (void)pmg;
    (void)logger;
  }

  /**
   * @brief Constructor for config and debug object
   */
  OdometryBase(const CONFIG& config, const DEBUG& debug) : config_(config), debug_(debug)
  {
    // Inherit this constructor and call the set functions for params and logging
    // e.g.
    // Module(const types::Config & config, const types::Debug & debug)
    // : OdometryBase<types::Config, types::Debug>(config, debug)
    // {
    //   // Additional initialization
    // }
  }

  virtual void set_config(tam::pmg::ParamReferenceManager* pmg) = 0;
  virtual void set_logging(tam::tsl::ReferenceLogger* logger) const = 0;
  CONFIG config_;
  DEBUG debug_;
};
}  // namespace tam::core::state
