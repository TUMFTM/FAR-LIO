/*
 * Copyright 2023 Marcel Weinmann, Maximilian Leitenstern
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

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// general constants
#include "tum_helpers_cpp/constants.hpp"

// Param manager
#include "param_management_cpp/param_value_manager.hpp"

// Kalman Filter Base Class
#include "tam_state_estimation/kalman_filter/base.hpp"

// Outlier detection helper
#include "tam_state_estimation/helper/outlier_rejection.hpp"

namespace tam::core::state {
template <typename TConfig>
class EKF : public KFBase<TConfig>
{
public:
  /**
   * @brief Constructor
   */
  EKF();

  /**
   * @brief Correction step of the EKF
   *
   * @param[in] z             - Measurement vector
   */
  void update(const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>>& z)
  {
    // calculate the innovation/ measurement residuals
    residuals_raw_ = z - H_ * x_pred_;

    // normalize all residual angles
    for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
      for (int j = 0; j < TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; j++) {
        const uint8_t vec_offset =
          TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + j;
        residuals_raw_[vec_offset] = tam::helpers::geometry::normalize_angle(residuals_raw_[vec_offset]);
      }
    }

    // Calculate the innovation (residual) covariance S
    const Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> S =
      H_ * P_pred_ * H_.transpose() + R_;

    // Calculate the Kalman Gain K
    K_.noalias() = P_pred_ * H_.transpose() * S.inverse();

    // perform the outlier filtering
    // the maximum allowed residum is defined in the variable P_VDC_OutlierBounds
    residuals_raw_ = residuals_raw_.cwiseProduct(fusion_vec_);
    if (kalman_filter_params_.enable_mahalanobis_outlier_rejection) {
      residuals_ = outlier_rejection::mahalanobis_outlier_rejection<TConfig>(residuals_raw_, outlier_bound_, x_);
    } else {
      residuals_ = outlier_rejection::box_outlier_rejection(residuals_raw_, outlier_bound_);
    }

    // set the residual to zero if the sensor was not updated or is not valid
    residuals_ = residuals_.cwiseProduct(fusion_vec_);

    // update the state vector w.r.t the Kalman Gain
    x_ = x_pred_ + K_ * residuals_;

    // normalize all angles of the state vector
    for (int i = 0; i < TConfig::STATE_VECTOR_SIZE; i++) {
      if (TConfig::STATE_VECTOR_ANGLE_INDICATOR[i]) {
        x_[i] = tam::helpers::geometry::normalize_angle(x_[i]);
      }
    }

    // update the state covariance matrix
    P_.noalias() = (Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>::Identity() - K_ * H_) * P_pred_;
  }

  /**
   * @brief Prediction step of the EKF
   *
   * @param[in] u             - Input vector
   */
  void predict(const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>>& u);

  /**
   * @brief get all debug values of the kalman filter
   *
   * @param[out]              - std::unordered_map:
   *                            residuals of the Extended Kalman Filter
   */
  const tam::types::state::unordered_identifier_map<std::vector<std::pair<std::string, double>>>& get_debug(void);

private:
  // Variables
  /**
   * @brief Jacobian matrix A consisting of partial derivatives of the system update equation
   *        with respect to the system state
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> A_{};

  /**
   * @brief Jacobian matrix B consisting of partial derivatives of the system update equation
   *        with respect to the system input
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::INPUT_VECTOR_SIZE> B_{};

public:
  /**
   * @brief Tell the compiler that we are using the variables defined in KFBase
   */
  using KFBase<TConfig>::x_;
  using KFBase<TConfig>::x_pred_;
  using KFBase<TConfig>::P_;
  using KFBase<TConfig>::P_pred_;
  using KFBase<TConfig>::R_;
  using KFBase<TConfig>::R_adaptive_;
  using KFBase<TConfig>::Q_;
  using KFBase<TConfig>::H_;
  using KFBase<TConfig>::H_full_;
  using KFBase<TConfig>::K_;
  using KFBase<TConfig>::residuals_;
  using KFBase<TConfig>::residuals_raw_;
  using KFBase<TConfig>::outlier_bound_;
  using KFBase<TConfig>::fusion_vec_;
  using KFBase<TConfig>::param_manager_;
  using KFBase<TConfig>::kalman_filter_params_;
  using KFBase<TConfig>::previous_param_state_hash_;
  using KFBase<TConfig>::debug_output_;
};
}  // namespace tam::core::state

#include "tam_state_estimation/kalman_filter/ekf_2d.hpp"
#include "tam_state_estimation/kalman_filter/ekf_3d.hpp"
