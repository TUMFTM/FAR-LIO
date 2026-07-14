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

#include <Eigen/Dense>
#include <vector>
#include <functional>

#include "odometry_utils/utils.hpp"
namespace tam::core::state::utils
{
/**
 * @brief Reduce a factor over all correspondences in parallel
 * @param [in] correspondences         Correspondences
 * @param [in] factor                  Factor to evaluate per correspondence
 * @param [in] reduce                  Binary operation to combine the factor outputs
 * @return                             Reduced output
 */
template <typename TConfig, typename T, typename FactorType, typename ReduceType>
T reduce_correspondences(
  std::vector<types::Correspondence<TConfig>> & correspondences, const FactorType factor,
  const ReduceType reduce)
{
  using correspondence_iterator = std::vector<types::Correspondence<TConfig>>::iterator;
  const auto & reduced = tbb::parallel_reduce(
    // Range
    tbb::blocked_range<correspondence_iterator>{correspondences.begin(), correspondences.end()},
    // Default initialize output
    T{},
    // 1st Lambda: Parallel computation
    [&](const tbb::blocked_range<correspondence_iterator> & r, T element) -> T {
      return std::transform_reduce(
        r.begin(), r.end(), element, reduce,
        [&](types::Correspondence<TConfig> & correspondence) { return factor(correspondence); });
    },
    // 2nd Lambda: Parallel reduction of the private Jacboians
    reduce);

  return reduced;
}
/**
 * @brief Build the linear system for the ICP algorithm
 * @param [in] correspondences         Correspondences
 * @param [in] factor                  Factor computing JTJ and JTr per correspondence
 * @return                             Linear system
 */
template <typename TConfig, typename FactorType>
types::LinearSystem build_linear_system(
  std::vector<types::Correspondence<TConfig>> & correspondences, const FactorType & factor)
{
  auto sum_linear_systems = [](types::LinearSystem a, const types::LinearSystem & b) {
    a.JTJ += b.JTJ;
    a.JTr += b.JTr;
    return a;
  };
  const types::LinearSystem & ls = reduce_correspondences<TConfig, types::LinearSystem>(
    correspondences, factor, sum_linear_systems);
  return ls;
}
/**
 * @brief Compute the error of a set of correspondences for a given transformation
 * @param [in] correspondences Correspondences
 * @param [in] factor          Error functor holding the transformation to apply
 * @return                     Transformation error
 */
template <typename TConfig, typename ErrorType>
float compute_error(
  std::vector<types::Correspondence<TConfig>> & correspondences, const ErrorType & factor)
{
  const float error =
    reduce_correspondences<TConfig, float>(correspondences, factor, std::plus<float>{});
  return error;
}
/**
 * @brief Get correspondences between points and map
 * @param [in] points                   Points to get correspondences for
 * @param [in] map                      Map to get correspondences from
 * @param [in] correspondence_threshold Correspondence threshold
 * @return                        Correspondences
 */
template <typename TConfig>
std::vector<types::Correspondence<TConfig>> get_correspondences(
  const std::vector<types::Point<TConfig>> & points, const MapHandler<TConfig> * map,
  const float correspondence_threshold)
{
  using points_iterator = std::vector<types::Point<TConfig>>::const_iterator;
  tbb::concurrent_vector<types::Correspondence<TConfig>> tbb_correspondences{};
  tbb_correspondences.reserve(points.size());
  float correspondence_threshold2 = correspondence_threshold * correspondence_threshold;

  // calculate all closest neighbors in parallel
  tbb::parallel_for(
    tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
    [&](const tbb::blocked_range<points_iterator> & r) {
      std::for_each(r.begin(), r.end(), [&](const auto & point) {
        const auto & correspondence = map->search_closest_neighbor(point, 1);
        if (correspondence.distance < correspondence_threshold2) {
          tbb_correspondences.emplace_back(correspondence);
        }
      });
    });

  // Combine thread-local results into a single std::vector
  std::vector<types::Correspondence<TConfig>> correspondences(
    tbb_correspondences.begin(), tbb_correspondences.end());

  return correspondences;
}
}  // namespace tam::core::state::utils
