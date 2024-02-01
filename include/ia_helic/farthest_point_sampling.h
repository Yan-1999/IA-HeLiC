/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <pcl/common/geometry.h>

#include <climits>
#include <random>
#include "pcl/pcl_base.h"

namespace pcl
{
/** \brief @b FarthestPointSampling applies farthest point sampling using euclidean
 * distance, starting with a random point, utilizing a naive method.
 * \author Haritha Jayasinghe
 * \ingroup filters
 * \todo add support to export/import distance metric
 */
template <typename PointT>
class FarthestPointSampling : public FilterIndices<PointT>
{
  using PCLBase<PointT>::input_;
  using PCLBase<PointT>::indices_;
  using Filter<PointT>::filter_name_;
  using FilterIndices<PointT>::keep_organized_;
  using FilterIndices<PointT>::user_filter_value_;
  using FilterIndices<PointT>::extract_removed_indices_;
  using FilterIndices<PointT>::removed_indices_;

  using typename FilterIndices<PointT>::PointCloud;

public:
  /** \brief Empty constructor. */
  FarthestPointSampling(bool extract_removed_indices = false)
    : FilterIndices<PointT>(extract_removed_indices)
    , sample_size_(std::numeric_limits<int>::max())
    , seed_(std::random_device()())
  {
    filter_name_ = "FarthestPointSamping";
  }

  /** \brief Set number of points to be sampled.
   * \param sample_size the number of points to sample
   */
  inline void setSample(std::size_t sample_size)
  {
    sample_size_ = sample_size;
  }

  /** \brief Get the value of the internal \a sample_size parameter.
   */
  inline std::size_t getSample() const
  {
    return (sample_size_);
  }

  /** \brief Set seed of random function.
   * \param seed for the random number generator, to choose the first sample point
   */
  inline void setSeed(unsigned int seed)
  {
    seed_ = seed;
  }

  /** \brief Get the value of the internal \a seed_ parameter.
   */
  inline unsigned int getSeed() const
  {
    return (seed_);
  }

protected:
  /** \brief Number of points that will be returned. */
  std::size_t sample_size_;
  /** \brief Random number seed. */
  unsigned int seed_;

  /** \brief Sample of point indices
   * \param indices indices of the filtered point cloud
   */
  void applyFilter(pcl::Indices& indices) override;
  void applyFilter(PointCloud& indices) override;
};
}  // namespace pcl

template <typename PointT>
void pcl::FarthestPointSampling<PointT>::applyFilter(pcl::Indices& indices)
{
  const std::size_t size = input_->size();
  // if requested number of point is equal to the point cloud size, copy original cloud
  if (sample_size_ == size)
  {
    indices = *indices_;
    removed_indices_->clear();
    return;
  }
  // check if requested number of points is greater than the point cloud size
  if (sample_size_ > size)
  {
    PCL_THROW_EXCEPTION(BadArgumentException, "Requested number of points is greater than point cloud size!");
  }

  std::vector<float> distances_to_selected_points(size, std::numeric_limits<float>::max());

  // set random seed
  std::mt19937 random_gen(seed_);
  std::uniform_int_distribution<std::int32_t> dis(0, size - 1);

  // pick the first point at random
  std::int32_t max_index = dis(random_gen);
  distances_to_selected_points[max_index] = -1.0;
  indices.push_back(max_index);

  for (std::size_t j = 1; j < sample_size_; ++j)
  {
    std::int32_t next_max_index = 0;

    const PointT& max_index_point = (*input_)[max_index];
    // recompute distances
    for (std::size_t i = 0; i < size; ++i)
    {
      if (distances_to_selected_points[i] == -1.0)
        continue;
      distances_to_selected_points[i] =
          std::min(distances_to_selected_points[i], geometry::distance((*input_)[i], max_index_point));
      if (distances_to_selected_points[i] > distances_to_selected_points[next_max_index])
        next_max_index = i;
    }

    // select farthest point based on previously calculated distances
    // since distance is set to -1 for all selected elements,previously selected
    // elements are guaranteed to not be selected
    max_index = next_max_index;
    distances_to_selected_points[max_index] = -1.0;
    indices.push_back(max_index);
    // set distance to -1 to ignore during max element search
  }

  if (extract_removed_indices_)
  {
    for (std::size_t k = 0; k < distances_to_selected_points.size(); ++k)
    {
      if (distances_to_selected_points[k] != -1.0)
        (*removed_indices_).push_back(k);
    }
  }
}

template <typename PointT>
void pcl::FarthestPointSampling<PointT>::applyFilter(PointCloud& output)
{
  Indices indices;
  if (keep_organized_)
  {
    if (!extract_removed_indices_)
    {
      PCL_WARN(
          "[pcl::FilterIndices<PointT>::applyFilter] extract_removed_indices_ was set to 'true' to keep the point "
          "cloud organized.\n");
      extract_removed_indices_ = true;
    }
    applyFilter(indices);

    output = *input_;

    // To preserve legacy behavior, only coordinates xyz are filtered.
    // Copying a PointXYZ initialized with the user_filter_value_ into a generic
    // PointT, ensures only the xyz coordinates, if they exist at destination,
    // are overwritten.
    const PointXYZ ufv(user_filter_value_, user_filter_value_, user_filter_value_);
    for (const auto ri : *removed_indices_)  // ri = removed index
      copyPoint(ufv, output[ri]);
    if (!std::isfinite(user_filter_value_))
      output.is_dense = false;
  }
  else
  {
    output.is_dense = true;
    applyFilter(indices);
    pcl::copyPointCloud(*input_, indices, output);
  }
}
