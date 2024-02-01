#include "ia_helic/surfel_map.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <vector>

#include "Eigen/Core"
#include "fmt/format.h"
#include "ia_helic/farthest_point_sampling.h"
#include "ia_helic/utils.h"
#include "ia_helic/sensor_system.h"
#include "omp.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/common/distances.h"
#include "pcl/common/io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/pcl_base.h"
#include "pcl/point_cloud.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pclomp/ndt_omp.h"
#include "pclomp/voxel_grid_covariance_omp.h"

#ifdef PCL_NO_PRECOMPILE
#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"
#endif

template class pclomp::NormalDistributionsTransform<ia_helic::MapPoint, ia_helic::MapPoint>;
template class pclomp::VoxelGridCovariance<ia_helic::MapPoint>;

int checkPlaneType(const Eigen::Vector3d& eigen_value, const Eigen::Matrix3d& eigen_vector, const double& plane_lambda)
{
  Eigen::Vector3d sorted_vec;
  Eigen::Vector3i ind;
  ia_helic::argsortDesc(eigen_value, sorted_vec, ind);

  double p = 2 * (sorted_vec[1] - sorted_vec[2]) / (sorted_vec[2] + sorted_vec[1] + sorted_vec[0]);

  if (p < plane_lambda)
  {
    return -1;
  }

  int min_idx = ind[2];
  Eigen::Vector3d plane_normal = eigen_vector.block<3, 1>(0, min_idx);
  plane_normal = plane_normal.array().abs();

  ia_helic::argsortDesc(plane_normal, sorted_vec, ind);
  return ind[2];
}

template <typename PointT>
bool fitPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Vector4d& coeffs, double distance_threshold,
              pcl::PointCloud<PointT>& cloud_inliers)
{
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::SACSegmentation<PointT> seg;  /// Create the segmentation object
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);

  seg.setInputCloud(cloud);
  seg.segment(inliers, coefficients);

  if (inliers.indices.size() < 20)
  {
    return false;
  }

  for (int i = 0; i < 4; i++)
  {
    coeffs(i) = coefficients.values[i];
  }

  pcl::copyPointCloud(*cloud, inliers, cloud_inliers);
  return true;
}

void ia_helic::SurfelMap::build(const pclomp::NormalDistributionsTransform<MapPoint, MapPoint>& ndt, ros::Time map_time)
{
  Eigen::Vector4d surf_coeff;
  map_time_ = map_time;
  for (auto& [_, leaf] : ndt.getTargetCells().getLeaves())
  {
    if (leaf.nr_points < MIN_POINTS_IN_CELL)
    {
      continue;
    }
    int plane_type = checkPlaneType(leaf.getEvals(), leaf.getEvecs(), plane_lambda_);
    if (false && plane_type < 0)
    {
      continue;
    }
    auto cloud_raw = leaf.pointList_.makeShared();
    auto cloud_inliers = pcl::make_shared<MapCloud>();
    if (!fitPlane(cloud_raw, surf_coeff, associated_radius_, *cloud_inliers))
    {
      continue;
    }

    surfels_.emplace_back(cloud_raw, cloud_inliers, surf_coeff);
  }
}

double point2PlaneDistance(const Eigen::Vector3d& pt, const Eigen::Vector4d& plane_coeff)
{
  Eigen::Vector3d normal = plane_coeff.head<3>();
  double dist = pt.dot(normal) + plane_coeff(3);
  dist = dist > 0 ? dist : -dist;

  return dist;
}

void associatePointsToSurfels(const pcl::KdTreeFLANN<ia_helic::MapPoint>& kd_tree,
                              const ia_helic::AlignedVector<ia_helic::Surfel>& surfels, double associate_radius,
                              ia_helic::AlignedVector<std::atomic_int>& out_surfel_id_vec)
{
  auto& cloud_map = *(kd_tree.getInputCloud());
  assert(out_surfel_id_vec.size() == cloud_map.size());
  for (std::size_t i = 0; i < out_surfel_id_vec.size(); i++)
  {
    out_surfel_id_vec[i] = -1;
  }

#pragma omp parallel for
  for (int s_idx = 0; s_idx < surfels.size(); s_idx++)
  {
    pcl::Indices indices;
    std::vector<float> dummy;

    auto& coeff = surfels[s_idx].coeff;
    auto& box_min = surfels[s_idx].box_min;
    auto& box_max = surfels[s_idx].box_max;
    double radius = 0.5 * (box_max - box_min).norm();
    auto center_xyz = (0.5 * (box_max + box_min)).cast<float>();
    ia_helic::MapPoint center{ center_xyz(0), center_xyz(1), center_xyz(2), ia_helic::ELiDARLabel::UNKNOWN_LIDAR, 0, 0 };
    kd_tree.radiusSearch(center, radius, indices, dummy);
    for (std::size_t p_idx : indices)
    {
      if (std::isfinite(cloud_map[p_idx].x) && std::isfinite(cloud_map[p_idx].y) && std::isfinite(cloud_map[p_idx].z) &&
          cloud_map[p_idx].x > box_min[0] && cloud_map[p_idx].x < box_max[0] && cloud_map[p_idx].y > box_min[1] &&
          cloud_map[p_idx].y < box_max[1] && cloud_map[p_idx].z > box_min[2] && cloud_map[p_idx].z < box_max[2])
      {
        Eigen::Vector3d point(cloud_map[p_idx].x, cloud_map[p_idx].y, cloud_map[p_idx].z);
        if (point2PlaneDistance(point, coeff) < associate_radius)
        {
          out_surfel_id_vec[p_idx] = s_idx;
        }
      }
    }
  }
}

void associatePointsToSurfels(const pcl::KdTreeFLANN<ia_helic::MapPoint>& kd_tree,
                              const ia_helic::AlignedVector<ia_helic::Surfel>& surfels,
                              ia_helic::RigidTransform T_spoint_to_surfel, double associate_radius,
                              ia_helic::AlignedVector<std::atomic_int>& out_surfel_id_vec)
{
  auto& cloud_map = *(kd_tree.getInputCloud());
  assert(out_surfel_id_vec.size() == cloud_map.size());
  for (std::size_t i = 0; i < out_surfel_id_vec.size(); i++)
  {
    out_surfel_id_vec[i] = -1;
  }
  ia_helic::RigidTransform T_surfel_to_spoint = T_spoint_to_surfel.inverse();

#pragma omp parallel for
  for (int s_idx = 0; s_idx < surfels.size(); s_idx++)
  {
    pcl::Indices indices;
    std::vector<float> dummy;

    Eigen::Vector4d coeff = surfels[s_idx].coeff;
    Eigen::Vector3d box_min = surfels[s_idx].box_min;
    Eigen::Vector3d box_max = surfels[s_idx].box_max;
    double radius = 0.5 * (box_max - box_min).norm();
    Eigen::Vector3d center_xyz = T_surfel_to_spoint * (0.5 * (box_max + box_min));
    ia_helic::MapPoint center{ static_cast<float>(center_xyz(0)),
                            static_cast<float>(center_xyz(1)),
                            static_cast<float>(center_xyz(2)),
                            ia_helic::ELiDARLabel::UNKNOWN_LIDAR,
                            0,
                            0 };
    kd_tree.radiusSearch(center, radius, indices, dummy);
    for (std::size_t p_idx : indices)
    {
      Eigen::Vector3d point(cloud_map[p_idx].x, cloud_map[p_idx].y, cloud_map[p_idx].z);
      Eigen::Vector3d point_in_surfel = T_spoint_to_surfel * point;
      if (std::isfinite(point_in_surfel(0)) && std::isfinite(point_in_surfel(1)) && std::isfinite(point_in_surfel(2)) &&
          point_in_surfel(0) > box_min[0] && point_in_surfel(0) < box_max[0] && point_in_surfel(1) > box_min[1] &&
          point_in_surfel(1) < box_max[1] && point_in_surfel(2) > box_min[2] && point_in_surfel(2) < box_max[2] &&
          point2PlaneDistance(point_in_surfel, coeff) < associate_radius)
      {
        out_surfel_id_vec[p_idx] = s_idx;
      }
    }
  }
}

void ia_helic::SurfelMap::associateMultipleClouds(const pcl::KdTreeFLANN<MapPoint>& kd_tree,
                                               const AlignedVector<RawCloud::ConstPtr>& raw_clouds)
{
  if (surfels_.empty())
  {
    IA_HELIC_THROW("Surfels map is empty! ");
  }
  if (raw_clouds.empty())
  {
    IA_HELIC_THROW("`raw_clouds[]` is empty!");
  }

  auto& cloud_map = *(kd_tree.getInputCloud());
  spoints_.reserve(cloud_map.size());
  spoints_in_surfel_.resize(surfels_.size());

  AlignedVector<std::atomic_int> surfel_id(cloud_map.size());
  associatePointsToSurfels(kd_tree, surfels_, associated_radius_, surfel_id);

  for (std::size_t i = 0; i < cloud_map.size(); i++)
  {
    if (surfel_id[i] < 0)
    {
      continue;
    }
    auto& map_point = cloud_map[i];
    LiDARLabel lidar_label = map_point.label;
    std::uint32_t point_id = map_point.pindex;
    auto& raw_point = raw_clouds.at(lidar_label)->at(point_id);
    SurfelPoint sp{ raw_point, map_point, lidar_label, static_cast<std::size_t>(surfel_id[i]) };
    spoints_.push_back(sp);
    spoints_in_surfel_[surfel_id[i]].push_back(sp);
  }
  spoints_.shrink_to_fit();
  std::sort(spoints_.begin(), spoints_.end(),
            [](const SurfelPoint& lhs, const SurfelPoint& rhs) { return lhs.raw_point.t < rhs.raw_point.t; });
}

void ia_helic::SurfelMap::associateInliers(const RawCloud::ConstPtr raw_cloud)
{
  if (surfels_.empty())
  {
    IA_HELIC_THROW("Surfels map is empty!");
  }
  if (raw_cloud->empty())
  {
    IA_HELIC_THROW("Raw cloud is empty!");
  }
  spoints_.reserve(raw_cloud->size());
  spoints_in_surfel_.resize(surfels_.size());
#pragma omp parallel for
  for (std::size_t i_surfel = 0; i_surfel < surfels_.size(); i_surfel++)
  {
    auto& spoints_in_this_surfel = spoints_in_surfel_[i_surfel];
    spoints_in_this_surfel.reserve(surfels_[i_surfel].cloud_inlier->size());
    for (const MapPoint& map_point : *(surfels_[i_surfel].cloud_inlier))
    {
      std::uint32_t point_id = map_point.pindex;
      auto& raw_point = raw_cloud->at(point_id);
      SurfelPoint sp{ raw_point, map_point, label_, i_surfel };
      spoints_in_this_surfel.push_back(sp);
    }
#pragma omp critical
    {
      std::copy(spoints_in_this_surfel.begin(), spoints_in_this_surfel.end(), std::back_inserter(spoints_));
    }
  }
  spoints_.shrink_to_fit();
  std::sort(spoints_.begin(), spoints_.end(),
            [](const SurfelPoint& lhs, const SurfelPoint& rhs) { return lhs.raw_point.t < rhs.raw_point.t; });
}

void ia_helic::SurfelMap::associateCloud(const pcl::KdTreeFLANN<MapPoint>& kd_tree, const RawCloud::ConstPtr raw_cloud)
{
  if (surfels_.empty())
  {
    IA_HELIC_THROW("Surfels map is empty!");
  }
  if (raw_cloud->empty())
  {
    IA_HELIC_THROW("Raw cloud is empty!");
  }

  auto& cloud_map = *(kd_tree.getInputCloud());
  spoints_.reserve(cloud_map.size());
  spoints_in_surfel_.resize(surfels_.size());

  AlignedVector<std::atomic_int> surfel_id(cloud_map.size());
  associatePointsToSurfels(kd_tree, surfels_, associated_radius_, surfel_id);

  for (std::size_t i = 0; i < cloud_map.size(); i++)
  {
    if (surfel_id[i] < 0)
    {
      continue;
    }
    auto& map_point = cloud_map[i];
    std::uint32_t point_id = map_point.pindex;
    auto& raw_point = raw_cloud->at(point_id);
    SurfelPoint sp{ raw_point, map_point, label_, static_cast<std::size_t>(surfel_id[i]) };
    spoints_.push_back(sp);
    spoints_in_surfel_[surfel_id[i]].push_back(sp);
  }
  std::sort(spoints_.begin(), spoints_.end(),
            [](const SurfelPoint& lhs, const SurfelPoint& rhs) { return lhs.raw_point.t < rhs.raw_point.t; });
}

void ia_helic::SurfelMap::getColoredSurfels(ColoredCloud& cloud) const
{
  constexpr std::array<int, 6> COLOR_LIST = {
    0xFF0000, 0xFF00FF, 0x436EEE, 0xBF3EFF, 0xB4EEB4, 0xFFE7BA,
  };

  cloud.clear();
  size_t index = 0;
  ColoredCloud cloud_rgb;
  for (const auto& surfel : surfels_)
  {
    cloud_rgb.clear();
    pcl::copyPointCloud(*surfel.cloud_inlier, cloud_rgb);
    int color = COLOR_LIST[index];
    index++;
    index %= COLOR_LIST.size();
    for (auto& p : cloud_rgb)
    {
      p.rgba = color;
    }
    cloud += cloud_rgb;
  }
}

void ia_helic::SurfelMap::downSampleSimple(int rate)
{
  if (spoints_.empty())
  {
    IA_HELIC_THROW("No surfel points!");
  }

  spoints_in_surfel_.clear();

  spoints_down_sampled_.reserve(spoints_.size() / rate);
  for (std::size_t i = 0; i < spoints_.size(); i += rate)
  {
    spoints_down_sampled_.push_back(spoints_[i]);
  }
}

void ia_helic::SurfelMap::downSampleFarthest(int rate, std::size_t n_lidar, double time_span)
{
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  if (spoints_.empty())
  {
    IA_HELIC_THROW("No surfel points!");
  }

  pcl::Indices indices;
  AlignedVector<PointCloud::Ptr> cloud_of_lidar(n_lidar);
  AlignedVector<pcl::Indices> indices_cloud_of_lidar(n_lidar);
  double last_t;
  pcl::FarthestPointSampling<PointT> fps;

  spoints_down_sampled_.clear();
  spoints_down_sampled_.reserve(spoints_.size() / rate);

  for (auto& p_cloud : cloud_of_lidar)
  {
    p_cloud = pcl::make_shared<PointCloud>();
  }

  for (std::size_t isp = 0; isp < spoints_.size(); isp++)
  {
    double t = spoints_[isp].raw_point.t;
    auto& map_p = spoints_[isp].map_point;
    if (t - last_t > time_span)
    {
      last_t = t;
      for (std::size_t il = 0; il < n_lidar; il++)
      {
        indices.clear();
        fps.setInputCloud(cloud_of_lidar[il]);
        fps.setSample(std::min(cloud_of_lidar[il]->size() / rate + 1, cloud_of_lidar[il]->size()));
        fps.filter(indices);
        for (std::size_t idx : indices)
        {
          spoints_down_sampled_.push_back(spoints_[indices_cloud_of_lidar[il][idx]]);
        }
        cloud_of_lidar[il]->clear();
        indices_cloud_of_lidar[il].clear();
      }
    }
    LiDARLabel lidar_label = spoints_[isp].lidar_label;
    cloud_of_lidar[lidar_label]->emplace_back(map_p.x, map_p.y, map_p.z);
    indices_cloud_of_lidar[lidar_label].push_back(isp);
  }
}

void ia_helic::CrossSurfelMap::associate(const pcl::KdTreeFLANN<MapPoint>& kd_tree,
                                      const RigidTransform& T_spoint_to_surfel, const MapCloud& raw_cloud)
{
  auto& surfels = surfel_map_->surfels();
  LiDARLabel label = surfel_map_->label();
  if (surfels.empty())
  {
    IA_HELIC_THROW("Surfels map is empty!");
  }
  if (raw_cloud.empty())
  {
    IA_HELIC_THROW("`raw_clouds` is empty!");
  }

  auto& spoint_cadidate = *(kd_tree.getInputCloud());
  spoints_.reserve(spoint_cadidate.size());

  AlignedVector<std::atomic_int> surfel_id(spoint_cadidate.size());
  associatePointsToSurfels(kd_tree, surfels, T_spoint_to_surfel, associated_radius_, surfel_id);

  for (std::size_t i = 0; i < spoint_cadidate.size(); i++)
  {
    if (surfel_id[i] < 0)
    {
      continue;
    }
    auto& spoint = spoint_cadidate[i];
    std::uint32_t point_id = spoint.pindex;
    auto& raw_point = raw_cloud.at(point_id);
    spoints_.emplace_back<SurfelPoint>({ raw_point, spoint, label, static_cast<std::size_t>(surfel_id[i]) });
  }
  spoints_.shrink_to_fit();
  std::sort(spoints_.begin(), spoints_.end(),
            [](const SurfelPoint& lhs, const SurfelPoint& rhs) { return lhs.raw_point.t < rhs.raw_point.t; });
}

void ia_helic::CrossSurfelMap::getColoredSurfelPoints(ColoredCloud& cloud) const
{
  constexpr std::array<int, 6> COLOR_LIST = {
    0xFF0000, 0xFF00FF, 0x436EEE, 0xBF3EFF, 0xB4EEB4, 0xFFE7BA,
  };
  cloud.resize(spoints_.size());
  for (std::size_t i = 0; i < spoints_.size(); i++)
  {
    auto& map_point = spoints_[i].map_point;
    cloud[i].x = map_point.x;
    cloud[i].y = map_point.y;
    cloud[i].z = map_point.z;
    cloud[i].rgba = COLOR_LIST[spoints_[i].plane_id % COLOR_LIST.size()];
  }
};
