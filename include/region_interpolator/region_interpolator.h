#pragma once

#include "../types.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace roi_detector
{
/**
 * @brief Class for detecting region-of-interest boundaries
 */
struct RegionInterpolator
{
  virtual ~RegionInterpolator() = default;

  virtual pcl::PolygonMesh interpolate(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) const = 0;

};

} // namespace snp_isu_tpp
