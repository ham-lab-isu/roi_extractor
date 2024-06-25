#pragma once

#include "../types.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <vector>

namespace roi_detector
{
/**
 * @brief Class for detecting region-of-interest boundaries
 */
struct BoundaryExtractor
{
  virtual ~BoundaryExtractor() = default;

  virtual pcl::PolygonMesh extractInside(const pcl::PolygonMesh& mesh, const Boundary& boundary) const = 0;
  virtual pcl::PolygonMesh extractBetween(const pcl::PolygonMesh& mesh, const Boundary& insideBoundary, const Boundary& outsideBoundary) const = 0;
  virtual pcl::PolygonMesh extractOutside(const pcl::PolygonMesh& mesh, const Boundary& boundary) const = 0;

};

} // namespace snp_isu_tpp
