#pragma once

#include "../types.h"

#include <pcl/PolygonMesh.h>
#include <vector>

namespace roi_detector
{
/**
 * @brief Class for detecting region-of-interest boundaries
 */
struct BoundaryDetector
{
  virtual ~BoundaryDetector() = default;

  virtual Boundaries detectAllBoundaries(const pcl::PolygonMesh& mesh) const = 0;
  virtual std::vector<std::pair<Boundary, Boundary>> pairBoundaries(const Boundaries& boundaries) const = 0;
};

} // namespace snp_isu_tpp
