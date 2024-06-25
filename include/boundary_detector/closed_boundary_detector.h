#pragma once

#include "boundary_detector.h"
#include "../utils.h"

namespace roi_detector
{
/**
 * @brief Class for detecting region-of-interest boundaries that are closed polygons
 */
class ClosedBoundaryDetector : public BoundaryDetector
{
public:
  ClosedBoundaryDetector(ColorClusteringParameters params);

  Boundaries detectAllBoundaries(const pcl::PolygonMesh& mesh) const override;
  std::vector<std::pair<Boundary, Boundary>> pairBoundaries(const Boundaries& boundaries) const override;

private:
  ColorClusteringParameters params_;
};

} // namespace snp_isu_tpp
