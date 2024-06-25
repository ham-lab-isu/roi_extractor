#pragma once

#include "boundary_extractor.h"
#include "../utils.h"



namespace roi_detector
{
/**
 * @brief Class for detecting region-of-interest boundaries that are closed polygons
 */
class ClosedBoundaryExtractor : public BoundaryExtractor
{
public:
  ClosedBoundaryExtractor();

  pcl::PolygonMesh extractInside(const pcl::PolygonMesh& mesh, const Boundary& boundary) const override;
  pcl::PolygonMesh extractBetween(const pcl::PolygonMesh& mesh, const Boundary& insideBoundary, const Boundary& outsideBoundary) const override;
  pcl::PolygonMesh extractOutside(const pcl::PolygonMesh& mesh, const Boundary& boundary) const override;

private:
};

} // namespace snp_isu_tpp
