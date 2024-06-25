#pragma once

#include "region_interpolator.h"
#include "../utils.h"

namespace roi_detector
{
/**
 * @brief Class for detecting region-of-interest boundaries that are closed polygons
 */
class MLSRegionInterpolator : public RegionInterpolator
{
public:
  MLSRegionInterpolator();

  pcl::PolygonMesh interpolate(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) const override;

private:
};

} // namespace snp_isu_tpp
