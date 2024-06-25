#include "boundary_detector/closed_boundary_detector.h"

#include <algorithm>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <deque>

#include "utils.h"

namespace roi_detector
{

ClosedBoundaryDetector::ClosedBoundaryDetector(ColorClusteringParameters params)
  : params_(params)
{
}

Boundaries ClosedBoundaryDetector::detectAllBoundaries(const pcl::PolygonMesh& mesh) const
{
  std::vector<pcl::PointCloud<pcl::PointXYZHSV>> clusters = detectColorClusters(mesh, params_);

  // Extract the clouds into boundaries
  Boundaries boundaries;
  boundaries.reserve(clusters.size());

  const int dim = sizeof(pcl::PointXYZ) / sizeof(float);
  const int stride = sizeof(pcl::PointXYZHSV) / sizeof(float);
  for(const pcl::PointCloud<pcl::PointXYZHSV>& cluster : clusters)
    boundaries.push_back(cluster.getMatrixXfMap(dim, stride, 0));

  // Check which boundaries are closed polygons
  // How? Don't know yet...

  return boundaries;
}

std::vector<std::pair<Boundary, Boundary>> ClosedBoundaryDetector::pairBoundaries(const Boundaries& boundaries) const
{
  std::vector<std::pair<Boundary, Boundary>> boundary_pairs;

  // Enumerate the combinations of boundary indices to compare
  std::deque<std::pair<std::size_t, std::size_t>> compare_idx;
  for(std::size_t i = 0; i < boundaries.size(); ++i)
  {
    for(std::size_t j = 0; j < boundaries.size(); ++j)
    {
      if(i != j)
        compare_idx.push_back(std::make_pair(i, j));
    }
  }

  while(!compare_idx.empty())
  {
    std::size_t i, j;
    std::tie(i, j) = compare_idx.front();
    compare_idx.pop_front();

    // Make a polygon out of the ith boundary
    const Boundary& possible_outer_boundary = boundaries[i];
    pcl::PointCloud<pcl::PointXYZ> polygon = createPolygon(possible_outer_boundary);

    // Check if each point in jth boundary lies within the polygon of the ith boundary
    const Boundary& possible_inner_boundary = boundaries[j];
    bool all_inside = true;
    for(Eigen::Index c = 0; c < possible_inner_boundary.cols(); ++c)
    {
      pcl::PointXYZ point = createPoint(possible_inner_boundary.col(c));
      all_inside &= pcl::isPointIn2DPolygon(point, polygon);
      if(!all_inside)
        break;
    }

    if(all_inside)
    {
      boundary_pairs.push_back(std::make_pair(possible_inner_boundary, possible_outer_boundary));

      // Remove all other entries in compare_idx containing i or j
      auto it = std::remove_if(compare_idx.begin(), compare_idx.end(), [i, j](const std::pair<std::size_t, std::size_t>& pair){
        bool remove = false;
        remove |= pair.first == i;
        remove |= pair.first == j;
        remove |= pair.second == i;
        remove |= pair.second == j;
        return remove;
      });
      compare_idx.erase(it, compare_idx.end());
    }
  }

  return boundary_pairs;
}

} // namespace snp_isu_tpp
