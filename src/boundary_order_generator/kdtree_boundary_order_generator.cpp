#include "boundary_order_generator/kdtree_boundary_order_generator.h"

#include <numeric>

#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace roi_detector
{
KdTreeBoundaryOrderGenerator::KdTreeBoundaryOrderGenerator(int k) : k_(k)
{
}

Boundary KdTreeBoundaryOrderGenerator::order(const Boundary& boundary) const
{
  auto search = [this](const Boundary& boundary) -> std::vector<std::vector<int>>
  {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->resize(boundary.cols());
    cloud->getMatrixXfMap() = boundary;

    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    std::vector<int> all_idx(cloud->size());
    std::iota(all_idx.begin(), all_idx.end(), 0);

    std::vector<std::vector<int>> neighbors;
    std::vector<std::vector<float>> dists;
    tree.nearestKSearch(*cloud, all_idx, k_, neighbors, dists);

    return neighbors;
  };

  const std::vector<std::vector<int>> neighbors = search(boundary);
  std::vector<int> ordered_boundary_idx;
  ordered_boundary_idx.reserve(boundary.cols());

  // Add the first point
  int idx = 0;
  ordered_boundary_idx.push_back(idx);

  // Loop until we reach a point, all of whose neighbors have already been added to the ordered boundary list
  while(idx >= 0)
  {
    bool found = false;
    for(std::size_t i = 0; i < neighbors[idx].size(); ++i)
    {
      int neighbor = neighbors[idx][i];

      auto it = std::find(ordered_boundary_idx.begin(), ordered_boundary_idx.end(), neighbor);
      if(it == ordered_boundary_idx.end())
      {
        ordered_boundary_idx.push_back(neighbor);
        idx = neighbor;
        found = true;
        break;
      }
    }

    // Break the while loop if no neighbors were found that didn't already exist in the ordered boundary index list
    if(!found)
      break;
  }

  // Extract the boundary from the indices
  Boundary ordered_boundary(4, ordered_boundary_idx.size());
  for(int i = 0; i < ordered_boundary_idx.size(); ++i)
    ordered_boundary.col(i) = boundary.col(idx);

  return ordered_boundary;
}

} // namespace snp_isu_tpp
