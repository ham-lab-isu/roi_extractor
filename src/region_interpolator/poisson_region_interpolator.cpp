#include "region_interpolator/poisson_region_interpolator.h"
#include "boundary_extractor/closed_boundary_extractor.h"

#include <algorithm>

#include <pcl/io/vtk_lib_io.h>

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>

//#include <pcl/surface/poisson.h>

namespace roi_detector
{

PoissonRegionInterpolator::PoissonRegionInterpolator()
{
}

pcl::PolygonMesh PoissonRegionInterpolator::interpolate(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) const
{
  //POISSON
//  pcl::PointCloud<pcl::PointXYZ>::Ptr between_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::io::loadPCDFile<pcl::PointXYZ>("betweenCloud.pcd", *between_cloud);

  //https://github.com/atduskgreg/pcl-poisson-example/blob/master/poisson_recon.cpp

//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//  ne.setSearchMethod (tree);
//  ne.setInputCloud(cloud);
//  ne.setRadiusSearch(0.02);

//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
//  ne.compute(*cloud_normals);

//  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
//  pcl::concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(6);
  //poisson.setPointWeight(6);
  // READ INTO POISSON MORE, IMPROVE PARAMETERS
  poisson.setInputCloud(cloud);
  pcl::PolygonMesh poisson_mesh;
  poisson.reconstruct(poisson_mesh);

//  // Convert the mesh to a point cloud
//  pcl::PointCloud<pcl::PointXYZ>::Ptr interpolatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::fromPCLPointCloud2(poisson_mesh.cloud, *interpolatedCloud);

  // Save the reconstructed cloud
  //pcl::io::savePCDFile<pcl::PointXYZ>("reconstructed_cloud.pcd", *reconstructedCloud);

  //extractor.extractInside(poisson_mesh, ordered_boundary_pairs[1].second);

  return poisson_mesh;
}

} // namespace snp_isu_tpp
