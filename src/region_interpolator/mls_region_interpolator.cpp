#include "region_interpolator/mls_region_interpolator.h"

#include <algorithm>

#include <pcl/io/vtk_lib_io.h>

#include <pcl/surface/mls.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <pcl/conversions.h>

#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>


//#include <pcl/surface/poisson.h>

namespace roi_detector
{

MLSRegionInterpolator::MLSRegionInterpolator()
{
}

pcl::PolygonMesh MLSRegionInterpolator::interpolate(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) const
{ 
//  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
//  normalEstimation.setSearchMethod(tree);

//  pcl::PointCloud<pcl::PointXYZRGBNormal> new_cloud;
//  pcl::copyPointCloud(cloud, *new_cloud);

//  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//  // Init object (second point type is for the normals, even if unused)
//  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
//  mls.setComputeNormals (true);
//  // Set parameters
//  mls.setInputCloud (new_cloud);
//  mls.setPolynomialOrder (2);
//  mls.setSearchMethod (tree);
//  mls.setSearchRadius (0.04); //0.04
//  // Reconstruct
//  mls.process (*mls_points);
// Load point cloud from a PCD file

// NEW
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
//if (pcl::io::loadPCDFile<pcl::PointXYZ>("betweenCloud.pcd", *cloud_in) == -1)
//  throw std::runtime_error("Failed to load cloud_in");

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//mls.setInputCloud(cloud);
//mls.setPolynomialOrder(2);
//mls.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
//mls.setSearchRadius(0.04);
//mls.process(*cloud_out);

//  if(pcl::io::savePCDFile("new_cloud.pcd", *cloud_out) < 0)
//    throw std::runtime_error("Failed to save new_cloud");

//  pcl::PointCloud<pcl::PointXYZ>::Ptr between_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::io::loadPCDFile<pcl::PointXYZ>("between_cloud.pcd", *between_cloud);

//  // Create the Poisson reconstruction object
//  pcl::Poisson<pcl::PointXYZ> poisson;
//  poisson.setDepth(8); // Set the depth level of the octree
//  poisson.setInputCloud(between_cloud);

//  // Perform the surface reconstruction
//  pcl::PolygonMesh mesh;
//  poisson.reconstruct(mesh);

//  // Convert the mesh to a point cloud
//  pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::fromPCLPointCloud2(mesh.cloud, *reconstructedCloud);

//  // Save the reconstructed cloud
//  pcl::io::savePCDFile<pcl::PointXYZ>("reconstructed_cloud.pcd", *reconstructedCloud);


// Testing Moving Least Squares interpolator

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>("betweenCloud.pcd", *cloud_in) == -1)
//      throw std::runtime_error("Failed to load cloud_in");

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//    mls.setInputCloud(cloud_in);
//    mls.setPolynomialOrder(2);
//    mls.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
//    mls.setSearchRadius(0.04);
//    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
//    mls.setDilationVoxelSize(0.01);  // Voxel grid size for dilation
//    mls.setDilationIterations(2);    // Number of dilation iterations
//    mls.process(*cloud_out);

//    if(pcl::io::savePCDFile("interpolatedCloud.pcd", *cloud_out) < 0)
//      throw std::runtime_error("Failed to save interpolatedCloud");
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//ne.setSearchMethod (tree);
//ne.setInputCloud(cloud);
//ne.setRadiusSearch(0.02);

//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
//ne.compute(*cloud_normals);

//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
//pcl::concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);
  pcl::PolygonMesh mesh;

  return mesh;
}

} // namespace snp_isu_tpp
