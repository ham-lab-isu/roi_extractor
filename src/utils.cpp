#include "utils.h"

#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/extract_clusters.h>

namespace roi_detector
{
void ColorClusteringParameters::setHLimits(float h_min, float h_max)
{
  h_min_ = std::min(std::max(h_min, 0.0f), 360.0f);
  h_max_ = std::min(std::max(h_max, 0.0f), 360.0f);
}

void ColorClusteringParameters::setSLimits(float s_min, float s_max)
{
  s_min_ = std::min(std::max(s_min, 0.0f), 360.0f);
  s_max_ = std::min(std::max(s_max, 0.0f), 360.0f);
}

void ColorClusteringParameters::setVLimits(float v_min, float v_max)
{
  v_min_ = std::min(std::max(v_min, 0.0f), 360.0f);
  v_max_ = std::min(std::max(v_max, 0.0f), 360.0f);
}

std::vector<pcl::PointCloud<pcl::PointXYZHSV>> detectColorClusters(const pcl::PolygonMesh& mesh, const ColorClusteringParameters& params_)
{
  // Extract vertices from mesh
  auto vertex_hsv = pcl::make_shared<pcl::PointCloud<pcl::PointXYZHSV>>();
  {
    auto vertex_rgb = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromPCLPointCloud2(mesh.cloud, *vertex_rgb);

    // Convert to HSV
    pcl::PointCloudXYZRGBtoXYZHSV(*vertex_rgb, *vertex_hsv);
  }

  // Apply PassThrough filters for h, s, and v channels
  pcl::PassThrough<pcl::PointXYZHSV> h_filter, s_filter, v_filter;

  // Configure and run h filter
  h_filter.setFilterFieldName("h");
  h_filter.setFilterLimits(params_.getHMin(), params_.getHMax());
  h_filter.setInputCloud(vertex_hsv);

  auto h_indices = pcl::make_shared<pcl::Indices>();
  h_filter.filter(*h_indices);

  // Configure and run s filter
  s_filter.setFilterFieldName("s");
  s_filter.setFilterLimits(params_.getSMin(), params_.getSMax());
  s_filter.setInputCloud(vertex_hsv);
  s_filter.setIndices(h_indices);

  auto s_indices = pcl::make_shared<pcl::Indices>();
  s_filter.filter(*s_indices);

  // Configure and run the v filter
  v_filter.setFilterFieldName("v");
  v_filter.setFilterLimits(params_.getVMin(), params_.getVMax());
  v_filter.setInputCloud(vertex_hsv);
  v_filter.setIndices(s_indices);

  auto v_indices = pcl::make_shared<pcl::Indices>();
  v_filter.filter(*v_indices);

  // Cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> cluster_ex;
  cluster_ex.setInputCloud(vertex_hsv);
  cluster_ex.setIndices(v_indices);
  cluster_ex.setClusterTolerance(params_.cluster_tolerance);
  cluster_ex.setMinClusterSize(params_.min_cluster_size);
  cluster_ex.setMaxClusterSize(params_.max_cluster_size);

  std::vector<pcl::PointIndices> clusters;
  cluster_ex.extract(clusters);

  // Convert clusters to boundaries
  std::vector<pcl::PointCloud<pcl::PointXYZHSV>> cluster_clouds;
  cluster_clouds.reserve(clusters.size());
  for(const pcl::PointIndices& cluster : clusters)
  {
    if (params_.leaf_size)
    {
      // Downsample the cluster
      pcl::VoxelGrid<pcl::PointXYZHSV> voxel_grid;
      voxel_grid.setLeafSize(*params_.leaf_size, *params_.leaf_size, *params_.leaf_size);
      voxel_grid.setInputCloud(vertex_hsv);

      auto cluster_indices = pcl::make_shared<pcl::Indices>(cluster.indices);
      voxel_grid.setIndices(cluster_indices);

      pcl::PointCloud<pcl::PointXYZHSV> filtered_cloud;
      voxel_grid.filter(filtered_cloud);

      cluster_clouds.push_back(filtered_cloud);
    }
    else
    {
      pcl::ExtractIndices<pcl::PointXYZHSV> extract;
      extract.setInputCloud(vertex_hsv);

      auto cluster_indices = pcl::make_shared<pcl::Indices>(cluster.indices);
      extract.setIndices(cluster_indices);

      pcl::PointCloud<pcl::PointXYZHSV> extracted_cloud;
      extract.filter(extracted_cloud);

      cluster_clouds.push_back(extracted_cloud);
    }
  }

  return cluster_clouds;
}

pcl::PointXYZ createPoint(const Eigen::Vector4f& pt)
{
  pcl::PointXYZ point;
  point.x = pt.x();
  point.y = pt.y();
  point.z = pt.z();

  return point;
}

pcl::PointCloud<pcl::PointXYZ> createPolygon(const Boundary& b)
{
  pcl::PointCloud<pcl::PointXYZ> poly;

  poly.resize(b.cols());

  poly.getMatrixXfMap() = b;

  return poly;
}

} // namespace snp_isu_tpp
