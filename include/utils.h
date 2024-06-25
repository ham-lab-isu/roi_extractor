#pragma once

#include "types.h"

#include <boost/optional.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace roi_detector
{
/**
 * @brief Extracts a subset within the non-convex boundary from the input cloud
 */
std::vector<int> extractSubset(const pcl::PCLPointCloud2& cloud, const Boundary& boundary);
/**
 * @brief Extracts a submesh within the non-convex boundary from the input mesh
 */
pcl::PolygonMesh extractSubmesh(const pcl::PolygonMesh& mesh, const Boundary& boundary);

class ColorClusteringParameters
{
public:
  void setHLimits(float h_min, float h_max);
  void setSLimits(float s_min, float s_max);
  void setVLimits(float v_min, float v_max);

  float getHMin() const { return h_min_;}
  float getHMax() const { return h_max_; }
  float getVMin() const { return v_min_; }
  float getVMax() const { return v_max_; }
  float getSMin() const { return s_min_; }
  float getSMax() const { return s_max_; }

  boost::optional<float> leaf_size = {};
  double cluster_tolerance = 0.0;
  int min_cluster_size = 0;
  int max_cluster_size = std::numeric_limits<int>::max();

private:
  float h_min_ = 0.0;
  float h_max_ = 360.0;
  float s_min_ = 0.0;
  float s_max_ = 1.0;
  float v_min_ = 0.0;
  float v_max_ = 1.0;
};

std::vector<pcl::PointCloud<pcl::PointXYZHSV>> detectColorClusters(const pcl::PolygonMesh& mesh, const ColorClusteringParameters& params);

pcl::PointXYZ createPoint(const Eigen::Vector4f& pt);

pcl::PointCloud<pcl::PointXYZ> createPolygon(const Boundary& b);

}
