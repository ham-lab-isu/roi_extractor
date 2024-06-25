#include "boundary_detector/closed_boundary_detector.h"
#include "boundary_extractor/closed_boundary_extractor.h"
#include "boundary_order_generator/kdtree_boundary_order_generator.h"
#include "boundary_order_generator/crust_boundary_order_generator.h"
#include "boundary_order_generator/tsp_boundary_order_generator.h"
#include "utils.h"

#include "region_interpolator/poisson_region_interpolator.h"

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/surface/mls.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

using namespace roi_detector;

void saveBoundaryCloudsOrderVisible(const Boundaries& boundaries, std::string file_name)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (std::size_t i = 0; i < boundaries.size(); ++i)
  {
    const Boundary& b = boundaries[i];

    pcl::PointCloud<pcl::PointXYZRGB> boundary_cloud;
    boundary_cloud.resize(b.cols());

    int dim = sizeof(pcl::PointXYZ) / sizeof(float);
    int stride = sizeof(pcl::PointXYZRGB) / sizeof(float);
    boundary_cloud.getMatrixXfMap(dim, stride, 0) = b;

    int current_point = 0;

    for (auto& pt : boundary_cloud.points)
    {
      float shade = ceil(256.0/boundary_cloud.size() * current_point);
      pt._PointXYZRGB::r = shade;
      pt._PointXYZRGB::g = shade;
      pt._PointXYZRGB::b = shade;

      current_point += 1;
    }
  }

//    cloud->operator+=(boundary_cloud);

//    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
//    viewer.setBackgroundColor(0.0, 0.0, 0.0);
//    viewer.addPointCloud(cloud, "boundary_cloud");

//    for (size_t i = 0; i < cloud -> size() - 1; ++i) {
//      pcl::PointXYZ& point1 = cloud[1];
//      pcl::PointXYZ& point2 = cloud->at(i + 1);
//      viewer.addLine(point1, point2, 1.0, 0.0, 0.0, "line" + std::to_string(i));
//    }
//  }

  if (pcl::io::savePCDFile(file_name + ".pcd", *cloud) < 0)
    throw std::runtime_error("Failed to save boundary cloud");
}

void saveBoundaryClouds(const Boundaries& boundaries)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // saving cloud with clusters colored
  for (std::size_t i = 0; i < boundaries.size(); ++i)
  {
    const Boundary& b = boundaries[i];

    pcl::PointCloud<pcl::PointXYZRGB> boundary_cloud;
    boundary_cloud.resize(b.cols());

    int dim = sizeof(pcl::PointXYZ) / sizeof(float);
    int stride = sizeof(pcl::PointXYZRGB) / sizeof(float);
    boundary_cloud.getMatrixXfMap(dim, stride, 0) = b;

    for (auto& pt : boundary_cloud.points)
    {
      float v = (float(i + 1) / float(boundaries.size())) * 255;
      pt._PointXYZRGB::r = v;
      pt._PointXYZRGB::g = v;
      pt._PointXYZRGB::b = v;
    }

    cloud->operator+=(boundary_cloud);

      if (pcl::io::savePCDFile("boundary_cloud_clusters.pcd", *cloud) < 0)
          throw std::runtime_error("Failed to save boundary cloud");
  }

  if (pcl::io::savePCDFile("boundary_cloud_clusters.pcd", *cloud) < 0)
    throw std::runtime_error("Failed to save boundary cloud");
}

void saveBoundaryCloud(const Boundary& boundary, int color)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // saving cloud
  const Boundary& b = boundary;

  pcl::PointCloud<pcl::PointXYZRGB> boundary_cloud;
  boundary_cloud.resize(b.cols());

  int dim = sizeof(pcl::PointXYZ) / sizeof(float);
  int stride = sizeof(pcl::PointXYZRGB) / sizeof(float);
  boundary_cloud.getMatrixXfMap(dim, stride, 0) = b;

  for (auto& pt : boundary_cloud.points)
  {
    pt._PointXYZRGB::r = color;
    pt._PointXYZRGB::g = color;
    pt._PointXYZRGB::b = color;
  }

  cloud->operator+=(boundary_cloud);

  if (pcl::io::savePCDFile(std::to_string(color) + "_boundary_cloud.pcd", *cloud) < 0)
    throw std::runtime_error("Failed to save boundary cloud");
}

void savePairedBoundaryClouds(const std::vector<std::pair<Boundary, Boundary>>& ordered_boundary_pairs)
{
  for(std::size_t i = 0; i < ordered_boundary_pairs.size(); ++i)
  {
    const auto & pair = ordered_boundary_pairs[i];
    pcl::PointCloud<pcl::PointXYZRGB> pair_cloud, outer_cloud, inner_cloud;

    outer_cloud.resize(pair.first.cols());
    inner_cloud.resize(pair.second.cols());

//    outer_cloud.getMatrixXfMap() = pair.first;
//    inner_cloud.getMatrixXfMap() = pair.second;


    for (Eigen::Index c = 0; c < pair.first.cols(); ++c)
    {
      outer_cloud.points[c].x = pair.first(0, c);
      outer_cloud.points[c].y = pair.first(1, c);
      outer_cloud.points[c].z = pair.first(2, c);
    }

    for (Eigen::Index c = 0; c < pair.second.cols(); ++c)
    {
      inner_cloud.points[c].x = pair.second(0, c);
      inner_cloud.points[c].y = pair.second(1, c);
      inner_cloud.points[c].z = pair.second(2, c);
    }

    pair_cloud += outer_cloud;
    pair_cloud += inner_cloud;

    // coloring each pair a random color
    int r = rand() % 255;
    int g = rand() % 255;
    int b = rand() % 255;
    for (auto& pt : pair_cloud.points)
    {
      pt.r = r;
      pt.g = g;
      pt.b = b;
    }

    if(pcl::io::savePCDFile("pair_cloud_" + std::to_string(i) + ".pcd", pair_cloud) < 0)
      throw std::runtime_error("Failed to save pair cloud " + std::to_string(i));
  }
}

int main(int argc, char** argv)
{
  try
  {
    if(argc != 10)
      throw std::runtime_error("Incorrect number of arguments");

    // Load a mesh from file
    pcl::PolygonMesh mesh;
    if(pcl::io::loadPolygonFile(argv[1], mesh) < 0)
      throw std::runtime_error("Failed to load mesh file from '" + std::string(argv[1]) + "'");

    ColorClusteringParameters p;
    p.setHLimits(std::stof(argv[2]), std::stof(argv[3]));
    p.setSLimits(std::stof(argv[4]), std::stof(argv[5]));
    p.setVLimits(std::stof(argv[6]), std::stof(argv[7]));
    p.cluster_tolerance = std::stod(argv[8]);
    p.min_cluster_size = 15;
    p.leaf_size = std::stof(argv[9]);

    ClosedBoundaryDetector detector(p);
    Boundaries boundaries = detector.detectAllBoundaries(mesh);

    if(boundaries.empty())
      throw std::runtime_error("No boundaries detected");
    std::cout << "Detected " << boundaries.size() << " boundaries" << std::endl;

    saveBoundaryClouds(boundaries);
    saveBoundaryCloudsOrderVisible(boundaries, "boundary_cloud_unordered");

    // Order the points within the boundaries
    //BoundaryOrderGenerator::ConstPtr order_gen = std::make_shared<KdTreeBoundaryOrderGenerator>(10);
    BoundaryOrderGenerator::ConstPtr order_gen = std::make_shared<TSPBoundaryOrderGenerator>();
    //BoundaryOrderGenerator::ConstPtr order_gen = std::make_shared<CrustBoundaryOrderGenerator>();

    Boundaries ordered_boundaries;
    ordered_boundaries.reserve(boundaries.size());
    std::transform(boundaries.begin(), boundaries.end(), std::back_inserter(ordered_boundaries), std::bind(&BoundaryOrderGenerator::order, order_gen, std::placeholders::_1));

    // Saving ordered boundaries for debugging
    saveBoundaryCloudsOrderVisible(ordered_boundaries, "boundary_cloud_ordered");

    // Pair the boundaries
    std::vector<std::pair<Boundary, Boundary>> ordered_boundary_pairs = detector.pairBoundaries(ordered_boundaries);
    std::cout << "Detected " << ordered_boundary_pairs.size() << " pairs" << std::endl;

    savePairedBoundaryClouds(ordered_boundary_pairs);

    saveBoundaryCloud(ordered_boundary_pairs[0].first, 0);
    saveBoundaryCloud(ordered_boundary_pairs[0].second, 255);

    ClosedBoundaryExtractor extractor;
    // Testing inside extractor on inside boundary of pair
    auto insideMesh = extractor.extractInside(mesh, ordered_boundary_pairs[0].first);

//    if(pcl::io::savePCDFile("insideCloud.pcd", insideCloud) < 0)
//      throw std::runtime_error("Failed to save insideCloud");
    if(!pcl::io::savePolygonFile("insideMesh.ply", insideMesh))
      throw std::runtime_error("Failed to save insideMesh");

    // Testing outside extractor on outside boundary of pair
    auto outsideMesh = extractor.extractOutside(mesh, ordered_boundary_pairs[0].second);
    //auto outsideMesh = extractor.extractOutside(mesh, ordered_boundary_pairs[0].first);

//    if(pcl::io::savePCDFile("outsideCloud.pcd", outsideCloud) < 0)
//      throw std::runtime_error("Failed to save outsideCloud");

    if(!pcl::io::savePolygonFile("outsideMesh.ply", outsideMesh))
      throw std::runtime_error("Failed to save outsideMesh");

    // Testing between extractor on pair
    auto betweenMesh = extractor.extractBetween(mesh, ordered_boundary_pairs[0].first, ordered_boundary_pairs[0].second);
    //auto betweenMesh = extractor.extractBetween(mesh, ordered_boundary_pairs[0].first, ordered_boundary_pairs[0].second);

//    if(pcl::io::savePCDFile("betweenCloud.pcd", betweenCloud) < 0)
//      throw std::runtime_error("Failed to save betweenCloud");
    if(!pcl::io::savePolygonFile("betweenMesh.ply", betweenMesh))
      throw std::runtime_error("Failed to save betweenMesh");

    /*
    // INTERPOLATION TESTING - WORK IN PROGRESS
    pcl::PolygonMesh interpolatedMesh;

    const pcl::PointCloud<pcl::PointNormal>::Ptr betweenCloudPtr = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::fromPCLPointCloud2(betweenMesh.cloud, *betweenCloudPtr);

    const pcl::PointCloud<pcl::PointNormal>::Ptr betweenCloudSmoothedPtr = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    pcl::MovingLeastSquares<pcl::PointNormal,pcl::PointNormal> mls;
    mls.setInputCloud(betweenCloudPtr);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
    mls.setSearchRadius(0.01);
    mls.process(*betweenCloudSmoothedPtr);

    // Copy normals from smoothed cloud to betweenCloud
    for (std::size_t i = 0; i < betweenCloudSmoothedPtr->size(); ++i)
    {
      betweenCloudPtr->points[i].normal_x = betweenCloudSmoothedPtr->points[i].normal_x;
      betweenCloudPtr->points[i].normal_y = betweenCloudSmoothedPtr->points[i].normal_y;
      betweenCloudPtr->points[i].normal_z = betweenCloudSmoothedPtr->points[i].normal_z;
    }

    if(pcl::io::savePLYFile("betweenCloudPtr.ply", *betweenCloudPtr) < 0)
      throw std::runtime_error("Failed to save betweenCloudPtr");

    PoissonRegionInterpolator interpolator;
    interpolatedMesh = interpolator.interpolate(betweenCloudSmoothedPtr);

    if(!pcl::io::savePolygonFile("interpolatedCloud.ply", interpolatedMesh))
      throw std::runtime_error("Failed to save interpolatedCloud");

    // Convert the mesh to a point cloud
//    pcl::PointCloud<pcl::PointXYZ> interpolatedCloudNoCut;
//    pcl::fromPCLPointCloud2(interpolatedMesh.cloud, interpolatedCloudNoCut);

//    if(pcl::io::savePCDFile("interpolatedCloudNoCut.pcd", interpolatedCloudNoCut) < 0)
//      throw std::runtime_error("Failed to save interpolatedCloudNoCut");

    auto interpolatedCloud = extractor.extractInside(interpolatedMesh, ordered_boundary_pairs[0].second);

    if(!pcl::io::savePolygonFile("interpolatedCloud.ply", interpolatedCloud))
      throw std::runtime_error("Failed to save interpolatedCloud");

//    #include <pcl/surface/mls.h>
//    #include <pcl/PCLPointCloud2.h>
//    #include <pcl/search/kdtree.h>
//    #include <pcl/features/normal_3d.h>

//    auto combinedCloud  = betweenCloud;
//    combinedCloud += interpolatedCloud;

//    const pcl::PointCloud<pcl::PointXYZ>::Ptr combinedCloudPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(combinedCloud);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//    mls.setInputCloud(combinedCloudPtr);
//    mls.setPolynomialOrder(2);
//    mls.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
//    mls.setSearchRadius(0.04);
//    mls.process(*cloud_out);

//    if(pcl::io::savePCDFile("MLSCloud.pcd", *cloud_out) < 0)
//      throw std::runtime_error("Failed to save new_cloud");

     */
    return 0;
  }
  catch(const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }
}
