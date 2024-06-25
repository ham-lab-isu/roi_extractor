#include "boundary_extractor/closed_boundary_extractor.h"

#include <algorithm>
#include <pcl/conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

//#include <noether_filtering/subset_extraction/subset_extractor.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <pcl/io/vtk_lib_io.h>

namespace roi_detector
{

ClosedBoundaryExtractor::ClosedBoundaryExtractor()
{
}

pcl::PolygonMesh ClosedBoundaryExtractor::extractInside(const pcl::PolygonMesh& mesh, const Boundary& boundary) const
{
  // Determine which points in mesh are inside boundary
  pcl::PointCloud<pcl::PointXYZ> meshCloud;
  pcl::fromPCLPointCloud2(mesh.cloud, meshCloud);

  // Create polygon for boundary
  auto polygon = createPolygon(boundary);

  // Create vector for inside point indices
  std::vector<int> insideIndices;

  // Loop through all points in mesh
  for (std::size_t i = 0; i < meshCloud.points.size(); ++i)
  {
    const auto& point = meshCloud.points[i];

    // Check if point is inside boundary
    if (pcl::isPointIn2DPolygon(point, polygon))
    {
      // If inside, add index to insideIndices vector
      insideIndices.push_back(static_cast<int>(i));
    }
  }

  // Extract the inside points using the indices
//  pcl::PointCloud<pcl::PointXYZ> insideCloud;
//  pcl::copyPointCloud(meshCloud, insideIndices, insideCloud);
//  return insideCloud;

  std::vector<int> inlier_vertex_indices = insideIndices;
  pcl::PolygonMesh input_mesh = mesh;

  // mark inlying points as true and outlying points as false
  std::vector<bool> whitelist(input_mesh.cloud.width * input_mesh.cloud.height, false);
  for (std::size_t i = 0; i < inlier_vertex_indices.size(); ++i)
  {
    whitelist[static_cast<std::size_t>(inlier_vertex_indices[i])] = true;
  }

  // All points marked 'false' in the whitelist will be removed. If
  // all of the polygon's points are marked 'true', that polygon
  // should be included.
  pcl::PolygonMesh intermediate_mesh;
  intermediate_mesh.cloud = input_mesh.cloud;
  for (std::size_t i = 0; i < input_mesh.polygons.size(); ++i)
  {
    if (whitelist[input_mesh.polygons[i].vertices[0]] &&
        whitelist[input_mesh.polygons[i].vertices[1]] &&
        whitelist[input_mesh.polygons[i].vertices[2]])
    {
      intermediate_mesh.polygons.push_back(input_mesh.polygons[i]);
    }
  }

  // Remove unused points and save the result to the output mesh
  pcl::surface::SimplificationRemoveUnusedVertices simplifier;
  pcl::PolygonMesh output_mesh;
  simplifier.simplify(intermediate_mesh, output_mesh);

  return output_mesh;

  //return noether::extractSubMeshFromInlierVertices(mesh, insideIndices);
}

pcl::PolygonMesh ClosedBoundaryExtractor::extractBetween(const pcl::PolygonMesh& mesh, const Boundary& insideBoundary, const Boundary& outsideBoundary) const
{
  // Determine which points in mesh are between the boundaries
  pcl::PointCloud<pcl::PointXYZ> meshCloud;
  pcl::fromPCLPointCloud2(mesh.cloud, meshCloud);

  // Create polygon for boundaries
  auto insidePolygon = createPolygon(insideBoundary);
  auto outsidePolygon = createPolygon(outsideBoundary);

  // Create vector for inside point indices
  std::vector<int> betweenIndices;

  // Loop through all points in mesh
  for (std::size_t i = 0; i < meshCloud.points.size(); ++i)
  {
    const auto& point = meshCloud.points[i];

    // Check if point is inside boundary
    if(!pcl::isPointIn2DPolygon(point, insidePolygon) && pcl::isPointIn2DPolygon(point, outsidePolygon))
    {
      // If inside, add index to insideIndices vector
      betweenIndices.push_back(static_cast<int>(i));
    }
  }

  // Extract the inside points using the indices
  //  pcl::PointCloud<pcl::PointXYZ> insideCloud;
  //  pcl::copyPointCloud(meshCloud, insideIndices, insideCloud);
  //  return insideCloud;
  std::vector<int> inlier_vertex_indices = betweenIndices;
  pcl::PolygonMesh input_mesh = mesh;

  // mark inlying points as true and outlying points as false
  std::vector<bool> whitelist(input_mesh.cloud.width * input_mesh.cloud.height, false);
  for (std::size_t i = 0; i < inlier_vertex_indices.size(); ++i)
  {
    whitelist[static_cast<std::size_t>(inlier_vertex_indices[i])] = true;
  }

  // All points marked 'false' in the whitelist will be removed. If
  // all of the polygon's points are marked 'true', that polygon
  // should be included.
  pcl::PolygonMesh intermediate_mesh;
  intermediate_mesh.cloud = input_mesh.cloud;
  for (std::size_t i = 0; i < input_mesh.polygons.size(); ++i)
  {
    if (whitelist[input_mesh.polygons[i].vertices[0]] &&
        whitelist[input_mesh.polygons[i].vertices[1]] &&
        whitelist[input_mesh.polygons[i].vertices[2]])
    {
      intermediate_mesh.polygons.push_back(input_mesh.polygons[i]);
    }
  }

  // Remove unused points and save the result to the output mesh
  pcl::surface::SimplificationRemoveUnusedVertices simplifier;
  pcl::PolygonMesh output_mesh;
  simplifier.simplify(intermediate_mesh, output_mesh);

  return output_mesh;

  //return noether::extractSubMeshFromInlierVertices(mesh, betweenIndices);
}

pcl::PolygonMesh ClosedBoundaryExtractor::extractOutside(const pcl::PolygonMesh& mesh, const Boundary& boundary) const
{

  // Determine which points in mesh are outside boundary
  pcl::PointCloud<pcl::PointXYZ> meshCloud;
  pcl::fromPCLPointCloud2(mesh.cloud, meshCloud);

  // Create polygon for boundary
  auto polygon = createPolygon(boundary);

  // Create point cloud for outside points
  pcl::PointCloud<pcl::PointXYZ> outsideCloud;

  // Create vector for inside point indices
  std::vector<int> outsideIndices;

  // Loop through all points in mesh
  for (std::size_t i = 0; i < meshCloud.points.size(); ++i)
  {
    const auto& point = meshCloud.points[i];

    // Check if point is inside boundary
    if (!pcl::isPointIn2DPolygon(point, polygon))
    {
      // If inside, add index to insideIndices vector
      outsideIndices.push_back(static_cast<int>(i));
    }
  }

  // Extract the inside points using the indices
//    pcl::PointCloud<pcl::PointXYZ> outsideCloud;
//    pcl::copyPointCloud(meshCloud, outsideIndices, outsideCloud);
//    return outsideCloud;

//  return outsideCloud;
  std::vector<int> inlier_vertex_indices = outsideIndices;
  pcl::PolygonMesh input_mesh = mesh;

  // mark inlying points as true and outlying points as false
  std::vector<bool> whitelist(input_mesh.cloud.width * input_mesh.cloud.height, false);
  for (std::size_t i = 0; i < inlier_vertex_indices.size(); ++i)
  {
    whitelist[static_cast<std::size_t>(inlier_vertex_indices[i])] = true;
  }

  // All points marked 'false' in the whitelist will be removed. If
  // all of the polygon's points are marked 'true', that polygon
  // should be included.
  pcl::PolygonMesh intermediate_mesh;
  intermediate_mesh.cloud = input_mesh.cloud;
  for (std::size_t i = 0; i < input_mesh.polygons.size(); ++i)
  {
    if (whitelist[input_mesh.polygons[i].vertices[0]] &&
        whitelist[input_mesh.polygons[i].vertices[1]] &&
        whitelist[input_mesh.polygons[i].vertices[2]])
    {
      intermediate_mesh.polygons.push_back(input_mesh.polygons[i]);
    }
  }

  // Remove unused points and save the result to the output mesh
  pcl::surface::SimplificationRemoveUnusedVertices simplifier;
  pcl::PolygonMesh output_mesh;
  simplifier.simplify(intermediate_mesh, output_mesh);

  return output_mesh;

  //return noether::extractSubMeshFromInlierVertices(mesh, outsideIndices);
}

} // namespace snp_isu_tpp
