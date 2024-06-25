#include "boundary_order_generator/crust_boundary_order_generator.h"

#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/segment_data.hpp>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace bp = boost::polygon;
using Point = bp::point_data<int>;

namespace roi_detector
{
std::vector<Point> toBoost(const Boundary& boundary)
{
  // Project to 2D

  std::vector<Point> points;
  points.reserve(boundary.cols());
  for (Eigen::Index c = 0; c < boundary.cols(); ++c)
  {
    points.push_back(Point(boundary.col(c).x() * 1e6, boundary.col(c).y() * 1e6));
  }

  return points;
}

std::vector<std::vector<std::size_t>> triangulate(const bp::voronoi_diagram<double>& vd)
{
  std::vector<std::vector<std::size_t>> triangles;

  // https://stackoverflow.com/questions/34342038/how-to-triangulate-polygons-in-boost
  for (const auto& vertex : vd.vertices())
  {
    std::vector<std::size_t> triangle;
    auto edge = vertex.incident_edge();
    do
    {
      auto cell = edge->cell();
      assert(cell->contains_point());

      triangle.push_back(cell->source_index());
      if (triangle.size() == 3)
      {
        // Add the triangle to the list
        triangles.push_back(triangle);

        // Remove the 1st vertex of the triangle, to allow for further triangle creation in the case that this Voronoi
        // vertex is degenerate and has more than 3 neighbors
        triangle.erase(triangle.begin() + 1);
      }

      edge = edge->rot_next();
    } while (edge != vertex.incident_edge());
  }

  return triangles;
}

std::map<std::size_t, std::set<std::size_t>>
trianglesToAdjacencyList(const std::vector<std::vector<std::size_t>>& triangles)
{
  std::map<std::size_t, std::set<std::size_t>> adjacency_list;
  for (const std::vector<std::size_t>& tri : triangles)
  {
    adjacency_list[tri[0]].insert(tri[1]);
    adjacency_list[tri[1]].insert(tri[0]);

    adjacency_list[tri[1]].insert(tri[2]);
    adjacency_list[tri[2]].insert(tri[1]);

    adjacency_list[tri[2]].insert(tri[0]);
    adjacency_list[tri[0]].insert(tri[2]);
  }
  return adjacency_list;
}

pcl::Vertices toPCL(const std::vector<std::size_t>& poly)
{
  pcl::Vertices v;
  v.vertices.reserve(poly.size());
  std::transform(poly.begin(), poly.end(), std::back_inserter(v.vertices),
                 [](std::size_t i) { return static_cast<uint32_t>(i); });
  return v;
}

pcl::PolygonMesh toMesh(const std::vector<Point>& points, const std::vector<std::vector<std::size_t>>& triangles)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  std::transform(points.begin(), points.end(), std::back_inserter(cloud), [](const Point& pt) {
    return pcl::PointXYZ(static_cast<float>(pt.x()) / 1.0e6, static_cast<float>(pt.y()) / 1.0e6, 0.0);
  });

  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);

  mesh.polygons.reserve(triangles.size());
  std::transform(triangles.begin(), triangles.end(), std::back_inserter(mesh.polygons), &toPCL);

  return mesh;
}

Boundary CrustBoundaryOrderGenerator::order(const Boundary& boundary) const
{
  // Convert boundary to boost::polygon
  std::vector<Point> points = toBoost(boundary);

  // Get the Voronoi vertices of the input set
  bp::voronoi_diagram<double>::vertex_container_type voronoi_points;
  {
    bp::voronoi_diagram<double> vd;
    bp::construct_voronoi(points.begin(), points.end(), &vd);
    voronoi_points = vd.vertices();

    // Save the triangulation for debug
    pcl::io::savePolygonFile("delaunay_mesh.ply", toMesh(points, triangulate(vd)));
  }

  // Create a union of the Voronoi diagram vertices and the input points
  std::vector<Point> union_set;
  union_set.reserve(points.size() + voronoi_points.size());
  union_set.insert(union_set.end(), points.begin(), points.end());
  for (const auto& v : voronoi_points)
    union_set.push_back(Point(v.x(), v.y()));

  // Delaunay triangulation of the vertex union
  bp::voronoi_diagram<double> vd;
  bp::construct_voronoi(union_set.begin(), union_set.end(), &vd);
  std::vector<std::vector<std::size_t>> triangles = triangulate(vd);

  // Convert triangles to adjacency list
  const std::map<std::size_t, std::set<std::size_t>> adjacency_list = trianglesToAdjacencyList(triangles);

  // An edge of the triangulation belongs to the crust curve if both endpoints belong to the input set of points
  // Walk through the adjacency list for vertices that both belong to the original set
  std::vector<int> order;
  order.reserve(boundary.cols());

  int idx = 0;
  order.push_back(idx);
  bool found = true;
  while (found)
  {
    assert(idx < points.size());
    found = false;

    for (auto endpoint = adjacency_list.at(idx).begin(); endpoint != adjacency_list.at(idx).end(); ++endpoint)
    {
      // Check if the endpoint is in the original set of points
      // Its index should be less than the size of points, since `points` make up the first block of `union_set`
      if (*endpoint < points.size())
      {
        // Check if endpoint is not already in the order list
        if (std::find(order.begin(), order.end(), *endpoint) == order.end())
        {
          order.push_back(*endpoint);

          // Update the index and the condition flag
          found = true;
          idx = *endpoint;

          // Break out of the for-loop
          break;
        }
      }
    }
  }

  // Check that the order includes most of the points
  double threshold = 0.8;
  if (order.size() < threshold * boundary.cols())
  {
    std::stringstream ss;
    ss << "Ordered boundary contains < " << std::setprecision(2) << threshold * 100.0 << "\\% of the input points ("
       << order.size() << "/" << boundary.cols() << ")";
    throw std::runtime_error(ss.str());
  }

  // Reconstruct the ordered boundary
  Boundary output(4, order.size());
  for (Eigen::Index i = 0; i < order.size(); ++i)
    output.col(i) = boundary.col(order[i]);

  return output;
}

} // namespace snp_isu_tpp
