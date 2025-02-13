#include "geometry/util.h"

#include <algorithm>
#include <cassert>
#include <vector>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Plane_3.h>
#include <glm/glm.hpp>

using DelaunayK = CGAL::Exact_predicates_inexact_constructions_kernel;
using Delaunay = CGAL::Delaunay_triangulation_2<DelaunayK>;
using Point = DelaunayK::Point_2;

using CartesianK = CGAL::Simple_cartesian<double>;
using CGALPoint = CartesianK::Point_3;
using CGALPlane = CartesianK::Plane_3;

std::pair<glm::vec3, glm::vec3> util::computeLeastSquaresFittingPlane(
    const std::vector<glm::vec3>& vertices
) {
  if (vertices.size() < 3) {
    throw std::runtime_error("At least 3 points required for plane fitting.");
  }

  // convert glm::vec3 points to CGAL points
  std::vector<CGALPoint> cgal_points;
  for (const auto& v : vertices) {
    cgal_points.emplace_back(v.x, v.y, v.z);
  }

  // best-fit plane
  CGALPlane plane;
  CGAL::linear_least_squares_fitting_3(
      cgal_points.begin(), cgal_points.end(), plane, CGAL::Dimension_tag<0>()
  );

  CartesianK::Point_3 origin = plane.projection(CGAL::ORIGIN);
  CartesianK::Vector_3 normal = plane.orthogonal_vector();
  normal = normal / CGAL::sqrt(normal.squared_length());

  return {
      glm::vec3(origin.x(), origin.y(), origin.z()), glm::vec3(normal.x(), normal.y(), normal.z())
  };
}

std::vector<glm::uvec3> util::delaunay(const std::vector<glm::vec2>& vertices) {
  std::vector<glm::uvec3> triangles;

  assert(vertices.size() > 3);

  // convert glm::vec2 vertices to CGAL points
  std::vector<Point> points;
  for (const auto& v : vertices) {
    points.push_back(Point(v.x, v.y));
  }

  // delaunay triangulation
  Delaunay dt;
  dt.insert(points.begin(), points.end());

  // extract triangles from the triangulation
  for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
    // get the vertex indices of the triangle
    auto v1 = std::distance(
        points.begin(), std::find(points.begin(), points.end(), face->vertex(0)->point())
    );
    auto v2 = std::distance(
        points.begin(), std::find(points.begin(), points.end(), face->vertex(1)->point())
    );
    auto v3 = std::distance(
        points.begin(), std::find(points.begin(), points.end(), face->vertex(2)->point())
    );

    triangles.push_back(glm::uvec3(v1, v2, v3));
  }

  return triangles;
}
