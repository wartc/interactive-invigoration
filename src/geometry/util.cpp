#include "geometry/util.h"

#include <algorithm>
#include <cassert>
#include <vector>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <glm/glm.hpp>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Delaunay = CGAL::Delaunay_triangulation_2<K>;
using Point = K::Point_2;

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
