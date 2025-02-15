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
using CGALPoint2 = DelaunayK::Point_2;

using CartesianK = CGAL::Simple_cartesian<double>;
using CGALPoint3 = CartesianK::Point_3;
using CGALPlane = CartesianK::Plane_3;

std::pair<glm::vec3, glm::vec3> util::computeLeastSquaresFittingPlane(
    const std::vector<glm::vec3>& vertices
) {
  if (vertices.size() < 3) {
    throw std::runtime_error("At least 3 points required for plane fitting.");
  }

  // convert glm::vec3 points to CGAL points
  std::vector<CGALPoint3> cgal_points;
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

  assert(vertices.size() >= 3);

  // convert glm::vec2 vertices to CGAL points
  std::vector<CGALPoint2> points;
  for (const auto& v : vertices) {
    points.push_back(CGALPoint2(v.x, v.y));
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

std::vector<int> util::computeBoundaryVertices(
    const std::vector<glm::vec2>& planarCoords, const std::vector<glm::uvec3>& triangles
) {
  std::map<std::pair<int, int>, int> edgeCount;
  std::vector<std::pair<int, int>> boundaryEdges;

  for (const auto& tri : triangles) {
    std::vector<std::pair<int, int>> edges = {
        {tri.x, tri.y},
        {tri.y, tri.z},
        {tri.z, tri.x}
    };
    for (const auto& edge : edges) {
      auto sortedEdge =
          std::make_pair(std::min(edge.first, edge.second), std::max(edge.first, edge.second));
      edgeCount[sortedEdge]++;
    }
  }

  // group unsorted boundary edges
  for (const auto& tri : triangles) {
    std::vector<std::pair<int, int>> edges = {
        {tri.x, tri.y},
        {tri.y, tri.z},
        {tri.z, tri.x}
    };
    for (const auto& edge : edges) {
      auto sortedEdge =
          std::make_pair(std::min(edge.first, edge.second), std::max(edge.first, edge.second));
      if (edgeCount[sortedEdge] == 1) {
        boundaryEdges.push_back(edge);  // store original edge direction
      }
    }
  }

  // reconstruct the boundary loop
  if (boundaryEdges.empty()) return {};

  std::vector<int> boundaryLoop;
  int currentVertex = boundaryEdges[0].first;
  int nextVertex = boundaryEdges[0].second;
  boundaryLoop.push_back(currentVertex);
  boundaryLoop.push_back(nextVertex);

  boundaryEdges.erase(boundaryEdges.begin());

  while (!boundaryEdges.empty() && nextVertex != boundaryLoop[0]) {
    for (size_t i = 0; i < boundaryEdges.size(); ++i) {
      if (boundaryEdges[i].first == nextVertex) {
        currentVertex = boundaryEdges[i].first;
        nextVertex = boundaryEdges[i].second;

        boundaryLoop.push_back(nextVertex);
        boundaryEdges.erase(boundaryEdges.begin() + i);
        break;
      } else if (boundaryEdges[i].second == nextVertex) {
        currentVertex = boundaryEdges[i].second;
        nextVertex = boundaryEdges[i].first;

        boundaryLoop.push_back(nextVertex);
        boundaryEdges.erase(boundaryEdges.begin() + i);
        break;
      }
    }
  }

  // CCW order using the polygon area sign
  float area = 0.0f;
  for (size_t i = 0; i < boundaryLoop.size(); ++i) {
    const glm::vec2& p1 = planarCoords[boundaryLoop[i]];
    const glm::vec2& p2 = planarCoords[boundaryLoop[(i + 1) % boundaryLoop.size()]];
    area += (p2.x - p1.x) * (p2.y + p1.y);
  }

  if (area > 0) {  // CW if area is positive
    std::reverse(boundaryLoop.begin(), boundaryLoop.end());
  }

  return boundaryLoop;
}
