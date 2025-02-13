#ifndef __GEOMETRY_UTILS_H__
#define __GEOMETRY_UTILS_H__

#include <vector>

#include <glm/glm.hpp>

namespace util {

// returns the origin and normalized normal vector of the plane
std::pair<glm::vec3, glm::vec3> computeLeastSquaresFittingPlane(
    const std::vector<glm::vec3>& vertices
);

std::vector<glm::uvec3> delaunay(const std::vector<glm::vec2>& vertices);

};  // namespace util

#endif
