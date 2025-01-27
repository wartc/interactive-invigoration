#ifndef __GEOMETRY_UTILS_H__
#define __GEOMETRY_UTILS_H__

#include <vector>

#include <glm/glm.hpp>

namespace util {

std::vector<glm::uvec3> delaunay(const std::vector<glm::vec2>& vertices);

std::vector<glm::uvec3> delaunay(const std::vector<glm::vec2>& vertices);

};  // namespace util

#endif
