#ifndef __MESH_H__
#define __MESH_H__

#include <vector>

#include <glm/glm.hpp>

class Mesh {
 private:
  unsigned int vao, vbo, ebo;
  std::vector<glm::vec3> vertices;
  std::vector<glm::uvec3> indices;  // triangle indices

 public:
  Mesh(const std::vector<glm::vec3>& _vertices, const std::vector<glm::uvec3>& _indices)
      : vertices{_vertices}, indices{_indices} {
    init();
  }

  Mesh(std::vector<glm::vec3>&& _vertices, std::vector<glm::uvec3>&& _indices)
      : vertices{std::move(_vertices)}, indices{std::move(_indices)} {
    init();
  }

  void render();

 private:
  void init();
};

#endif
