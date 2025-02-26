#ifndef __MESH_H__
#define __MESH_H__

#include <vector>

#include <glm/glm.hpp>

class Mesh {
 private:
  unsigned int vao, vbo, ebo;
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::uvec3> indices;  // triangle indices

 public:
  Mesh(
      const std::vector<glm::vec3>& _vertices, const std::vector<glm::uvec3>& _indices,
      const std::vector<glm::vec3>& _normals = {}
  )
      : vertices{_vertices}, normals{_normals}, indices{_indices} {
    init();
  }

  Mesh(
      std::vector<glm::vec3>&& _vertices, std::vector<glm::uvec3>&& _indices,
      std::vector<glm::vec3>&& _normals = {}
  )
      : vertices{std::move(_vertices)}, normals{std::move(_normals)}, indices{std::move(_indices)} {
    init();
  }

  void render() const;

 private:
  void init();
};

#endif
