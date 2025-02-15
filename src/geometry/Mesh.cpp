#include "geometry/Mesh.h"

#include <glad/glad.h>

void Mesh::render() const {
  glBindVertexArray(vao);
  glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}

void Mesh::init() {
  std::vector<GLuint> flatIndices;
  for (const auto& tri : indices) {
    flatIndices.push_back(tri.x);
    flatIndices.push_back(tri.y);
    flatIndices.push_back(tri.z);
  }

  bool hasNormals = normals.size() == vertices.size();
  std::vector<glm::vec3> interleavedVertexData;
  for (int i = 0; i < vertices.size(); ++i) {
    interleavedVertexData.push_back(vertices[i]);

    if (hasNormals) interleavedVertexData.push_back(normals[i]);
  }

  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);
  glGenBuffers(1, &ebo);

  glBindVertexArray(vao);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(
      GL_ARRAY_BUFFER, sizeof(glm::vec3) * interleavedVertexData.size(),
      interleavedVertexData.data(), GL_STATIC_DRAW
  );

  auto stride = hasNormals ? 2 * sizeof(glm::vec3) : sizeof(glm::vec3);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
  glEnableVertexAttribArray(0);

  if (hasNormals) {
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
    glEnableVertexAttribArray(1);
  }

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(
      GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * flatIndices.size(), flatIndices.data(),
      GL_STATIC_DRAW
  );

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
