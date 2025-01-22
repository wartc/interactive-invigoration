#ifndef __SPLINE_H__
#define __SPLINE_H__

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "Shader.h"

class Spline {
 private:
  std::vector<glm::vec3> points{};
  std::vector<unsigned int> indices{};
  unsigned int VBO{}, EBO{}, VAO{};

 public:
  Spline(const std::vector<glm::vec3>& points_) : points{points_} { init(); }

  void draw(Shader& shader) {
    shader.setVec4("splineColor", {0.70f, 0.98f, 0.64f, 1.0f});
    glBindVertexArray(VAO);
    glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }

 private:
  void init() {
    for (int i = 0; i < points.size() - 1; ++i) {
      indices.push_back(i);
      indices.push_back(i + 1);
    }

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * points.size(), points.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), indices.data(),
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }
};

#endif
