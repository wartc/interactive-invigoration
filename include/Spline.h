#ifndef __SPLINE_H__
#define __SPLINE_H__

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

constexpr float SPLINE_ALPHA = 0.5f;  // centripetal catmull-rom
constexpr float SPLINE_TENSION = 0.4f;

class Spline {
 private:
  std::vector<glm::vec3> points{};
  std::vector<unsigned int> indices{};
  unsigned int vbo{}, ebo{}, vao{};

 public:
  Spline() {}

  explicit Spline(const std::vector<glm::vec3>& points_) : points{points_} {
    smoothSpline();
    init();
  }

  explicit Spline(std::vector<glm::vec3>&& points_) noexcept : points(std::move(points_)) {
    smoothSpline();
    init();
  }

  void draw() {
    glBindVertexArray(vao);
    glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }

  void setPoints(const std::vector<glm::vec3>& points_) {
    points = points_;
    smoothSpline();
    init();
  }

 private:
  void init() {
    indices.clear();

    for (int i = 0; i < points.size() - 1; ++i) {
      indices.push_back(i);
      indices.push_back(i + 1);
    }

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * points.size(), points.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), indices.data(),
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  void smoothSpline() {
    int nPoints = points.size();
    assert(nPoints > 1);

    const float step = 0.1f;
    std::vector<glm::vec3> interpolated;
    glm::vec3 start = 2.0f * points[0] - points[1];
    glm::vec3 end = 2.0f * points[nPoints - 1] - points[nPoints - 2];

    for (int i = 0; i < nPoints - 1; ++i) {
      glm::vec3 p0 = i >= 1 ? points[i - 1] : start;
      glm::vec3 p4 = i + 2 < nPoints ? points[i + 2] : end;

      // interpolate in the interval [ points[i], points[i + 1] )
      for (float t = 0.0f; t < 1.0f; t += step) {
        interpolated.push_back(catmullRom(p0, points[i], points[i + 1], p4, t));
      }
    }

    // include the last point explicitly
    interpolated.push_back(points[nPoints - 1]);

    points = interpolated;
  }

  glm::vec3 catmullRom(
      const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, float t
  ) {
    float t01 = pow(distance(p0, p1), SPLINE_ALPHA);
    float t12 = pow(distance(p1, p2), SPLINE_ALPHA);
    float t23 = pow(distance(p2, p3), SPLINE_ALPHA);

    glm::vec3 m1 =
        (1.0f - SPLINE_TENSION) * (p2 - p1 + t12 * ((p1 - p0) / t01 - (p2 - p0) / (t01 + t12)));
    glm::vec3 m2 =
        (1.0f - SPLINE_TENSION) * (p2 - p1 + t12 * ((p3 - p2) / t23 - (p3 - p1) / (t12 + t23)));

    glm::vec3 a = 2.0f * (p1 - p2) + m1 + m2;
    glm::vec3 b = -3.0f * (p1 - p2) - m1 - m1 - m2;

    return a * (t * t * t) + b * (t * t) + m1 * t + p1;
  }
};

#endif
