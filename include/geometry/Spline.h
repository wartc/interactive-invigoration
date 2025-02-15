#ifndef __SPLINE_H__
#define __SPLINE_H__

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

constexpr int NUM_INTERPOLATED_POINTS = 10;  // 10 points between each other when interpolating
constexpr float SPLINE_ALPHA = 0.5f;         // centripetal catmull-rom
constexpr float SPLINE_TENSION = 0.6f;

class Spline {
 private:
  // vertex data for rendering
  std::vector<unsigned int> indices{};
  unsigned int vbo{}, ebo{}, vao{};

 public:
  std::vector<glm::vec3> points{};

  Spline() {}

  explicit Spline(const std::vector<glm::vec3>& points_) : points{points_} {}

  explicit Spline(std::vector<glm::vec3>&& points_) noexcept : points(std::move(points_)) {}

  void render() const;

  // interpolate and update the `points` vector
  void smoothenSpline() { points = interpolate(points); }

  // interpolate the points given and return the newly interpolated vertices
  static std::vector<glm::vec3> interpolate(const std::vector<glm::vec3>& points_);

  void initializeBuffers();

 private:
  static glm::vec3 catmullRom(
      const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, float t
  );
};

#endif
