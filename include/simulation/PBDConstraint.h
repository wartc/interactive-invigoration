#ifndef __PBD_CONSTRAINT_H__
#define __PBD_CONSTRAINT_H__

#include <array>

#include <glm/glm.hpp>

template <std::size_t N>
class PBDConstraint {
 public:
  enum ConstraintType { EQUALITY, INEQUALITY };

 protected:
  ConstraintType type;
  float stiffness;
  std::array<glm::vec3, N> points;

 public:
  PBDConstraint(ConstraintType _type, float _stiffness, const std::array<glm::vec3, N>& _points)
      : type{_type}, stiffness{_stiffness}, points{_points} {}

  // correction terms (delta p) for the points (considering the scaling)
  virtual std::array<glm::vec3, N> computeCorrection() const = 0;

  virtual float evaluate() const = 0;

  void setPoints(const std::array<glm::vec3, N>& _points) { points = _points; }

  bool isSatisfied() const {
    float value = evaluate();

    return type == EQUALITY ? (value == 0) : (value >= 0);
  }
};

class CollisionConstraint : public PBDConstraint<2> {
 private:
  float pointRadius;

 public:
  CollisionConstraint(
      ConstraintType _type, float _stiffness, const std::array<glm::vec3, 2>& _points, float d
  )
      : PBDConstraint<2>(_type, _stiffness, _points), pointRadius{d} {}

  std::array<glm::vec3, 2> computeCorrection() const override {
    glm::vec3 u = points[0] - points[1];
    float len = glm::length(u);
    float k = 1 - std::pow(1 - stiffness, 0.5f);

    return {
        ((len - pointRadius) * (u / len)) * k,  //
        (-(len - pointRadius) * (u / len)) * k  //
    };
  }

  float evaluate() const override { return glm::length(points[0] - points[1]) - (2 * pointRadius); }
};

class CircularProfileConstraint : public PBDConstraint<1> {
 private:
  float radius;
  glm::vec3 center;

 public:
  CircularProfileConstraint(
      ConstraintType _type, float _stiffness, const std::array<glm::vec3, 1>& _points,
      float _radius, const glm::vec3& _center = {0.0f, 0.0f, 0.0f}
  )
      : PBDConstraint<1>(_type, _stiffness, _points), radius{_radius}, center{_center} {}

  std::array<glm::vec3, 1> computeCorrection() const override {
    glm::vec3 u = center - points[0];
    float len = glm::length(u);

    return {((len - radius) * (u / len)) * stiffness};
  }

  float evaluate() const override { return radius - glm::length(center - points[0]); }
};

#endif
