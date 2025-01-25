#ifndef __PBD_H__
#define __PBD_H__

#include <vector>

#include <glm/glm.hpp>

#include "PBDConstraint.h"

constexpr float K_ATTRACTION = 0.0f;
constexpr int SOLVER_INTERATIONS = 1;

// position based dynamics class
// no masses are considered (w = m = 1)
class PBD {
 private:
  std::vector<glm::vec3> x{};
  std::vector<glm::vec3> v{};

  std::vector<glm::vec3> attractors{};
  CircularProfileConstraint boundaryConstraint;

  float dt{};
  float kdamping{};
  float particleRadius{};

 public:
  PBD(const std::vector<glm::vec3>& pos, const std::vector<glm::vec3>& attrs, float dampingFactor,
      float _dt, float particleRadius, glm::vec3 profileCenter, float profileRadius)
      : x{pos},
        attractors{attrs},
        kdamping{dampingFactor},
        dt{_dt},
        particleRadius{particleRadius},
        boundaryConstraint(
            PBDConstraint<1>::INEQUALITY, 0.0f, {profileCenter}, profileRadius, profileCenter
        ) {}

  std::vector<glm::vec3> execute(int iterations);

 private:
  void simulate();
  glm::vec3 computeExternalForces(int idx);
  void dampVelocities();

  void solve(const std::vector<CollisionConstraint>& mcoll);
};

#endif
