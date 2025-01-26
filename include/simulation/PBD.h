#ifndef __PBD_H__
#define __PBD_H__

#include <set>
#include <vector>

#include <glm/glm.hpp>

#include "simulation/PBDConstraint.h"

constexpr float GAMMA_ATTRACTION = 100.0f;
constexpr int SOLVER_INTERATIONS = 100;

// position based dynamics class
// no masses are considered (w = m = 1)
class PBD {
 private:
  std::vector<glm::vec3> x{};
  std::vector<glm::vec3> v{};
  std::vector<glm::vec3> p{};

  std::vector<glm::vec3> attractors{};
  CircularProfileConstraint boundaryConstraint;
  CollisionConstraint collisionConstraint;

  float dt{};
  float kdamping{};
  float particleRadius{};

 public:
  PBD(const std::vector<glm::vec3>& pos, const std::vector<glm::vec3>& attrs, float dampingFactor,
      float _dt, float particleRadius, glm::vec3 profileCenter, float profileRadius)
      : attractors{attrs},
        kdamping{dampingFactor},
        dt{_dt},
        particleRadius{particleRadius},
        collisionConstraint(PBDConstraint<2>::INEQUALITY, 1.0f, {}, particleRadius),
        boundaryConstraint(PBDConstraint<1>::INEQUALITY, 1.0f, {}, profileRadius, profileCenter) {
    setPoints(pos);
  }

  std::vector<glm::vec3> execute(int iterations);

  void setPoints(const std::vector<glm::vec3>& points) {
    x = points;
    v.resize(x.size());
    p.resize(x.size());
  }

 private:
  void simulate();
  glm::vec3 computeExternalForces(int idx);

  void solve(const std::set<std::pair<int, int>>& mcoll);
};

#endif
