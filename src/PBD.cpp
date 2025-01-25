#include "PBD.h"

#include <iostream>
#include <vector>

#include <glm/geometric.hpp>

#include "PBDConstraint.h"

std::vector<glm::vec3> PBD::execute(int iterations) {
  v.resize(x.size());
  v.clear();

  for (int i = 0; i < iterations; ++i) {
    simulate();
  }

  return x;
}

void PBD::simulate() {
  std::vector<glm::vec3> p(x.size());

  for (int i = 0; i < x.size(); ++i) {
    computeExternalForces(i);
    v[i] += dt * 50;
  }

  dampVelocities();

  for (int i = 0; i < x.size(); ++i) {
    p[i] = x[i] + dt * v[i];
    std::cout << "p[" << i << "] = (" << p[i].x << ", " << p[i].y << ", " << p[i].z << ")\n";
  }

  std::vector<CollisionConstraint> mcoll;
  for (int i = 0; i < p.size(); ++i) {
    for (int j = 0; j < p.size(); ++j) {
      if (i == j) continue;

      glm::vec3 u = p[i] - p[j];
      if (glm::dot(u, u) < particleRadius * particleRadius) {
        mcoll.push_back(CollisionConstraint{
            PBDConstraint<2>::INEQUALITY, 1.0f, {p[i], p[j]},
              particleRadius
        });
      }
    }
  }

  for (int i = 0; i < SOLVER_INTERATIONS; ++i) {
    solve(mcoll);
  }

  for (int i = 0; i < x.size(); ++i) {
    v[i] = (p[i] - x[i]) / dt;
    x[i] = p[i];
  }

  // no need for a more sophisticated velocity update...
}

glm::vec3 PBD::computeExternalForces(int idx) {
  glm::vec3 force{0.0f};

  // attractors: inversely proportional to distance
  for (auto& attractorPos : attractors) {
    glm::vec3 u = attractorPos - x[idx];
    float len2 = glm::dot(u, u);
    std::cout << len2 << std::endl;

    if (len2 == 0) return {0.0f, 0.0f, 0.0f};

    force += K_ATTRACTION * (u / len2);
  }

  return force;
}

void PBD::dampVelocities() {}

void PBD::solve(const std::vector<CollisionConstraint>& mcoll) {}

