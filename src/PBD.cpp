#include "PBD.h"

#include <algorithm>
#include <vector>

#include <glm/geometric.hpp>

#include "PBDConstraint.h"

std::vector<glm::vec3> PBD::execute(int iterations) {
  std::fill(v.begin(), v.end(), glm::vec3{0.0f, 0.0f, 0.0f});
  std::fill(p.begin(), p.end(), glm::vec3{0.0f, 0.0f, 0.0f});

  for (int i = 0; i < iterations; ++i) {
    simulate();
  }

  return x;
}

void PBD::simulate() {
  for (int i = 0; i < x.size(); ++i) {
    v[i] += computeExternalForces(i);
  }

  // optionally damp velocities, according to PBD paper (ex.: to maintain rigid body constraints)
  // not that necessary with the parameters set in this implementation
  // dampVelocities();

  for (int i = 0; i < x.size(); ++i) {
    p[i] = x[i] + dt * v[i];
  }

  std::set<std::pair<int, int>> mcoll;
  for (int i = 0; i < p.size() - 1; ++i) {
    for (int j = i + 1; j < p.size(); ++j) {
      glm::vec3 u = p[i] - p[j];
      if (glm::dot(u, u) < particleRadius * particleRadius) {
        mcoll.insert(std::make_pair(i, j));
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
  // velocityUpdate();
}

glm::vec3 PBD::computeExternalForces(int idx) {
  glm::vec3 force{0.0f};

  // attractors: linearly proportional to distance
  for (auto& attractorPos : attractors) {
    force += GAMMA_ATTRACTION * (attractorPos - x[idx]);
  }

  return force;
}

void PBD::solve(const std::set<std::pair<int, int>>& mcoll) {
  // constraint to not let strands leave the branch profile
  for (int i = 0; i < p.size(); ++i) {
    boundaryConstraint.setPoints({p[i]});
    if (!boundaryConstraint.isSatisfied()) {
      p[i] += boundaryConstraint.computeCorrection()[0];
    }
  }

  // collision constraints
  for (auto& [i, j] : mcoll) {
    collisionConstraint.setPoints({p[i], p[j]});
    if (!collisionConstraint.isSatisfied()) {
      auto correction = collisionConstraint.computeCorrection();
      p[i] += correction[0];
      p[j] += correction[1];
    }
  }
}

