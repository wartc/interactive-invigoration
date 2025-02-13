#include "core/Strand.h"

#include "geometry/Spline.h"

std::shared_ptr<StrandParticle> Strand::addParticle(
    const glm::vec3& pos, const glm::vec3& localPos
) {
  auto particle = std::make_shared<StrandParticle>(id, pos, localPos);
  particles.push_back(particle);

  return particle;
}

void Strand::interpolateParticles() {
  std::vector<glm::vec3> positions(particles.size());

  for (int i = 0; i < particles.size(); ++i) positions[i] = particles[i]->pos;

  auto interpolatedPositions = Spline::interpolate(positions);

  // new particles vector
  std::vector<std::shared_ptr<StrandParticle>> updatedParticles;

  int particleIndex = 0;  // idx for existing particles
  for (const auto& interpolatedPos : interpolatedPositions) {
    if (particles[particleIndex]->pos == interpolatedPos) {
      // if this interpolated position matches an existing particle, keep the original
      updatedParticles.push_back(particles[particleIndex]);
      ++particleIndex;  // Move to the next existing particle
    } else {
      auto newParticle =
          std::make_shared<StrandParticle>(id, interpolatedPos, glm::vec3(0.0f), true);
      updatedParticles.push_back(newParticle);
    }
  }

  particles = std::move(updatedParticles);
}

void Strand::renderStrand() const {
  std::vector<glm::vec3> positions(particles.size());

  for (int i = 0; i < particles.size(); ++i) positions[i] = particles[i]->pos;

  Spline spline(positions);
  spline.smoothenSpline();

  spline.initializeBuffers();
  spline.render();
}

void Strand::renderStrandParticles() const {
  unsigned int vao, vbo;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);

  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);

  std::vector<glm::vec3> positions(particles.size());
  for (int i = 0; i < particles.size(); ++i) positions[i] = particles[i]->pos;

  glBufferData(
      GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), &positions[0], GL_STATIC_DRAW
  );

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_POINTS, 0, positions.size());

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

