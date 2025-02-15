#include "core/Strand.h"

#include <memory>

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

void Strand::initializeSplineBuffers() {
  std::vector<glm::vec3> positions(particles.size());

  for (int i = 0; i < particles.size(); ++i) positions[i] = particles[i]->pos;

  spline = Spline(positions);
  spline.initializeBuffers();
}

void Strand::renderSpline(const Shader& sh) const {
  sh.setVec4("color", color);
  spline.render();
}

void Strand::initializeGeneralizedCylinder() {
  // generate vertices in a circle around the strand particles
  std::vector<glm::vec3> vertices;
  std::vector<glm::uvec3> indices;

  for (const auto& particle : particles) {
    glm::vec3 pos = particle->pos;

    // generate a circle around the particle
    for (int i = 0; i < NUM_CIRCLE_VERTICES; ++i) {
      float theta = 2.0f * M_PI * i / NUM_CIRCLE_VERTICES;
      float x = STRAND_RADIUS * cos(theta);
      float z = STRAND_RADIUS * sin(theta);

      vertices.push_back(pos + glm::vec3(x, 0.0f, z));
    }
  }

  // generate indices for the cylinder
  for (int i = 0; i < particles.size() - 1; ++i) {
    for (int j = 0; j < NUM_CIRCLE_VERTICES; ++j) {
      int idx0 = i * NUM_CIRCLE_VERTICES + j;
      int idx1 = i * NUM_CIRCLE_VERTICES + (j + 1) % NUM_CIRCLE_VERTICES;
      int idx2 = (i + 1) * NUM_CIRCLE_VERTICES + j;
      int idx3 = (i + 1) * NUM_CIRCLE_VERTICES + (j + 1) % NUM_CIRCLE_VERTICES;

      indices.push_back(glm::uvec3(idx0, idx1, idx2));
      indices.push_back(glm::uvec3(idx1, idx3, idx2));
    }
  }

  generalizedCylinder = std::make_unique<Mesh>(vertices, indices);
}

void Strand::renderStrand(const Shader& sh) const {
  sh.setVec4("color", color);
  generalizedCylinder->render();
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

