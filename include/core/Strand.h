#ifndef __STRAND_H__
#define __STRAND_H__

#include <memory>
#include <ostream>
#include <vector>

#include <glm/glm.hpp>

#include "geometry/Mesh.h"
#include "geometry/Spline.h"
#include "Shader.h"

constexpr int NUM_CIRCLE_VERTICES = 16;
constexpr float STRAND_RADIUS = 0.0075f;

struct StrandParticle {
  int strandId{};
  bool interpolated{};
  glm::vec3 pos;
  glm::vec3 localPos;

  StrandParticle(
      int _strandId, const glm::vec3& worldp, const glm::vec3& localp = {},
      bool _interpolated = false
  )
      : strandId{_strandId}, pos{worldp}, localPos{localp}, interpolated{_interpolated} {}

  friend std::ostream& operator<<(std::ostream& out, const StrandParticle& particle) {
    out << "World: (" << particle.pos.x << ", " << particle.pos.y << ", " << particle.pos.z
        << "); Local: (" << particle.localPos.x << ", " << particle.localPos.y << ", "
        << particle.localPos.z << ").";
    return out;
  }
};

class Strand {
 private:
  inline static int ID_COUNTER = 0;
  std::vector<std::shared_ptr<StrandParticle>> particles;

  // rendering
  Spline spline;
  std::unique_ptr<Mesh> generalizedCylinder;
  glm::vec4 color;

 public:
  const int id;

  explicit Strand() : id{ID_COUNTER++} {
    // use random color for each strand
    // base color is RGB: (111, 186, 131), with a random perturbation
    // for each channel, higher probability of increasing red & green channel

    color = {
        std::min(111.0f / 255.0f + (std::rand() % 40 - 10) / 255.0f, 1.0f),
        std::min(186.0f / 255.0f + (std::rand() % 40 - 10) / 255.0f, 1.0f),
        std::min(131.0f / 255.0f + (std::rand() % 20 - 10) / 255.0f, 1.0f), 1.0f
    };
  }

  static int getStrandCount() { return ID_COUNTER; }

  std::shared_ptr<StrandParticle> addParticle(const glm::vec3& pos, const glm::vec3& localPos = {});

  const std::vector<std::shared_ptr<StrandParticle>>& getParticles() const { return particles; }

  void interpolateParticles();

  // render methods
  void initializeSplineBuffers();
  void renderSpline(const Shader& sh) const;

  void initializeGeneralizedCylinder();
  void renderStrand(const Shader& sh) const;

  void renderStrandParticles() const;

  friend std::ostream& operator<<(std::ostream& out, const Strand& strand) {
    out << "Strand ID: " << strand.id << ". Particles positions: " << std::endl;

    for (const auto& particle : strand.particles) {
      out << '\t' << particle << std::endl;
    }

    return out;
  }
};

#endif
