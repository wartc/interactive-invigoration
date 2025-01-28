#ifndef __STRAND_H__
#define __STRAND_H__

#include <memory>
#include <ostream>
#include <vector>

#include <glm/glm.hpp>

struct StrandParticle {
  int strandId;
  glm::vec3 pos;
  glm::vec3 localPos;

  StrandParticle(int _strandId, const glm::vec3& worldp, const glm::vec3& localp = {})
      : strandId{_strandId}, pos{worldp}, localPos{localp} {}

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

 public:
  const int id;

  explicit Strand() : id{ID_COUNTER++} {}

  static int getStrandCount() { return ID_COUNTER; }

  std::shared_ptr<StrandParticle> addParticle(const glm::vec3& pos, const glm::vec3& localPos = {});

  const std::vector<std::shared_ptr<StrandParticle>>& getParticles() { return particles; }

  void interpolateParticles();
  void renderStrandParticles() const;
  void renderStrand() const;

  friend std::ostream& operator<<(std::ostream& out, const Strand& strand) {
    out << "Strand ID: " << strand.id << ". Particles positions: " << std::endl;

    for (const auto& particle : strand.particles) {
      out << '\t' << particle << std::endl;
    }

    return out;
  }
};

#endif
