#ifndef __TREE_H__
#define __TREE_H__

#include <map>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "core/PlantGraph.h"
#include "core/Strand.h"

constexpr int NUM_STRANDS_PER_LEAF = 10;
constexpr float STRAND_RADIUS = 0.02f;
constexpr float NODE_STRAND_AREA_RADIUS = 0.1f;
constexpr glm::mat3 DEFAULT_COORDINATES{
    {1.0f, 0.0f,  0.0f},
    {0.0f, 0.0f, -1.0f},
    {0.0f, 1.0f,  0.0f}
};

class Tree {
  PlantGraph& pg;

  std::map<int, glm::mat3> frontplanes;
  std::vector<Strand> strands;
  std::map<int, std::vector<std::shared_ptr<StrandParticle>>> nodeParticles;

 public:
  Tree(PlantGraph& _pg) : pg{_pg} {}

  void computeStrandsPosition();
  void interpolateStrandParticles();

  void renderStrands() const;
  void renderStrandParticles() const;

  void printNodeParticles(int nodeId) const;

 private:
  void computeStrandsInNode(int nodeId);
  void computeCoordinateSystems();
  void applyPBD();
};

#endif
