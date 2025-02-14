#ifndef __TREE_H__
#define __TREE_H__

#include <cstddef>
#include <map>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "core/PlantGraph.h"
#include "core/Strand.h"
#include "geometry/Mesh.h"

constexpr int NUM_STRANDS_PER_LEAF = 10;
constexpr float STRAND_RADIUS = 0.02f;
constexpr float NODE_STRAND_AREA_RADIUS = 0.1f;
constexpr glm::mat3 DEFAULT_COORDINATES{
    {1.0f, 0.0f,  0.0f},
    {0.0f, 0.0f, -1.0f},
    {0.0f, 1.0f,  0.0f}
};

struct CrossSection {
  std::vector<glm::vec3> particlePositions{};
  std::vector<int> particleStrandIds{};
  std::vector<int> particleIndices{};

  int getNumParticles() const { return particlePositions.size(); }
};

class Tree {
  PlantGraph& pg;

  std::map<int, glm::mat3> frontplanes;
  std::vector<Strand> strands;
  std::map<int, std::vector<std::shared_ptr<StrandParticle>>> nodeParticles;

  std::map<int, std::vector<CrossSection>> interpolatedCrossSections;

  // maps a pair (node id, cross section index) to its corresponding triangle indices
  std::map<std::pair<int, int>, std::vector<glm::uvec3>> crossSectionsTriangulations;

 public:
  Tree(PlantGraph& _pg) : pg{_pg} {}

  // strand position computation
  void computeStrandsPosition();
  void interpolateAllBranchSegments();

  // mesh generation
  void triangulateCrossSections();
  Mesh generateMesh() const;

  // render methods
  void renderStrands() const;
  void renderStrandParticles() const;

  void printNodeParticles(int nodeId) const;

 private:
  void computeStrandsInNode(int nodeId);
  void computeCoordinateSystems();
  void applyPBD();

  void interpolateBranchSegment(int branchStartNode);
};

#endif
