#ifndef __STRAND_H__
#define __STRAND_H__

#include <map>

#include <glm/gtc/matrix_transform.hpp>

#include "PlantGraph.h"
#include "Spline.h"

constexpr int NUM_STRANDS_PER_LEAF = 10;
constexpr float STRAND_RADIUS = 0.02f;
constexpr float NODE_STRAND_AREA_RADIUS = 0.1f;
constexpr glm::mat3 DEFAULT_COORDINATES{
    {1.0f, 0.0f,  0.0f},
    {0.0f, 0.0f, -1.0f},
    {0.0f, 1.0f,  0.0f}
};

struct Strand {
 private:
  inline static int ID_COUNTER = 0;

 public:
  const int id;
  glm::vec3 pos;

  explicit Strand(int _id, glm::vec3 _pos) : id{_id}, pos{_pos} {}
  explicit Strand(glm::vec3 _pos) : id{ID_COUNTER++}, pos{_pos} {}

  static int getStrandCount() { return ID_COUNTER; }
};

class Tree {
  PlantGraph& pg;

  std::map<int, glm::mat3> frontplanes;
  std::map<int, std::vector<Strand>> nodeStrands;

 public:
  Tree(PlantGraph& _pg) : pg{_pg} {}

  void computeStrandsPosition();
  std::vector<Spline> generateSplines() const;
  void showStrandsPoints();
  void printNodeStrands() const;

 private:
  void computeStrandsInNode(const Node& node);
  void computeCoordinateSystems();
  void applyPBD();
};

#endif
