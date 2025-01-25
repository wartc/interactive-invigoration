#include "Tree.h"

#include <algorithm>
#include <cstdlib>  // for rand
#include <ctime>    // for time (seed rand)
#include <iostream>
#include <vector>

#include "PBD.h"
#include "PlantGraph.h"

void Tree::computeStrandsPosition() {
  const Node& root = pg.getNode(0);

  time_t t = time(nullptr);
  std::cout << "random seed: " << t << std::endl;
  std::srand(t);

  computeCoordinateSystems();
  computeStrandsInNode(root);  // compute all strands recursively
  applyPBD();
}

// recursively compute strands in specified node of the plant graph
void Tree::computeStrandsInNode(const Node& node) {
  std::vector<int>& children = pg.adj[node.id];

  // 1. compute strands for children (or create strands if there are no children)
  if (!children.empty()) {
    for (auto child : children) {
      computeStrandsInNode(pg.getNode(child));
    }
  } else {
    // leaf nodes (no outgoing branches)
    // generate strand positions randomly in a defined radius
    for (int i = 0; i < NUM_STRANDS_PER_LEAF; ++i) {
      float radius = NODE_STRAND_AREA_RADIUS - STRAND_RADIUS;
      float theta = static_cast<float>(std::rand()) / RAND_MAX * 2 * M_PI;

      glm::vec3 strandPos = {radius * std::cos(theta), radius * std::sin(theta), 0};

      nodeStrands[node.id].push_back(Strand(strandPos));
    }

    return;  // nothing more to do for leaf nodes
  }

  // 2. merge results
  // if not branching, directly project the strand particle positions from the child plane
  // to the underlying branching node plane
  bool branching = children.size() > 1;
  if (!branching) {
    for (auto child : children) {
      for (auto& strandPos : nodeStrands[child]) {
        nodeStrands[node.id].push_back(strandPos);
      }
    }

    return;  // nothing more to do for not branch nodes
  }

  // strands coming from multiple branches -> merge algorithm
  // sort the children (ascending) according to their amount of strands
  std::sort(children.begin(), children.end(), [&](int a, int b) {
    return nodeStrands[a].size() > nodeStrands[b].size();
  });

  float dlarge = 0.0f;
  for (int i = 0; i < children.size(); ++i) {
    int child = children[i];
    glm::vec3 dir{glm::normalize(glm::vec2{pg.getNode(child).pos - node.pos}), 0.0f};

    float dsmall = 0.0f;
    for (auto& strand : nodeStrands[child]) {
      // offset (length and direction) to project from origin
      float offset = i != 0 ? dlarge + dsmall : 0;
      glm::vec3 mergedPos = strand.pos + offset * dir;

      if (i == 0)
        dlarge = std::max(dlarge, glm::length(mergedPos));
      else
        dsmall = std::max(dsmall, glm::length(mergedPos) - dlarge);

      nodeStrands[node.id].push_back(Strand(strand.id, mergedPos));
    }
  }
}

void Tree::computeCoordinateSystems() {
  pg.traverseDFS(0, [&](const Node& n) {
    // frontplanes
    if (!n.isRoot()) {
      glm::vec3 yparent = frontplanes[n.parentId][1];

      glm::vec3 zaxis = glm::normalize(n.pos - pg.getNode(n.parentId).pos);
      glm::vec3 xaxis = glm::normalize(glm::cross(yparent, zaxis));
      glm::vec3 yaxis = glm::cross(zaxis, xaxis);

      frontplanes[n.id] = glm::mat3(xaxis, yaxis, zaxis);
    } else {
      frontplanes[n.id] = DEFAULT_COORDINATES;
    }
  });
}

void Tree::applyPBD() {
  std::vector<glm::vec3> attractors{
      {0.0f, 0.0f, 0.0f}
  };
  PBD pbd(
      {}, attractors, 0.02, 0.002, STRAND_RADIUS, {0.0f, 0.0f, 0.0f}, 0.5 * NODE_STRAND_AREA_RADIUS
  );

  for (auto& [id, strands] : nodeStrands) {
    std::vector<glm::vec3> pos;

    for (auto& strand : strands) pos.push_back(strand.pos);

    // execute pbd for every node, to "pack" the strands, without intersections
    pbd.setPoints(pos);
    pos = pbd.execute(5 * Strand::getStrandCount());

    for (int i = 0; i < strands.size(); ++i) strands[i].pos = pos[i];
  }
}

/* ---------------------- DISPLAY METHODS ---------------------- */

void Tree::printNodeStrands() const {
  for (const auto& [id, strands] : nodeStrands) {
    std::cout << "Node ID: " << id << std::endl;
    for (size_t i = 0; i < strands.size(); ++i) {
      std::cout << "\tStrand id=" << strands[i].id << ": (" << strands[i].pos.x << ", "
                << strands[i].pos.y << ", " << strands[i].pos.z << ")" << std::endl;
    }
  }
}

std::vector<Spline> Tree::generateSplines() const {
  int nSplines = Strand::getStrandCount();
  std::vector<Spline> splines;
  std::vector<std::vector<glm::vec3>> allPositions(nSplines);

  pg.traverseDFS(0, [&](const Node& n) {
    for (const auto& strand : nodeStrands.at(n.id)) {
      allPositions[strand.id].push_back(n.pos + frontplanes.at(n.id) * strand.pos);
    }
  });

  for (int i = 0; i < nSplines; ++i) {
    splines.emplace_back(allPositions[i]);
  }

  return splines;
}

void Tree::showStrandsPoints() {
  unsigned int vao, vbo;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);

  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);

  std::vector<glm::vec3> positions;
  for (const auto& [id, strands] : nodeStrands) {
    glm::mat3 fp = frontplanes[id];
    glm::vec3 origin = pg.getNode(id).pos;

    for (const auto& strand : strands) {
      positions.push_back(origin + fp * strand.pos);
    }
  }

  glBufferData(
      GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), &positions[0], GL_STATIC_DRAW
  );

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_POINTS, 0, positions.size());

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}
