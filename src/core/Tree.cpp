#include "core/Tree.h"

#include <algorithm>
#include <cstdlib>  // for rand
#include <ctime>    // for time (seed rand)
#include <iostream>
#include <vector>

#include <glad/glad.h>

#include "core/Strand.h"
#include "geometry/Spline.h"
#include "geometry/util.h"
#include "simulation/PBD.h"

void Tree::computeStrandsPosition() {
  time_t t = time(nullptr);
  std::cout << "random seed: " << t << std::endl;
  std::srand(t);

  computeCoordinateSystems();
  computeStrandsInNode(0);  // compute all strands recursively
  applyPBD();
}

// recursively compute strands in specified node of the plant graph
void Tree::computeStrandsInNode(int nodeId) {
  const Node& node = pg.getNode(nodeId);
  glm::mat3 currentFrontplane = frontplanes[nodeId];
  std::vector<int>& children = pg.adj[nodeId];

  // 1. compute strands for children (or create strands if there are no children)
  if (!children.empty()) {
    for (auto child : children) {
      computeStrandsInNode(child);
    }
  } else {
    // leaf nodes (no outgoing branches)
    // generate strand particle positions randomly in a defined radius
    for (int i = 0; i < NUM_STRANDS_PER_LEAF; ++i) {
      float radius = NODE_STRAND_AREA_RADIUS - STRAND_RADIUS;
      float theta = static_cast<float>(std::rand()) / RAND_MAX * 2 * M_PI;

      glm::vec3 particlePos = {radius * std::cos(theta), radius * std::sin(theta), 0};

      Strand strand;
      auto particle = strand.addParticle(node.pos + currentFrontplane * particlePos, particlePos);

      strands.push_back(strand);
      nodeParticles[nodeId].push_back(particle);
    }

    return;  // nothing more to do for leaf nodes
  }

  // 2. merge results
  // if not branching, directly project the strand particle positions from the child plane
  // to the underlying branching node plane
  bool branching = children.size() > 1;
  if (!branching) {
    for (auto child : children) {
      for (auto& particle : nodeParticles[child]) {
        // project in same position
        auto newParticle = strands[particle->strandId].addParticle(
            node.pos + currentFrontplane * particle->localPos, particle->localPos
        );
        nodeParticles[nodeId].push_back(newParticle);
      }
    }

    return;  // nothing more to do for not branch nodes
  }

  // strands coming from multiple branches -> merge algorithm
  // sort the children (ascending) according to their amount of strand particles
  std::sort(children.begin(), children.end(), [&](int a, int b) -> bool {
    return nodeParticles[a].size() > nodeParticles[b].size();
  });

  float dlarge = 0.0f;
  for (int i = 0; i < children.size(); ++i) {
    int child = children[i];
    glm::vec3 dir{glm::normalize(glm::vec2{pg.getNode(child).pos - node.pos}), 0.0f};

    float dsmall = 0.0f;
    for (auto& particle : nodeParticles[child]) {
      // offset (length and direction) to project from origin
      float offset = i != 0 ? dlarge + dsmall : 0;
      glm::vec3 mergedPos = particle->localPos + offset * dir;

      if (i == 0)
        dlarge = std::max(dlarge, glm::length(mergedPos));
      else
        dsmall = std::max(dsmall, glm::length(mergedPos) - dlarge);

      auto newParticle = strands[particle->strandId].addParticle(
          node.pos + currentFrontplane * mergedPos, mergedPos
      );
      nodeParticles[nodeId].push_back(newParticle);
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

  for (auto& [nodeId, particles] : nodeParticles) {
    std::vector<glm::vec3> pos;

    for (auto& particle : particles) pos.push_back(particle->localPos);

    // execute pbd for every node, to "pack" the strands, without intersections
    pbd.setPoints(pos);
    pos = pbd.execute(5 * Strand::getStrandCount());

    // set the strand particles position after running the PBD simulation
    for (int i = 0; i < particles.size(); ++i) {
      particles[i]->pos = pg.getNode(nodeId).pos + frontplanes[nodeId] * pos[i];
      particles[i]->localPos = pos[i];
    }
  }
}

// *normal vector must be normalized. returns pair of closest points on opposite sides of the plane
std::pair<int, int> Tree::findClosestStrandParticlesToPlane(
    int strandId, glm::vec3 planeOrigin, glm::vec3 planeNormal
) {
  const auto& particles = strands[strandId].getParticles();

  assert(particles.size() >= 2);

  float previousDist = glm::dot(planeNormal, particles[0]->pos - planeOrigin);
  for (int i = 1; i < particles.size(); ++i) {
    float dist = glm::dot(planeNormal, particles[i]->pos - planeOrigin);

    if ((previousDist < 0 && dist >= 0) || (previousDist > 0 && dist <= 0)) {
      return {i - 1, i};
    }
  }

  return {-1, -1};  // the strand does not cross the plane
}

void Tree::interpolateAllBranchSegments() {
  for (int i = 0; i < Strand::getStrandCount(); ++i) strands[i].interpolateParticles();
  for (int i = 0; i < Node::getNodeCount(); ++i) interpolateBranchSegment(i);
}

void Tree::interpolateBranchSegment(int branchStartNode) {
  interpolatedNodeParticles[branchStartNode] = {};

  if (pg.adj[branchStartNode].empty()) return;  // ignore leaf nodes

  const int NUM_STEPS = 10;
  const float step = 1.0f / NUM_STEPS;

  const Node& startNode = pg.getNode(branchStartNode);

  for (auto& childBranch : pg.adj[branchStartNode]) {
    const Node& endNode = pg.getNode(childBranch);  // change to user input or tallest
    std::vector<glm::vec3> interpolatedPlaneOrigins =
        Spline::interpolate({startNode.pos, endNode.pos});

    for (int i = 1; i < interpolatedPlaneOrigins.size() - 1; ++i) {
      std::vector<StrandParticle> crossSection;
      // change to compute with tallest child node

      for (auto& nodeParticle : nodeParticles[branchStartNode]) {
        int strandId = nodeParticle->strandId;

        auto [idx1, idx2] = findClosestStrandParticlesToPlane(
            strandId, interpolatedPlaneOrigins[i], frontplanes[childBranch][2]
        );

        if (idx1 > 0 && idx2 > 0) {
          const auto& strandParticles = strands[strandId].getParticles();
          glm::vec3 defaultStart = 2.0f * (strandParticles[idx1]->pos - strandParticles[idx2]->pos);
          glm::vec3 before = idx1 - 1 >= 0 ? strandParticles[idx1 - 1]->pos : defaultStart;
          glm::vec3 after =
              idx2 + 1 < strandParticles.size() ? strandParticles[idx2 + 1]->pos : -defaultStart;

          glm::vec3 interpolatedPos = Spline::catmullRom(
              before, strandParticles[idx1]->pos, strandParticles[idx2]->pos, after, 0.5f
          );

          glm::vec3 projectedPos =
              interpolatedPos -
              glm::dot(interpolatedPos - interpolatedPlaneOrigins[i], frontplanes[childBranch][2]) *
                  frontplanes[childBranch][2];

          crossSection.emplace_back(strandId, projectedPos);
        }
      }
      interpolatedNodeParticles[branchStartNode].push_back(crossSection);
    }
  }
}

/* ---------------------- DISPLAY METHODS ---------------------- */

void Tree::printNodeParticles(int nodeId) const {
  std::cout << "Particles at node ID: " << nodeId << std::endl;
  for (auto& particle : nodeParticles.at(nodeId)) {
    std::cout << *particle << std::endl;
  }
}

std::vector<Mesh> Tree::generateMeshes() const {
  std::vector<Mesh> meshes;
  for (int i = 0; i < Node::getNodeCount(); ++i) {
    std::vector<glm::vec3> worldPositions;
    std::vector<glm::vec2> localPositions;
    for (auto& particle : nodeParticles.at(i)) {
      worldPositions.emplace_back(particle->pos);
      localPositions.emplace_back(particle->localPos);
    }
    meshes.emplace_back(worldPositions, util::delaunay(localPositions));
    for (auto& crossSection : interpolatedNodeParticles.at(i)) {
      worldPositions.clear();
      localPositions.clear();
      for (auto& particle : crossSection) {
        worldPositions.emplace_back(particle.pos);
        localPositions.emplace_back(particle.localPos);
      }
      meshes.emplace_back(worldPositions, util::delaunay(localPositions));
    }
  }

  return meshes;
}

void Tree::renderStrands() const {
  for (auto& strand : strands) {
    strand.renderStrand();
  }
}

void Tree::renderStrandParticles() const {
  for (auto& strand : strands) {
    strand.renderStrandParticles();
  }
}

void Tree::renderInterpolatedParticles() const {
  unsigned int vao, vbo;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);

  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);

  std::vector<glm::vec3> positions;
  for (auto& [id, particles] : nodeParticles) {
    for (auto& particle : nodeParticles.at(id)) positions.push_back(particle->pos);
    for (auto& crossSection : interpolatedNodeParticles.at(id)) {
      for (auto& particle : crossSection) positions.push_back(particle.pos);
    }
  };

  glBufferData(
      GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), &positions[0], GL_STATIC_DRAW
  );

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_POINTS, 0, positions.size());

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}
