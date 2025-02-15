#include "core/Tree.h"

#include <algorithm>
#include <cstdlib>  // for rand
#include <ctime>    // for time (seed rand)
#include <iostream>
#include <vector>

#include <glad/glad.h>

#include "core/Strand.h"
#include "geometry/Spline.h"  // for NUM_INTERPOLATED_POINTS
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

      strands.emplace_back(std::move(strand));
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
  PBD pbd({}, attractors, 0.02, 0.002, STRAND_RADIUS, {0.0f, 0.0f, 0.0f}, NODE_STRAND_AREA_RADIUS);

  for (auto& [nodeId, particles] : nodeParticles) {
    std::vector<glm::vec3> pos;

    for (auto& particle : particles) pos.push_back(particle->localPos);

    // execute pbd for every node, to "pack" the strands, without intersections
    pbd.setPoints(pos);
    pos = pbd.execute(
        5 * Strand::getStrandCount(), {0.0f, 0.0f, 0.0f}, 0.1 * pos.size() * NODE_STRAND_AREA_RADIUS
    );

    // set the strand particles position after running the PBD simulation
    for (int i = 0; i < particles.size(); ++i) {
      particles[i]->pos = pg.getNode(nodeId).pos + frontplanes[nodeId] * pos[i];
      particles[i]->localPos = pos[i];
    }
  }
}

void Tree::computeCrossSections() {
  for (int i = 0; i < Strand::getStrandCount(); ++i) strands[i].interpolateParticles();
  for (int i = 0; i < Node::getNodeCount(); ++i) interpolateBranchSegment(i);
  triangulateCrossSections();
}

void Tree::interpolateBranchSegment(int branchStartNode) {
  interpolatedCrossSections[branchStartNode] = {};

  if (pg.adj[branchStartNode].empty()) return;  // ignore leaf nodes

  for (auto& childId : pg.adj[branchStartNode]) {
    for (int i = 1; i < NUM_INTERPOLATED_POINTS; ++i) {
      CrossSection crossSection;

      // add the corresponding interpolation level to the cross section (not coplanar yet)
      for (auto& particle : nodeParticles[childId]) {
        const auto& strandParticles = strands[particle->strandId].getParticles();

        int idx;
        for (idx = 0; idx < strandParticles.size(); ++idx) {
          if (strandParticles[idx] == particle) break;
        }

        crossSection.particlePositions.push_back(strandParticles[idx + i]->pos);
        crossSection.particleStrandIds.push_back(particle->strandId);
        crossSection.particleIndices.push_back(idx + i);
      }

      // calculate least squares plane
      glm::vec3 planeOrigin, planeNormal;

      std::tie(planeOrigin, planeNormal) =
          util::computeLeastSquaresFittingPlane(crossSection.particlePositions);

      crossSection.particleNormals.resize(crossSection.getNumParticles());
      glm::vec3 centroid{0.0f};
      for (int j = 0; j < crossSection.getNumParticles(); ++j) {
        centroid += glm::vec3(crossSection.particlePositions[j]);
      }
      centroid /= crossSection.getNumParticles();

      // construct orthonormal basis
      glm::vec3 xaxis = glm::normalize(crossSection.particlePositions[0] - planeOrigin);
      xaxis = glm::normalize(xaxis - glm::dot(xaxis, planeNormal) * planeNormal);
      glm::vec3 yaxis = glm::normalize(glm::cross(planeNormal, xaxis));
      glm::mat3 basis = glm::mat3(xaxis, yaxis, planeNormal);

      // project points to plane and change to 2d basis
      for (int j = 0; j < crossSection.getNumParticles(); ++j) {
        glm::vec3 v = crossSection.particlePositions[j] - planeOrigin;
        glm::vec3 projected =
            crossSection.particlePositions[j] - glm::dot(v, planeNormal) * planeNormal;
        glm::vec3 local = glm::transpose(basis) * (projected - planeOrigin);

        crossSection.particleNormals[j] =
            glm::normalize(crossSection.particlePositions[j] - centroid);
        crossSection.particlePositions[j] = glm::vec3(local.x, local.y, 0.0f);
      }

      interpolatedCrossSections[branchStartNode].push_back(crossSection);
    }
  }
}

Mesh Tree::generateMesh() const {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::uvec3> indices;

  int vertexOffset = 0;
  for (int nodeId = 0; nodeId < Node::getNodeCount(); ++nodeId) {
    // first: node particles (not interpolated)
    const auto& nodeParts = nodeParticles.at(nodeId);
    for (const auto& particle : nodeParts) {
      vertices.push_back(particle->pos);
      normals.push_back(glm::normalize(particle->pos - pg.getNode(nodeId).pos));
    }

    for (const auto& triangle : crossSectionsTriangulations.at({nodeId, -1})) {
      indices.push_back(glm::uvec3(vertexOffset) + triangle);
    }

    vertexOffset += nodeParts.size();

    // second: interpolated cross sections
    for (int crossIdx = 0; crossIdx < interpolatedCrossSections.at(nodeId).size(); ++crossIdx) {
      const auto& curCrossSection = interpolatedCrossSections.at(nodeId)[crossIdx];
      const int crossSectionSize = curCrossSection.getNumParticles();

      // add the particle positions to the vertices
      for (int i = 0; i < crossSectionSize; ++i) {
        int strandId = curCrossSection.particleStrandIds[i];
        int idx = curCrossSection.particleIndices[i];

        vertices.push_back(strands[strandId].getParticles()[idx]->pos);
        normals.push_back(curCrossSection.particleNormals[i]);
      }

      // actual cross section triangulation
      for (const auto& triangle : crossSectionsTriangulations.at({nodeId, crossIdx})) {
        indices.push_back(glm::uvec3(vertexOffset) + triangle);
      }

      std::cout << "Interpolated cross section at node: " << nodeId << " with index: " << crossIdx
                << std::endl;

      // connect the cross section with the previous one
      CrossSection previousCrossSection;
      if (crossIdx == 0) {
        // connect with the last node particles if this is the first cross section
        for (int i = 0; i < nodeParts.size(); ++i) {
          previousCrossSection.particlePositions.push_back(nodeParts[i]->pos);
          previousCrossSection.particleStrandIds.push_back(nodeParts[i]->strandId);
        }
      } else {
        previousCrossSection = interpolatedCrossSections.at(nodeId)[crossIdx - 1];
      }

      // connect the previous boundary particles with corresponding strand ids
      // search the previous cross section particleStrandIds for the current strandId
      int curBoundaryIdx = 0;
      for (int i : curCrossSection.boundaryVertices) {
        curBoundaryIdx++;
        int strandId = curCrossSection.particleStrandIds[i];

        int matchingVertex = -1;
        int matchingIdx = 0;
        for (int j : previousCrossSection.boundaryVertices) {
          if (previousCrossSection.particleStrandIds[j] == strandId) {
            matchingVertex = j;
            break;
          }
          matchingIdx++;
        }

        std::cout << "Connecting strandId: " << strandId << " at current index: " << i
                  << " with previous index: " << matchingVertex << std::endl;

        if (matchingVertex == -1) {
          std::cerr << "Error: could not find corresponding strandId in previous cross section"
                    << std::endl;
          continue;
        }

        int prevOffset = vertexOffset - previousCrossSection.getNumParticles();
        int nextCurBoundaryIdx = (curBoundaryIdx) % curCrossSection.boundaryVertices.size();
        int nextMatchingIdx = (matchingIdx + 1) % previousCrossSection.boundaryVertices.size();
        int afterCurrent = curCrossSection.boundaryVertices[nextCurBoundaryIdx];
        int afterMatching = previousCrossSection.boundaryVertices[nextMatchingIdx];

        indices.emplace_back(
            vertexOffset + i, prevOffset + matchingVertex, prevOffset + afterMatching
        );
        indices.emplace_back(
            vertexOffset + i, prevOffset + afterMatching, vertexOffset + afterCurrent
        );
      }

      // @TODO: for the last cross section, connect with next node particles with corresponding
      // strand ids

      vertexOffset += crossSectionSize;
    }
  }

  return Mesh{vertices, indices, normals};
}

void Tree::triangulateCrossSections() {
  for (int i = 0; i < Node::getNodeCount(); ++i) {
    // mesh for not interpolated node particles
    std::vector<glm::vec2> planarCoords;
    for (auto& particle : nodeParticles.at(i)) {
      planarCoords.emplace_back(particle->localPos);
    }

    crossSectionsTriangulations[{i, -1}] = util::delaunay(planarCoords);

    // mesh for interpolated strand particles
    int crossSectionIdx = 0;
    for (auto& crossSection : interpolatedCrossSections[i]) {
      planarCoords.clear();

      for (int j = 0; j < crossSection.getNumParticles(); ++j) {
        planarCoords.emplace_back(
            crossSection.particlePositions[j].x, crossSection.particlePositions[j].y
        );
      }

      auto triangles = util::delaunay(planarCoords);
      crossSectionsTriangulations[{i, crossSectionIdx}] = triangles;
      crossSection.boundaryVertices = util::computeBoundaryVertices(planarCoords, triangles);

      crossSectionIdx++;
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

void Tree::initializeStrandBuffers() {
  for (auto& strand : strands) {
    strand.initializeGeneralizedCylinder();
  }
}

void Tree::renderStrands(const Shader& sh) const {
  for (auto& strand : strands) {
    strand.renderStrand(sh);
  }
}

void Tree::renderStrandParticles() const {
  for (auto& strand : strands) {
    strand.renderStrandParticles();
  }
}
