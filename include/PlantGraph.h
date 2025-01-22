#ifndef __PLANT_GRAPH__H
#define __PLANT_GRAPH__H

#include <ostream>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <glm/glm.hpp>

struct Node {
 private:
  static int ID_COUNT;  // Static counter for unique IDs

 public:
  int id;
  int parentId;
  glm::vec3 pos;
  // float diameter;
  // float length;
  // float shootFluxSignal;
  // ...

  friend std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << "Node ID: " << node.id << ", position: (" << node.pos.x << ", " << node.pos.y << ", "
       << node.pos.z << ")";
    return os;
  }
};

struct PlantGraph {
 private:
  int idCounter;  // unique node ids

 public:
  std::unordered_map<int, Node> nodes;           // node properties by id
  std::unordered_map<int, std::set<int>> graph;  // graph connections

  PlantGraph() : idCounter(0) {}

  PlantGraph(const glm::vec3& root) : idCounter(0) { addNode(root); }

  int addNode(const glm::vec3& pos, int parentId = 0) {
    int id = idCounter++;

    nodes[id] = {id, parentId, pos};

    // add node to the graph (without neighbors initially)
    graph[id] = std::set<int>();

    if (graph.find(parentId) != graph.end()) graph[parentId].insert(id);

    return id;
  }

  void addEdge(int id1, int id2) {
    // directed graph
    graph[id1].insert(id2);
    nodes[id2].parentId = id1;
  }

  Node getNode(int id) {
    if (id < idCounter) return nodes[id];
    throw;
  }

  void traverse(int start, std::function<void(Node)> fn) {
    std::unordered_set<int> visited;
    dfs(start, fn, visited);
  }

 private:
  void dfs(int start, std::function<void(Node)> fn, std::unordered_set<int>& visited) {
    if (visited.find(start) != visited.end()) {
      return;
    }

    visited.insert(start);
    fn(nodes[start]);

    // explore adjacent nodes
    for (int adj : graph[start]) {
      dfs(adj, fn, visited);
    }
  }
};

#endif
