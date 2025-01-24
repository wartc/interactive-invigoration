#ifndef __PLANT_GRAPH__H
#define __PLANT_GRAPH__H

#include <cassert>
#include <memory>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

// TODO: implement auto-id (like in the Strand struct)
struct Node {
 public:
  int id;
  int parentId;
  glm::vec3 pos;
  // float diameter;
  // float length;
  // float shootFluxSignal;
  // ...

  inline bool isRoot() const { return parentId == -1; }
};

struct PlantGraph {
 private:
  int idCounter;  // unique node IDs

 public:
  std::unordered_map<int, std::unique_ptr<Node>> nodes;  // stored node data
  std::unordered_map<int, std::vector<int>> adj;         // graphs adjacency list

  PlantGraph() : idCounter(0) {}

  PlantGraph(const glm::vec3& root) : idCounter(0) { addNode(root); }

  int addNode(const glm::vec3& pos, int parentId = -1) {
    int id = idCounter++;

    // initialize node
    nodes[id] = std::make_unique<Node>(Node{id, parentId, pos});
    adj[id] = std::vector<int>();

    // if it has a parent, link to the parent
    if (parentId != -1 && adj.find(parentId) != adj.end()) adj[parentId].push_back(id);

    return id;
  }

  // add edge between two existing nodes
  void addEdge(int id1, int id2) {
    assert(adj.find(id1) != adj.end() && adj.find(id2) != adj.end());

    adj[id1].push_back(id2);
  }

  const Node& getNode(int id) const { return *nodes.at(id); }

  void printGraph() const {
    for (const auto& [id, neighbors] : adj) {
      std::cout << "Node " << id << " adjecency: ";
      for (const auto& neighbor : neighbors) {
        std::cout << nodes.at(neighbor)->id << " ";
      }
      std::cout << "\n";
    }
  }

  void traverseDFS(int start, std::function<void(const Node&)> fn) const {
    std::vector<bool> visited(adj.size(), false);

    dfs(start, fn, visited);
  }

 private:
  void dfs(int id, std::function<void(const Node&)> fn, std::vector<bool>& visited) const {
    visited[id] = true;
    fn(getNode(id));

    for (int neighbor : adj.at(id)) {
      if (!visited[neighbor]) dfs(neighbor, fn, visited);
    }
  }
};

#endif
