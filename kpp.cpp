#include <algorithm>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <ostream>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

/*
        ========== DATA AND MODELS =============
*/
struct Node;

struct Edge {
  int length;
  int obs;
  int energyNeeded;
  std::vector<Node *> connectedNodes;

  Edge(int paramLength, int paramObs) {
    length = paramLength;
    obs = paramObs;
    energyNeeded = length + obs;
  }

  int GetEnergyNeeded(int currentTime) {
    return (currentTime % 2 == 0) ? energyNeeded * 0.8 : energyNeeded * 1.3;
  }
};

enum NodeType {
  DEFAULT,
  REST,
  ELECTRICAL,
  MECHANIC,
  CHARGING,
};

struct Node {
  std::string name;
  enum NodeType type;
  std::vector<Edge *> connectedEdges;
};

struct DijkstraNode {
  Node *node;
  int spentEnergy;
  double spentTime;

  bool operator>(const DijkstraNode &other) const {
    return spentEnergy > other.spentEnergy;
  }
};

/*
        ============= HELPERS ==================
*/

Node *GetorCreateNode(std::unordered_map<std::string, Node *> &nodeMap,
                      std::deque<Node> &globalNodes, const std::string name) {
  if (nodeMap.find(name) == nodeMap.end()) {
    // Construct node inside globalNodes
    globalNodes.push_back(Node{name, DEFAULT, {}});
    Node *nodePtr = &globalNodes.back();
    nodeMap[name] = nodePtr;
  }
  return nodeMap[name];
}

bool TryChangingNodeType(std::unordered_map<std::string, Node *> &nodeMap,
                         const std::string nodeName,
                         const NodeType targetType) {
  if (nodeMap.find(nodeName) != nodeMap.end()) {
    nodeMap[nodeName]->type = targetType;
    return true;
  } else {
    return false;
  }
}

void PrintSettings(const std::deque<Node> &nodes, const std::deque<Edge> &edges,
                   const Node *startNode, const Node *endNode, int globalTime) {
  std::cout << "\n===== MAP SUMMARY =====\n";

  // Nodes
  std::cout << "Nodes:\n";
  for (const auto &n : nodes) {
    std::cout << " - " << n.name << " (Type: ";
    switch (n.type) {
    case DEFAULT:
      std::cout << "DEFAULT";
      break;
    case REST:
      std::cout << "REST";
      break;
    case ELECTRICAL:
      std::cout << "ELECTRICAL";
      break;
    case MECHANIC:
      std::cout << "MECHANIC";
      break;
    case CHARGING:
      std::cout << "CHARGING";
      break;
    }
    std::cout << "), Connected edges: " << n.connectedEdges.size() << "\n";
  }

  // Edges
  std::cout << "\nEdges:\n";
  int i = 0;
  for (const auto &e : edges) {
    std::cout << " - Edge " << i++ << " (Length: " << e.length
              << ", Obs: " << e.obs << ", Energy: " << e.energyNeeded
              << ") connects: ";
    for (const auto *n : e.connectedNodes) {
      std::cout << n->name << " ";
    }
    std::cout << "\n";
  }

  // Start/End
  std::cout << "\nStart node: " << (startNode ? startNode->name : "None")
            << "\n";
  std::cout << "End node:   " << (endNode ? endNode->name : "None") << "\n";

  // Time
  std::cout << "Starting time: " << globalTime << " minutes\n";

  std::cout << "========================\n";
}

/*
        ============== CORE FUNCTIONS ===================
*/

void STNodeInput(std::unordered_map<std::string, Node *> &nodeMap,
                 Node *&startNode, Node *&endNode) {
  std::string startNodeName, endNodeName;

  while (true) {
    std::cout << "*start node* *end node*: ";
    std::cin >> startNodeName >> endNodeName;

    if (nodeMap.find(startNodeName) != nodeMap.end() &&
        nodeMap.find(endNodeName) != nodeMap.end()) {
      startNode = nodeMap[startNodeName];
      endNode = nodeMap[endNodeName];
      break;
    }
    std::cout << "Invalid node, try again.\n";
  }
}

void NodeTypeInput(std::unordered_map<std::string, Node *> &nodeMap,
                   const std::string prompt, const NodeType type) {
  std::string line;

  std::cout
      << std::endl
      << prompt +
             " point node (pisah dengan spasi. Input - jika berhenti/null): ";
  std::getline(std::cin >> std::ws, line);

  std::istringstream iss(line);
  std::string nodeName;

  while (iss >> nodeName) {
    if (nodeName == "-")
      break;

    if (!TryChangingNodeType(nodeMap, nodeName, type)) {
      std::cout << "Invalid node: " << nodeName << "\n";
    }
  }
}

void GetInputs(int *nodeCount, int *edgeCount, std::deque<Edge> *globalEdges,
               std::deque<Node> *globalNodes, Node *&startNode, Node *&endNode,
               double *globalTime) {
  std::unordered_map<std::string, Node *> nodeMap;

  while (true) {
    std::cout << "*Jumlah node* *jumlah edge*: ";
    std::cin >> *nodeCount >> *edgeCount;

    if (*edgeCount >= *nodeCount - 1)
      break;

    std::cout << "Jumlah edge minimal harus " << *nodeCount - 1
              << " agar semua node bisa terhubung.\n";
  }
  while (true) {
    globalEdges->clear();
    globalNodes->clear();
    nodeMap.clear();

    std::cout << std::endl
              << "==== SET UP EDGE ====" << std::endl
              << "format: asalNode tujuanNode panjangEdge obstacleEdge"
              << std::endl;

    bool edgeSetupFailed = false;

    for (int i = 0; i < *edgeCount; i++) {
      std::string nodeName1, nodeName2;
      int edgeLength, obs;

      std::cout << "Edge ke-" << i << ": ";
      std::cin >> nodeName1 >> nodeName2 >> edgeLength >> obs;

      // --- Check node quota ---
      int newNodesCount = 0;
      if (nodeMap.find(nodeName1) == nodeMap.end())
        newNodesCount++;
      if (nodeMap.find(nodeName2) == nodeMap.end())
        newNodesCount++;

      if (globalNodes->size() + newNodesCount > *nodeCount) {
        std::cout << "Jumlah node dari semua edge melebihi " << *nodeCount
                  << ". Ulangi input edge ini!\n";
        i--;
        continue;
      }

      // Create Nodes
      Node *n1 = GetorCreateNode(nodeMap, *globalNodes, nodeName1);
      Node *n2 = GetorCreateNode(nodeMap, *globalNodes, nodeName2);

      // Create edge
      globalEdges->emplace_back(edgeLength, obs);
      Edge *edgePtr = &globalEdges->back();

      // Link edge <-> globalNodes
      edgePtr->connectedNodes.push_back(n1);
      edgePtr->connectedNodes.push_back(n2);

      n1->connectedEdges.push_back(edgePtr);
      n2->connectedEdges.push_back(edgePtr);
    }

    if ((int)globalNodes->size() < *nodeCount) {
      std::cout << "Jumlah node dari semua edge kurang dari " << *nodeCount
                << ". Ulangi seluruh setup edge!\n";
      continue;
    }

    std::cout << "Edge setup finished!" << std::endl
              << "Setup tipe node dimulai!" << std::endl;
    break;
  }

  STNodeInput(nodeMap, startNode, endNode);
  NodeTypeInput(nodeMap, "Rest", REST);
  NodeTypeInput(nodeMap, "Charging", CHARGING);
  NodeTypeInput(nodeMap, "Mechanic", MECHANIC);
  NodeTypeInput(nodeMap, "Electrical", ELECTRICAL);

  double awalJam;
  std::cout << "Jam awal perjalanan (dalam satuan jam): ";
  std::cin >> awalJam;
  *globalTime = awalJam * 60;

  return;
}

std::vector<Node *> Dijkstra(std::deque<Node> *globalNodes, Node *startNode,
                             Node *endNode,
                             std::unordered_map<Node *, int> &nodesToEnergy,
                             std::unordered_map<Node *, int> &nodesToTime,
                             double startTime) {

  std::priority_queue<DijkstraNode, std::vector<DijkstraNode>,
                      std::greater<DijkstraNode>>
      dijkstraNodePQ;

  std::unordered_map<Node *, Node *> prev;

  for (Node &n : *globalNodes) {
    nodesToEnergy[&n] = std::numeric_limits<int>::max();
    nodesToTime[&n] = std::numeric_limits<int>::max();
    prev[&n] = nullptr;
  }

  nodesToEnergy[startNode] = 0;
  nodesToTime[startNode] = startTime;
  dijkstraNodePQ.push({startNode, 0, startTime});

  while (!dijkstraNodePQ.empty()) {
    DijkstraNode current = dijkstraNodePQ.top();
    dijkstraNodePQ.pop();

    if (current.spentEnergy > nodesToEnergy[current.node])
      continue;

    if (current.node == endNode)
      break;

    int currentTime = current.spentTime;
    if (current.node->type == NodeType::REST && (currentTime % 2 == 1)) {
      ++currentTime;
    }

    for (Edge *e : current.node->connectedEdges) {
      Node *neighbor = (e->connectedNodes[0] == current.node)
                           ? e->connectedNodes[1]
                           : e->connectedNodes[0];

      int edgeEnergy = e->GetEnergyNeeded(currentTime);
      int arrivalEnergy = current.spentEnergy + edgeEnergy;
      double arrivalTime = currentTime + 2;

      if (arrivalEnergy < nodesToEnergy[neighbor]) {
        nodesToEnergy[neighbor] = arrivalEnergy;
        nodesToTime[neighbor] = arrivalTime;
        prev[neighbor] = current.node;

        dijkstraNodePQ.push({neighbor, arrivalEnergy, arrivalTime});
      }
    }
  }

  // reconstruct path
  if (nodesToEnergy[endNode] == std::numeric_limits<int>::max())
    return {}; // unreachable

  std::vector<Node *> path;
  for (Node *v = endNode; v != nullptr; v = prev[v]) {
    path.push_back(v);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

int main() {
  // MAP
  int nodeCount, edgeCount;
  std::deque<Edge> edgeObjects;
  std::deque<Node> nodeObjects;
  Node *startNode, *endNode;

  // DATA
  int globalEnergy;
  double globalTime;
  const int maxEnergy = 1000;

  GetInputs(&nodeCount, &edgeCount, &edgeObjects, &nodeObjects, startNode,
            endNode, &globalTime);

  PrintSettings(nodeObjects, edgeObjects, startNode, endNode, globalTime);

  // Run Dijkstra
  std::unordered_map<Node *, int> nodesToEnergy, nodesToTime;
  std::vector<Node *> path = Dijkstra(&nodeObjects, startNode, endNode,
                                      nodesToEnergy, nodesToTime, globalTime);

  // If habis energy
  globalEnergy = maxEnergy;
  int requiredEnergy = nodesToEnergy[endNode];

  if (path.empty() || requiredEnergy > globalEnergy) {
    std::cout << "\nRobot gagal dalam mencapai tujuan :(\n";
    return 0;
  }

  // Normal output
  std::cout << "\nTotal energi minimum: " << nodesToEnergy[endNode] << "\n";

  std::cout << "Jalur: ";
  for (size_t i = 0; i < path.size(); i++) {
    std::cout << path[i]->name;
    if (i + 1 < path.size())
      std::cout << " -> ";
  }
  std::cout << "\n";

  std::cout << "Waktu tiba:\n";
  for (auto *n : path) {
    std::cout << n->name << " (menit " << nodesToTime[n] - globalTime << ")\n";
  }

  return 0;
}
