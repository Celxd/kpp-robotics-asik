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
#include <utility>
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

  void TravelEdge(int globalTime) {
    (globalTime % 2 == 0) ? energyNeeded *= 0.8 : energyNeeded *= 1.3;
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

  void Charge(int *globalEnergyPTR, int maxEnergy) {
    if (type == NodeType::CHARGING) {
      *globalEnergyPTR = maxEnergy;
    }
  }

  void Rest(int *globalTime) {
    if (type == REST) {
      *globalTime += 1;
    }
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
    Node *nodePtr =
        &globalNodes.back(); // get pointer to the node in the vector
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
               int *globalTime) {
  std::unordered_map<std::string, Node *> nodeMap;

  std::cout << "*Jumlah node* *jumlah edge*: ";
  std::cin >> *nodeCount >> *edgeCount;

  std::cout << std::endl
            << "==== SET UP EDGE ====" << std::endl
            << "format: node1 node2 panjangEdge obstacleEdge" << std::endl;

  for (int i = 0; i < *edgeCount; i++) {
    std::string nodeName1, nodeName2;
    int edgeLength, obs;

    std::cout << "Edge ke-" << i << ": ";
    std::cin >> nodeName1 >> nodeName2 >> edgeLength >> obs;
    std::cout << std::endl;

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
  std::cout << "Edge setup finished!" << std::endl
            << "Setup tipe node dimulai!" << std::endl;

  STNodeInput(nodeMap, startNode, endNode);
  NodeTypeInput(nodeMap, "Rest", REST);
  NodeTypeInput(nodeMap, "Charging", CHARGING);
  NodeTypeInput(nodeMap, "Mechanic", MECHANIC);
  NodeTypeInput(nodeMap, "Electrical", ELECTRICAL);

  int awalJam;
  std::cout << "Jam awal perjalanan (dalam satuan jam): ";
  std::cin >> awalJam;
  *globalTime = awalJam * 60;

  return;
}

// TODO: dijkstra algorithm wtf am i doing actually wth

std::vector<Node *> Dijkstra(std::deque<Node> *globalNodes, Node *startNode,
                             Node *endNode,
                             std::unordered_map<Node *, int> &nodesToEnergy,
                             std::unordered_map<Node *, int> &nodesToTime) {
  std::unordered_map<Node *, Node *> nodesToPrev;
  std::priority_queue<std::pair<int, Node *>,
                      std::vector<std::pair<int, Node *>>,
                      std::greater<std::pair<int, Node *>>>
      toVisitNodes;

  for (Node &currentNode : *globalNodes) {
    nodesToEnergy[&currentNode] = std::numeric_limits<int>::max();
    nodesToPrev[&currentNode] = nullptr;
    nodesToTime[&currentNode] = 0;
  }

  nodesToEnergy[startNode] = 0;
  nodesToPrev[startNode] = nullptr;
  toVisitNodes.push(std::make_pair(0, startNode));

  while (!toVisitNodes.empty()) {
    int spentEnergy = toVisitNodes.top().first;
    Node *currentNode = toVisitNodes.top().second;

    toVisitNodes.pop();

    if (spentEnergy > nodesToEnergy[currentNode])
      continue;
    if (currentNode == endNode)
      continue;

    for (auto currentEdge : currentNode->connectedEdges) {
      Node *neighborNode = (currentEdge->connectedNodes[0] == currentNode)
                               ? currentEdge->connectedNodes[1]
                               : currentEdge->connectedNodes[0];

      int newEnergy = spentEnergy + currentEdge->energyNeeded;
      int newTime = 2 + nodesToTime[currentNode];

      if (newEnergy < nodesToEnergy[neighborNode]) {
        nodesToEnergy[neighborNode] = newEnergy;
        nodesToPrev[neighborNode] = currentNode;
        nodesToTime[neighborNode] = newTime;

        toVisitNodes.push(std::make_pair(newEnergy, neighborNode));
      }
    }
  }

  // If unreachable
  if (nodesToEnergy[endNode] == std::numeric_limits<int>::max()) {
    return {}; // empty path
  }

  std::vector<Node *> path;

  for (Node *iteratorLocation = endNode; iteratorLocation != nullptr;
       iteratorLocation = nodesToPrev[iteratorLocation]) {
    path.push_back(iteratorLocation);
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
  int globalTime, globalEnergy;
  const int maxEnergy = 1000;

  GetInputs(&nodeCount, &edgeCount, &edgeObjects, &nodeObjects, startNode,
            endNode, &globalTime);

  PrintSettings(nodeObjects, edgeObjects, startNode, endNode, globalTime);

  // Run Dijkstra
  std::unordered_map<Node *, int> nodesToEnergy, nodesToTime;
  std::vector<Node *> path =
      Dijkstra(&nodeObjects, startNode, endNode, nodesToEnergy, nodesToTime);

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
    std::cout << n->name << " (menit " << nodesToTime[n] << ")\n";
  }

  return 0;
}
