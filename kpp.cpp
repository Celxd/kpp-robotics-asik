#include <iostream>
#include <ostream>
#include <unordered_map>
#include <vector>
#include <deque>

/*
        ========== DATA AND MODELS =============
*/
struct Node;

struct Edge {
  int length;
  int obs;
  double energyNeeded;
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
};

/*
        ============= HELPERS ==================
*/

Node *GetorCreateNode(std::unordered_map<std::string, Node *> &nodeMap,
                      std::deque<Node> &globalNodes, const std::string name,
                      NodeType type = NodeType::DEFAULT) {
  if (nodeMap.find(name) == nodeMap.end()) {
    // Construct node inside globalNodes
    globalNodes.push_back(Node{name, type, {}});
    Node *nodePtr =
        &globalNodes.back(); // get pointer to the node in the vector
    nodeMap[name] = nodePtr;
  }
  return nodeMap[name];
}

/*
        ============== CORE FUNCTIONS ===================
*/

void STNodeInput(std::unordered_map<std::string, Node *> &nodeMap,
                 Node *&startNode,
                 Node *&endNode) {
  std::string startNodeName, endNodeName;
  int jamMulai;

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

void GetInputs(int *nodeCount, int *edgeCount, std::deque<Edge> *globalEdges,
               std::deque<Node> *globalNodes, Node *&startNode,
               Node *&endNode) {
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
  // TODO: kerjain sisa input input

  return;
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
            endNode);

  return 0;
}
