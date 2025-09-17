#include <iostream>
#include <vector>
#include <unordered_map>

/*
        ========== DATA AND MODELS =============
*/
struct Node;

struct Edge
{
    int length;
    int obs;
    double energyNeeded;
    std::vector<Node *> connectedNodes;

    Edge(int paramLength, int paramObs)
    {
        length = paramLength;
        obs = paramObs;
        energyNeeded = length + obs;
    }

    void TravelEdge(int globalTime)
    {
        (globalTime % 2 == 0) ? energyNeeded *= 0.8 : energyNeeded *= 1.3;
    }
};

enum NodeType
{
    DEFAULT,
    REST,
    ELECTRICAL,
    MECHANIC,
    CHARGING,
};

struct Node
{
    std::string name;
    enum NodeType type;
    std::vector<Edge *> connectedEdges;

    void Charge(int *globalEnergyPTR, int maxEnergy)
    {
        if (type == NodeType::CHARGING)
        {
            *globalEnergyPTR = maxEnergy;
        }
    }
};

/*
        ============= HELPERS ==================
*/

Node *GetorCreateNode(std::unordered_map<std::string, Node *> nodeMap, std::string name, NodeType type = NodeType::DEFAULT)
{
    if (nodeMap.find(name) == nodeMap.end())
    {
        nodeMap[name] = new Node{name, type, {}};
    }
    return nodeMap[name];
}

/*
        ============== CORE FUNCTIONS ===================
*/

void GetInputs(int *nodeCount, int *edgeCount, std::vector<Edge> *globalEdges, std::vector<Node> *globalNodes)
{
    std::unordered_map<std::string, Node *> nodeMap;

    std::cout << "Jumlah node (space) jumlah edge: ";
    std::cin >> *nodeCount >> *edgeCount;

    std::cout << std::endl
              << "==== SET UP EDGE ====" << std::endl
              << "format: node1 node2 panjangEdge obstacleEdge" << std::endl;

    for (int i = 0; i < *edgeCount; i++)
    {
        std::string nodeName1, nodeName2;
        int edgeLength, obs;

        std::cout << "Edge ke-" << i << ": ";
        std::cin >> nodeName1 >> nodeName2 >> edgeLength >> obs;
        std::cout << std::endl;

        // Create Nodes
        Node *n1 = GetorCreateNode(nodeMap, nodeName1);
        Node *n2 = GetorCreateNode(nodeMap, nodeName2);

        // Create edge
        globalEdges->emplace_back(edgeLength, obs);
        Edge *edgePtr = &globalEdges->back();

        // Link edge <-> globalNodes
        edgePtr->connectedNodes.push_back(n1);
        edgePtr->connectedNodes.push_back(n2);

        n1->connectedEdges.push_back(edgePtr);
        n2->connectedEdges.push_back(edgePtr);
    }
    std::cout << "Edge setup finished!" << std::endl;

    return;
}

int main()
{
    // MAP
    int nodeCount, edgeCount;
    std::vector<Edge> edgeObjects;
    std::vector<Node> nodeObjects;

    // DATA
    int globalTime, globalEnergy;
    const int maxEnergy = 1000;

    GetInputs(&nodeCount, &edgeCount, &edgeObjects, &nodeObjects);

    return 0;
}
