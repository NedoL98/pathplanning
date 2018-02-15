#include "node.h"

Node::Node(int x, int y, int z, double g_dist, double H_dist, const Node *ancestor) {
    i = x;
    j = y;
    k = z;
    g = g_dist;
    H = H_dist;
    F = g + H;
    parent = ancestor;
}

Node::Node(std::vector<int> coordinates, double g_dist, double H_dist, const Node *ancestor) {
    i = coordinates[0];
    j = coordinates[1];
    k = coordinates[2];
    g = g_dist;
    H = H_dist;
    F = g + H;
    parent = ancestor;
}

bool Node::operator== (const Node &other) const {
    return i == other.i and j == other.j and k == other.k;
}

bool Node::operator != (const Node &other) const {
    return !(*this == other);
}

void Node::print() const {
    std::cout << "Coordinates: " << i << " " << j << " " << k << std::endl;
    std::cout << "F value: " << F << std::endl;
    std::cout << "G value: " << g << std::endl;
    std::cout << "H value: " << H << std::endl;
}
