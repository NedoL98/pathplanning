#include "isearch.h"
#include <chrono>
#include <ctime>
#include <map>
#include <ratio>
#include <set>
#include <utility>

ISearch::ISearch()
{
    hweight = 0;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options) {
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    //Lambda-functions as a comparator
    auto compareByCell = [](Node v1, Node v2) {
        if (v1.i != v2.i) {
            return v1.i < v2.i;
        }
        return v1.j < v2.j;
    };

    auto compareByDistance = [&](Node v1, Node v2) {
        if (v1.F != v2.F) {
            return v1.F < v2.F;
        }
        return breakTie(v1, v2);
    };

    std::set<Node, decltype(compareByDistance)> open(compareByDistance);
    std::list<Node> closed;

    //Auxilary structure, that helps keeping only one copy of node in OPEN
    std::set<Node, decltype(compareByCell)> is_open(compareByCell);

    Node goalNode = Node(map.getGoalPoint().first, map.getGoalPoint().second);
    Node startingNode = Node(map.getStartingPoint().first, map.getStartingPoint().second);

    open.insert(startingNode);
    is_open.insert(startingNode);

    int step_counter = 0;
    bool path_found = false;

    while (!open.empty())
    {
        ++step_counter;
        Node curNode = *open.begin(); //Get the nearest node

        open.erase(open.begin()); //Erase it from queue

        closed.push_back(curNode); //Mark it as a visited vertex

        if (curNode.i == map.getGoalPoint().first and
            curNode.j == map.getGoalPoint().second) //Goalpoint reached
        {
            path_found = true;
            break;
        }

        //Pointer to an ancestor
        std::list<Node>::iterator curNode_ptr = (--closed.end());
        Node *ancestor_ptr = (Node*)(&(*curNode_ptr));

        //Finding successors for current node
        auto successors = findSuccessors(curNode, map, options);

        for (auto nextNode : successors) { //Inserting new nodes
            //If it's not visited yet or if new distance is better

            //nextNode.parent = ancestor_ptr;
            nextNode.parent = getParent(&nextNode, ancestor_ptr, map, options);
            calculate_distances(nextNode, map, options);

            auto nodeCopyIter = is_open.find(nextNode);

            if (nodeCopyIter == is_open.end() or
                nodeCopyIter->F > nextNode.F) {

                //Erase the node, if already in open
                if (nodeCopyIter != is_open.end()) {
                    //Erasing the exact same node because of special compare function
                    //Which distinguishes different nodes not just by distance
                    //But by other parameters as well
                    open.erase(*nodeCopyIter);
                    is_open.erase(nodeCopyIter);
                }

                //Then insert the node
                open.insert(nextNode);
                is_open.insert(nextNode);
            }
        }
    }

    //Making primary path
    if (path_found) {
        //If path is found, then the last
        //node is goalNode
        makePrimaryPath(closed.back());
    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    //Making secondary path
    makeSecondaryPath();

    sresult.pathfound = path_found;
    if (sresult.pathfound) {
        sresult.pathlength = closed.back().g;
    }
    sresult.nodescreated = closed.size() + open.size();
    sresult.numberofsteps = step_counter;
    sresult.time = time_span.count();
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

bool ISearch::check(int x, int y, int dx, int dy, const Map &map, const EnvironmentOptions &options) {
    if (dx == 0 and dy == 0) //Zero movement case
    {
        return false;
    }
    //Out of bounds
    if (x + dx < 0 or x + dx >= map.getMapWidth() or
        y + dy < 0 or y + dy >= map.getMapHeight())
    {
        return false;
    }
    if (map.getValue(x + dx, y + dy) != 0) //New vertex must be free
    {
        return false;
    }
    if (dx != 0 and dy != 0)
    {
        if (!options.allowdiagonal)
        {
            return false;
        }
        int corners = map.getValue(x + dx, y) + map.getValue(x, y + dy); //Counting corners
        if (corners >= 1 and !options.cutcorners)
        {
            return false;
        }
        if (corners == 2 and !options.allowsqueeze)
        {
            return false;
        }
    }
    return true;
}

double ISearch::computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions &options) {
    double dx = abs(i1 - i2);
    double dy = abs(j1 - j2);
    switch (options.metrictype) {
        case CN_SP_MT_MANH:
            return dx + dy;
        case CN_SP_MT_EUCL:
            return sqrt(dx * dx + dy * dy);
        case CN_SP_MT_CHEB:
            return std::max(dx, dy);
        case CN_SP_MT_DIAG:
            return std::abs(dx - dy) + std::min(dx, dy) * sqrt(2.0);
    }
}

double ISearch::stepLength(const Node &node1, const Node &node2) {
    double dx = abs(node1.i - node2.i);
    double dy = abs(node1.j - node2.j);
    return sqrt(dx * dx + dy * dy);
}

bool ISearch::compareByDistance(const Node &node1, const Node &node2) {
    if (node1.i != node2.i) {
        return node1.i < node2.i;
    }
    return node1.j < node2.j;
}

bool ISearch::breakTie(const Node &node1, const Node &node2) {
    if (breakingties == CN_SP_BT_GMAX) {
        if (node1.g == node2.g) {
            return compareByDistance(node1, node2);
        }
        return node1.g > node2.g;
    } else if (breakingties == CN_SP_BT_GMIN) {
        if (node1.g == node2.g) {
            return compareByDistance(node1, node2);
        }
        return node1.g < node2.g;
    }
}

std::list<Node> ISearch::findSuccessors(const Node &curNode, const Map &map, const EnvironmentOptions &options) {
    std::list<Node> successors;
    int xEnd = map.getGoalPoint().first;
    int yEnd = map.getGoalPoint().second;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (check(curNode.i, curNode.j, dx, dy, map, options)) {
                Node newNode;
                newNode.i = curNode.i + dx;
                newNode.j = curNode.j + dy;
                //newNode.g = curNode.g + computeHFromCellToCell(curNode.i, curNode.j, newNode.i, newNode.j, options);
                //newNode.H = computeHFromCellToCell(newNode.i, newNode.j, xEnd, yEnd, options);
                //newNode.F = newNode.g + hweight * newNode.H;
                successors.push_back(newNode);
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode) {
    while (true) {
        lppath.push_front(curNode);
        if (curNode.parent != nullptr) {
            curNode = *curNode.parent;
        } else {
            break;
        }
    }
}

std::pair<int, int> ISearch::getDifference(Node &curNode, Node &nextNode) {
    return {nextNode.i - curNode.i, nextNode.j - curNode.j};
}

void ISearch::makeSecondaryPath() {
    if (lppath.empty()) {
        return;
    }
    auto nodePtr = lppath.begin();
    std::pair<int, int> previousDifference({0, 0}); //Probably better to make a bool flag

    while (nodePtr != lppath.end()) {
        auto nextPtr = nodePtr;
        ++nextPtr; //Not really a good codestyle either
        if (nextPtr == lppath.end()) {
            hppath.push_back(*nodePtr);
        } else {
            auto newDifference = getDifference(*nodePtr, *nextPtr);
            if (previousDifference != newDifference) {
                hppath.push_back(*nodePtr);
                previousDifference = newDifference;
            }
        }
        ++nodePtr;
    }
}

void ISearch::calculate_distances(Node &curNode, const Map &map, const EnvironmentOptions &options) {
    int xEnd = map.getGoalPoint().first;
    int yEnd = map.getGoalPoint().second;
    const Node *parent = curNode.parent;

    curNode.g = 0;
    curNode.H = computeHFromCellToCell(curNode.i, curNode.j, xEnd, yEnd, options);
    if (curNode.parent != nullptr) {
        curNode.g = curNode.parent->g + stepLength(curNode, *parent);
    }
    curNode.F = curNode.g + hweight * curNode.H;
}
