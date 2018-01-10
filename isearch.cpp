#include "isearch.h"
#include <map>
#include <set>
#include <time.h>
#include <utility>

ISearch::ISearch()
{
    hweight = 0;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options) {
    clock_t time_start = clock();

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
        return compareByCell(v1, v2);
    };

    std::multiset<Node, decltype(compareByDistance)> open(compareByDistance);
    std::multiset<Node, decltype(compareByCell)> closed(compareByCell);

    //Auxilary structure, that helps keeping only one copy of node in OPEN
    std::multiset<Node, decltype(compareByCell)> is_open(compareByCell);

    Node goalNode = Node(map.getGoalPoint().first, map.getGoalPoint().second);
    Node startingNode = Node(map.getStartingPoint().first, map.getStartingPoint().second);

    open.insert(startingNode);
    is_open.insert(startingNode);

    int step_counter = 0;

    // std::cout << "Here\n" << std::endl;

    while (!open.empty())
    {
        ++step_counter;
        Node curNode = *open.begin(); //Get the nearest node
        /*
        std::cout << "Current node: " << std::endl;
        std::cout << curNode.i << " " << curNode.j << std::endl;
        std::cout << "Current distance: " << std::endl;
        std::cout << curNode.g << std::endl;
        */
        open.erase(open.begin()); //Erase it from queue

        closed.insert(curNode); //Mark it as a visited vertex

        //Pointer to an ancestor
        auto curNode_ptr = closed.find(curNode);
        const Node *ancestor_ptr = &(*curNode_ptr);

        if (curNode.i == map.getGoalPoint().first and
            curNode.j == map.getGoalPoint().second) //Goalpoint reached
        {
            break;
        }

        //It's sufficient to take the one from ''distance'' structure
        //Because otherwise the element will be deleted
        auto successors = findSuccessors(*curNode_ptr, map, options);

        for (auto nextNode : successors) { //Inserting new nodes
            //If it's not visited yet or if new distance is better
            nextNode.parent = ancestor_ptr;

            auto nodeCopyIter = is_open.find(nextNode);

            if (nodeCopyIter == is_open.end() or
                nodeCopyIter->F > nextNode.F) {

                //Erase the node, if already in open
                if (nodeCopyIter != is_open.end()) {
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
    if (closed.count(goalNode)) {
        makePrimaryPath(*closed.find(goalNode));
    }

    clock_t time_finish = clock();

    //Making secondary path
    makeSecondaryPath();

    sresult.pathfound = (closed.count(goalNode));
    if (sresult.pathfound) {
        sresult.pathlength = closed.find(goalNode)->g;
    }
    sresult.nodescreated = closed.size() + open.size();
    sresult.numberofsteps = step_counter;
    sresult.time = static_cast<double>(time_finish) - static_cast<double>(time_start);
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

bool check(int x, int y, int dx, int dy, const Map &map, const EnvironmentOptions &options) {
    if (dx == 0 and dy == 0) //Zero movement case
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
    int dx = abs(i1 - i2);
    int dy = abs(j1 - j2);
    switch (options.metrictype) {
        case CN_SP_MT_MANH:
            return dx + dy;
        case CN_SP_MT_EUCL:
            return sqrt(dx * dx + dy * dy);
        case CN_SP_MT_CHEB:
            return std::max(dx, dy);
        case CN_SP_MT_DIAG:
            return dx + dy - std::min(dx, dy) * sqrt(2);
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
                newNode.g = curNode.g + computeHFromCellToCell(curNode.i, curNode.j, newNode.i, newNode.j, options);
                newNode.H = computeHFromCellToCell(newNode.i, newNode.j, xEnd, yEnd, options);
                newNode.F = newNode.g + hweight * newNode.H;
                successors.push_back(newNode);
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode) {
    while (true) {
        lppath.push_back(curNode);
        if (curNode.parent != nullptr) {
            curNode = *curNode.parent;
        } else {
            break;
        }
    }
}

std::pair<int, int> getDifference(Node &curNode, Node &nextNode) {
    return {nextNode.i - curNode.i, nextNode.j - curNode.j};
}

void ISearch::makeSecondaryPath() {
    auto nodePtr = lppath.begin();
    std::pair<int, int> previousDifference({0, 0}); //Probably better to make a bool flag

    while (nodePtr != lppath.end()) {
        auto nextPtr = nodePtr;
        ++nextPtr; //Not really a good codestyle either
        if (nextPtr == lppath.end()) {
            hppath.push_front(*nodePtr);
        } else {
            auto newDifference = getDifference(*nodePtr, *nextPtr);
            if (previousDifference != newDifference) {
                hppath.push_front(*nodePtr);
                previousDifference = newDifference;
            }
        }
        ++nodePtr;
    }
}
