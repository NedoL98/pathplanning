#include "isearch.h"
#include <algorithm>
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
    /*
    std::cout << "DIAGONAL: " << options.allowdiagonal << std::endl;
    std::cout << "CORNERS: " << options.cutcorners << std::endl;
    std::cout << "SQUEEZE: " << options.allowsqueeze << std::endl;
    */

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    //Lambda-functions as a comparator
    auto compareByCell = [](Node v1, Node v2) {
        if (v1.i != v2.i) {
            return v1.i < v2.i;
        } else if (v1.j != v2.j) {
            return v1.j < v2.j;
        }
        return v1.k < v2.k;
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

    Node goalNode = Node(map.getGoalPoint());
    Node startingNode = Node(map.getStartingPoint());

    //std::cout << startingNode.i << " " << startingNode.j << " " << startingNode.k << " " << std::endl;
    //std::cout << goalNode.i << " " << goalNode.j << " " << goalNode.k << " " << std::endl;

    open.insert(startingNode);
    is_open.insert(startingNode);

    int step_counter = 0;
    bool path_found = false;

    double prev_dist = 0;

    while (!open.empty())
    {
        ++step_counter;
        Node curNode = *open.begin(); //Get the nearest node

        open.erase(open.begin()); //Erase it from queue

        closed.push_back(curNode); //Mark it as a visited vertex

        if (prev_dist < curNode.g) {
            prev_dist = curNode.g;
            //curNode.print();
        }

        //Goalpoint reached
        if (curNode == goalNode) {
            path_found = true;
            break;
        }

        //Pointer to an ancestor
        auto curNode_ptr = --closed.end();
        const Node *ancestor_ptr = &(*curNode_ptr);

        //Finding successors for current node
        auto successors = findSuccessors(curNode, map, options);

        for (auto nextNode : successors) { //Inserting new nodes
            //If it's not visited yet or if new distance is better
            nextNode.parent = ancestor_ptr;

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

bool ISearch::check(const Node &curNode, std::vector<int> dm, const Map &map, const EnvironmentOptions &options) {
    //Zero movement case
    if (dm[0] == 0 and dm[1] == 0 and dm[2] == 0) {
        return false;
    }

    //Out of bounds
    if (curNode.i + dm[0] < 0 or curNode.i + dm[0] >= map.getMapWidth() or
        curNode.j + dm[1] < 0 or curNode.j + dm[1] >= map.getMapHeight()) {
        return false;
    }

    //Checking if possible to fly over an obstacle
    if (map.getValue(curNode.i + dm[0], curNode.j + dm[1]) > curNode.k + dm[2]) {
        return false;
    }

    //Checking if underneath the boundary
    if (map.getMapMaxAlt() != -1 and curNode.k + dm[2] > map.getMapMaxAlt()) {
        return false;
    }

    //Check diagonal moves
    if (!options.allowdiagonal and std::count(dm.begin(), dm.end(), 0) != static_cast<int>(dm.size()) - 1) {
        return false;
    }

    Node corner;
    std::vector<int> cur_coord = {curNode.i + dm[0], curNode.j + dm[1], curNode.k + dm[2]};
    for (int i = 0; i < static_cast<int>(dm.size()); ++i) {
        for (int j = 0; j < static_cast<int>(dm.size()); ++j) {
            if (i == j or dm[i] == 0 or dm[j] == 0) {
                continue;
            }
            //dm[i] != 0 and dm[j] != 0
            int cnt = 0;

            cur_coord[i] -= dm[i];
            corner = Node(cur_coord);
            cur_coord[i] += dm[i];
            cnt += (map.getValue(corner.i, corner.j) > corner.k);

            cur_coord[j] -= dm[j];
            corner = Node(cur_coord);
            cur_coord[j] += dm[j];
            cnt += (map.getValue(corner.i, corner.j) > corner.k);

            if (!options.cutcorners and cnt > 0) {
                return false;
            }

            if (!options.allowsqueeze and cnt > 1) {
                return false;
            }
        }
    }

    return true;
}

double ISearch::computeHFromCellToCell(const Node &from, const Node &to, const EnvironmentOptions &options) {
    std::vector<int> dm = {abs(from.i - to.i), abs(from.j - to.j), abs(from.k - to.k)};
    switch (options.metrictype) {
        case CN_SP_MT_MANH:
            return std::accumulate(dm.begin(), dm.end(), 0);
        case CN_SP_MT_DIAG: {
            double res = 0;
            for (int i = 3; i >= 1; --i) {
                double mn = *min_element(dm.begin(), dm.end());
                for (auto &c : dm) {
                    c -= mn;
                }
                res += mn * sqrt(i);
            }
            return res;
        }
        case CN_SP_MT_CHEB:
            return *max_element(dm.begin(), dm.end());
        default: //CN_SP_MT_EUCL
            return sqrt(std::inner_product(dm.begin(), dm.end(), dm.begin(), 0));
    }
}

bool ISearch::compareByCoordinate(const Node &node1, const Node &node2) {
    if (node1.i != node2.i) {
        return node1.i < node2.i;
    } else if (node1.j != node2.j) {
        return node1.j < node2.j;
    }
    return node1.k < node2.k;
}

bool ISearch::breakTie(const Node &node1, const Node &node2) {
    if (breakingties == CN_SP_BT_GMAX) {
        if (node1.g == node2.g) {
            return compareByCoordinate(node1, node2);
        }
        return node1.g > node2.g;
    } else { //breakingties == CN_SP_BT_GMIN
        if (node1.g == node2.g) {
            return compareByCoordinate(node1, node2);
        }
        return node1.g < node2.g;
    }
}

std::list<Node> ISearch::findSuccessors(const Node &curNode, const Map &map, const EnvironmentOptions &options) {
    std::list<Node> successors;
    Node goalNode(map.getGoalPoint());
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (check(curNode, {dx, dy, dz}, map, options)) {
                    Node newNode;
                    newNode.i = curNode.i + dx;
                    newNode.j = curNode.j + dy;
                    newNode.k = curNode.k + dz;
                    newNode.g = curNode.g + computeHFromCellToCell(curNode, newNode, options);
                    newNode.H = computeHFromCellToCell(newNode, goalNode, options);
                    newNode.F = newNode.g + hweight * newNode.H;
                    successors.push_back(newNode);
                }
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

std::vector<int> ISearch::getDifference(Node &curNode, Node &nextNode) {
    std::vector<int> ret;
    ret.push_back(nextNode.i - curNode.i);
    ret.push_back(nextNode.j - curNode.j);
    ret.push_back(nextNode.k - curNode.k);
    return ret;
}

void ISearch::makeSecondaryPath() {
    auto nodePtr = lppath.begin();
    std::vector<int> previousDifference(3, 0);

    while (nodePtr != lppath.end()) {
        auto nextPtr = nodePtr;
        ++nextPtr; //Not really a good codestyle either
        if (nextPtr == lppath.end()) {
            hppath.push_back(*nodePtr);
        } else {
            auto newDifference = getDifference(*nodePtr, *nextPtr);
            bool same = true;
            for (int i = 0; i < 3; ++i) {
                if (previousDifference[i] != newDifference[i]) {
                    same = false;
                }
            }
            if (!same) {
                hppath.push_back(*nodePtr);
                previousDifference = newDifference;
            }
        }
        ++nodePtr;
    }
}
