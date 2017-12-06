#include "isearch.h"
#include <map>
#include <set>
#include <time.h>
#include <utility>

const double EPS = 1e-7;

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}

double l_diff(double dx, double dy);

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
        if (v1.g != v2.g) {
            return v1.g < v2.g;
        }
        return compareByCell(v1, v2);
    };

    std::multiset<Node, decltype(compareByDistance)> open(compareByDistance);
    std::multiset<Node, decltype(compareByDistance)> closed(compareByDistance);

    std::set<Node, decltype(compareByCell)> distance(compareByCell); //Only visited nodes are stored here

    Node goalNode = Node(map.getGoalPoint().first, map.getGoalPoint().second);
    Node startingNode = Node(map.getStartingPoint().first, map.getStartingPoint().second);

    open.insert(startingNode);
    distance.insert(startingNode);

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

        if (curNode.i == map.getGoalPoint().first and
            curNode.j == map.getGoalPoint().second) //Goalpoint reached
        {
            break;
        }

        //It's sufficient to take the one from ''distance'' structure
        //Because otherwise the element will be deleted
        auto successors = findSuccessors(*distance.find(curNode), map, options);

        for (auto nextNode : successors) { //Inserting new nodes
            //If it's not visited yet or if new distance is better
            auto it = distance.find(nextNode);
            if (it == distance.end() or it->g > nextNode.g) {
                if (it != distance.end()) {
                    open.erase(*it); //We should delete this, because distance may be updated
                }
                open.insert(nextNode);
                distance.insert(nextNode); //No need to delete this, because compared by cell only, g- doesn't matter
            }
        }
    }

    //Making primary path
    if (distance.find(goalNode) != distance.end()) {
        makePrimaryPath(*distance.find(goalNode));
    }

    //Making secondary path
    makeSecondaryPath();

    clock_t time_finish = clock();

    sresult.pathfound = (distance.find(goalNode) != distance.end());
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

double l_diff(double dx, double dy) {
    return sqrt(dx * dx + dy * dy);
}

std::list<Node> ISearch::findSuccessors(const Node &curNode, const Map &map, const EnvironmentOptions &options) {
    std::list<Node> successors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (check(curNode.i, curNode.j, dx, dy, map, options)) {
                successors.push_back(Node(curNode.i + dx, curNode.j + dy,
                                          curNode.g + l_diff(dx, dy), &curNode));
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
