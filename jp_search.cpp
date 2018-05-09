#include "jp_search.h"

JP_Search::~JP_Search()
{
}

Node *jump(const Node *curNode, int dx, int dy, const Map &map, const EnvironmentOptions &options) {
    //Distance calculation is implemented in another function
    Node *nextNode = new Node(curNode->i + dx, curNode->j + dy);

    //If can't go
    if (map.getValue(nextNode->i, nextNode->j) != 0) {
        return nullptr;
    }

    //GoalPoint
    if (map.getGoalPoint().first == nextNode->i and
            map.getGoalPoint().second == nextNode->j) {
        return nextNode;
    }

    //Zero movement case
    if (dx == 0 and dy == 0) {
        return nullptr;
    }

    /*
    //Allowdiagonal
    if (dx != 0 and dy != 0 and !options.allowdiagonal) {
        return nullptr;
    }
    */

    if (dx != 0 and dy != 0) {
        int obstaclesCount = (map.getValue(curNode->i + dx, curNode->j) == 1) +
                (map.getValue(curNode->i, curNode->j + dy) == 1);
        if (obstaclesCount >= 1 and !options.cutcorners) {
            return nullptr;
        }
        if (obstaclesCount == 2 and !options.allowsqueeze) {
            return nullptr;
        }
    }



    //Diagonal case
    if (dx != 0 and dy != 0) {
        //Checking enforced neighbours
        if (map.getValue(nextNode->i - dx, nextNode->j) == 1) {
            return nextNode;
        }
        if (map.getValue(nextNode->i, nextNode->j - dy) == 1) {
            return nextNode;
        }

        //Trying to expand horizontally or vertically
        if (jump(nextNode, dx, 0, map, options) != nullptr or
                jump(nextNode, 0, dy, map, options) != nullptr) {
            return nextNode;
        }
    }
    //Non-diagonal case
    else {
        //Horizontal case
        if (dx != 0) {
            if (map.getValue(curNode->i, curNode->j - 1) == 1) {
                return nextNode;
            }
            if (map.getValue(curNode->i, curNode->j + 1) == 1) {
                return nextNode;
            }
        }
        //Vertical case
        else {
            if (map.getValue(curNode->i - 1, curNode->j) == 1) {
                return nextNode;
            }
            if (map.getValue(curNode->i + 1, curNode->j) == 1) {
                return nextNode;
            }
        }
    }

    return jump(nextNode, dx, dy, map, options);
}

std::list<Node> JP_Search::findSuccessors(const Node &curNode, const Map &map, const EnvironmentOptions &options) {
    std::list<Node> successors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            Node *jumpPoint = jump(&curNode, dx, dy, map, options);
            if (jumpPoint != nullptr) {
                successors.push_back(*jumpPoint);
            }
        }
    }
    return successors;
}

void JP_Search::makePrimaryPath(Node curNode) {
    while (true) {
        hppath.push_front(curNode);
        if (curNode.parent != nullptr) {
            curNode = *curNode.parent;
        } else {
            break;
        }
    }
}

void JP_Search::makeSecondaryPath() {
    auto nodePtr = hppath.begin();
    std::pair<int, int> previousDifference({0, 0});

    while (nodePtr != hppath.end()) {
        auto nextPtr = nodePtr;
        ++nextPtr;
        if (nextPtr == hppath.end()) {
            lppath.push_back(*nodePtr);
        } else {
            int dx = nextPtr->i - nodePtr->i;
            int dy = nextPtr->j - nodePtr->j;
            if (dx != 0) {
                dx = dx / abs(dx);
            }
            if (dy != 0) {
                dy = dy / abs(dy);
            }

            int x = nodePtr->i;
            int y = nodePtr->j;

            while (x != nextPtr->i or y != nextPtr->j) {
                lppath.push_back(Node(x, y));
                x += dx;
                y += dy;
            }
        }
        ++nodePtr;
    }
}
