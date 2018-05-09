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

    if (map.getGoalPoint().first == nextNode->i and
            map.getGoalPoint().second == nextNode->j) {
        return nextNode;
    }

    //Diagonal case
    if (dx != 0 and dy != 0) {
        //Checking enforced neighbours
        if (map.getValue(nextNode->i - dx, nextNode->j) == 1) {
            nextNode->i -= dx;
            return nextNode;
        }
        if (map.getValue(nextNode->i, nextNode->j - dy) == 1) {
            nextNode->j -= dy;
            return nextNode;
        }

        //Trying to expand horizontally or vertically
        Node *ret = jump(nextNode, dx, 0, map, options);
        if (ret != nullptr) {
            return ret;
        }
        ret = jump(nextNode, 0, dy, map, options);
        if (ret != nullptr) {
            return ret;
        }
    }
    //Non-diagonal case
    else {
        //Horizontal case
        if (dx != 0) {
            if (map.getValue(curNode->i, curNode->j - 1) == 1) {
                nextNode->j = curNode->j - 1;
                return nextNode;
            }
            if (map.getValue(curNode->i, curNode->j + 1) == 1) {
                nextNode->j = curNode->j + 1;
                return nextNode;
            }
        }
        //Vertical case
        else {
            if (map.getValue(curNode->i - 1, curNode->j) == 1) {
                nextNode->i = curNode->i - 1;
                return nextNode;
            }
            if (map.getValue(curNode->i + 1, curNode->j) == 1) {
                nextNode->i = curNode->i + 1;
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
            Node *jump_point = jump(&curNode, dx, dy, map, options);
            if (jump_point != nullptr) {
                successors.push_back(*jump_point);
            }
        }
    }
    return successors;
}

void JP_Search::makePrimaryPath(Node curNode) {
    //need to implement
}

void JP_Search::makeSecondaryPath() {
    //need to implement
}
