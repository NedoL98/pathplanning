#include "theta.h"

Theta::~Theta()
{
}

bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const Map *map, bool cutcorners, std::vector<Node> *path, bool to_fill) {
    //TODO: utilize cutcorners feature
    bool swapped = false;
    if (i2 < i1) {
        std::swap(i1, i2);
    }
    if (j2 < j1) {
        std::swap(j1, j2);
    }
    if (j2 - j1 > i2 - i1) {
        std::swap(j1, i1);
        std::swap(j2, i2);
        swapped = true;
    }

    int dx = i2 - i1;
    int dy = j2 - j1;
    int rm = 0;
    int j = j1;
    if (dx == 0) {
        return true;
    }
    //Now i2 >= i1 && j2 >= j1 && i2 - i1 >= j2 - j1
    for (int i = i1; i <= i2; ++i) {
        rm += dy;
        if (2 * rm >= dx) {
            if (to_fill) {
                if (swapped) {
                    path->push_back(Node(j, i));
                } else {
                    path->push_back(Node(i, j));
                }
            } else {
                if (map->getValue(i, j) == 1) {
                    return false;
                }
            }
            ++j;
            rm -= dx;
        }
        if (to_fill) {
            if (swapped) {
                path->push_back(Node(j, i));
            } else {
                path->push_back(Node(i, j));
            }
        } else {
            if (map->getValue(i, j) == 1) {
                return false;
            }
        }
    }
    return true;
}

const Node *Theta::getParent(Node *current, Node *parent, const Map &map, const EnvironmentOptions &options) {
    if (parent->parent == nullptr) {
        return parent;
    }
    const Node *parent_old = parent->parent;
    if (this->lineOfSight(current->i, current->j, parent_old->i, parent_old->j, &map, options.cutcorners)) {
        return parent_old;
    } else {
        return parent;
    }
}

void Theta::makeSecondaryPath() {
    auto nodePtr = hppath.begin();
    std::pair<int, int> previousDifference({0, 0});
    std::vector<Node> cur_path;

    while (nodePtr != hppath.end()) {
        auto nextPtr = nodePtr;
        ++nextPtr;
        if (nextPtr == hppath.end()) {
            lppath.push_back(*nodePtr);
        } else {
            cur_path.clear();
            this->lineOfSight(nodePtr->i, nodePtr->j, nextPtr->i, nextPtr->j, nullptr, false, &cur_path, true);
            for (auto curNode : cur_path) {
                if (curNode.i != nextPtr->i or curNode.j != nextPtr->j) {
                    lppath.push_back(curNode);
                }
            }
        }
        ++nodePtr;
    }
}

void Theta::makePrimaryPath(Node curNode) {
    while (true) {
        hppath.push_front(curNode);
        if (curNode.parent != nullptr) {
            curNode = *curNode.parent;
        } else {
            break;
        }
    }
}
