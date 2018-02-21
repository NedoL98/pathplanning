#include "theta.h"

Theta::~Theta()
{
}

bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const Map &map, bool cutcorners) {
    if (i2 < i1) {
        std::swap(i1, i2);
    }
    if (j2 < j1) {
        std::swap(j1, j2);
    }
    if (j2 - j1 > i2 - i1) {
        std::swap(j1, i1);
        std::swap(j2, i2);
    }

    int dx = i2 - i1;
    int dy = j2 - j1;
    int rm = 0;
    int j = j1;
    //Now i2 >= i1 && j2 >= j1 && i2 - i1 >= j2 - j1
    for (int i = i1; i <= i2; ++i) {
        rm += dy;
        if (rm >= dx) {
            if (map.getValue(i, j) == 1) {
                return false;
            }
            ++j;
            rm -= dx;
        }
        if (map.getValue(i, j) == 1) {
            return false;
        }
    }
    return true;
}

Node Theta::resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options) {
    //need to implement
    return current;
}

void Theta::makeSecondaryPath() {
}

void Theta::makePrimaryPath(Node curNode) {
    //need to implement
}
