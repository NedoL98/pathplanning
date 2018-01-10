#ifndef ASTAR_H
#define ASTAR_H
#include "isearch.h"

//A* search.
class Astar : public ISearch
{
    public:
        Astar(double HW, bool BT);
};

#endif
