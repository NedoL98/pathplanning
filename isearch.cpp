#include "isearch.h"
#include <map>
#include <set>
#include <time.h>

const double EPS = 1e-7;

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}

double l_diff(double dx, double dy);

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    clock_t time_start = clock();

    //Lambda-function as a std::set comparator
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

        auto successors = findSuccessors(curNode, map, options);

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

    std::cout << "Search finished!" << std::endl;
    //Restoring the path
    auto curNode_ptr = closed.begin();
    while (curNode_ptr != closed.end() and *curNode_ptr != goalNode) {
        ++curNode_ptr;
    }
    if (curNode_ptr != closed.end()) {
        lppath.push_front(*curNode_ptr);
        while (*curNode_ptr != startingNode) {
            --curNode_ptr;

            // std::cout << curNode_ptr->i << " " << curNode_ptr->j << std::endl;
            /*
            if (l_diff(lppath.back().i - curNode_ptr->i, lppath.back().j - curNode_ptr->j) <= 2) {
                std::cout << curNode_ptr->g << std::endl;
                std::cout << l_diff(lppath.back().i - curNode_ptr->i, lppath.back().j - curNode_ptr->j) << std::endl;
                std::cout << lppath.back().g << std::endl;
                std::cout << std::endl;
            }
            */
            if (fabs(curNode_ptr->g + l_diff(lppath.back().i - curNode_ptr->i,
                                             lppath.back().j - curNode_ptr->j) - lppath.back().g) < EPS) {
                lppath.push_back(*curNode_ptr);
            }
        }
    }


    //No need to find lpath, because it's the same as hpath

    // std::cout << hppath.size() << std::endl;

    clock_t time_finish = clock();

    // std::cout << distance[map.getGoalPoint().first][map.getGoalPoint().second] << std::endl;

    sresult.pathfound = (curNode_ptr != closed.end());
    sresult.nodescreated = closed.size() + open.size();
    sresult.numberofsteps = step_counter;
    sresult.time = static_cast<double>(time_finish) - static_cast<double>(time_start);
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

bool check(int x, int y, int dx, int dy, const Map &map, const EnvironmentOptions &options)
{
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

std::list<Node> ISearch::findSuccessors(Node &curNode, const Map &map, const EnvironmentOptions &options)
{
    std::list<Node> successors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (check(curNode.i, curNode.j, dx, dy, map, options)) {
                successors.push_back(Node(curNode.i + dx, curNode.j + dy, curNode.g + l_diff(dx, dy)));
            }
        }
    }
    return successors;
}

/*void ISearch::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void ISearch::makeSecondaryPath()
{
    //need to implement
}*/
