#include "isearch.h"
#include <set>
#include <time.h>

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}

bool canVisit(int x, int y, int dx, int dy, const Map &map, const EnvironmentOptions &options)
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

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    clock_t time_start = clock();

    std::vector<std::vector<int>> distance(map.getMapHeight(), std::vector<int>(map.getMapWidth(), -1));
    auto compareByDistance = [&](std::pair<int, int> v1, std::pair<int, int> v2) //Lambda-function as a std::set comparator
    {
        return distance[v1.first][v1.second] < distance[v2.first][v2.second];
    };

    // std::cout << map.getValue(map.getStartingPoint().first, map.getStartingPoint().second) << std::endl;
    // std::cout << map.getValue(map.getGoalPoint().first, map.getGoalPoint().second) << std::endl;

    std::vector<std::pair<int, int>> visited;
    std::multiset<std::pair<int, int>, decltype(compareByDistance)> current_queue(compareByDistance);

    distance[map.getStartingPoint().first][map.getStartingPoint().second] = 0;
    current_queue.insert(map.getStartingPoint());

    int step_counter = 0;

    while (!current_queue.empty())
    {
        std::pair<int, int> v = *current_queue.begin(); //Get the nearest node
        /*
        std::cout << "Current node: " << std::endl;
        std::cout << v.first << " " << v.second << std::endl;
        std::cout << "Current distance: " << std::endl;
        std::cout << distance[v.first][v.second] << std::endl;
        */
        current_queue.erase(current_queue.begin()); //Erase it from queue
        visited.push_back(v); //Mark it as a visited vertex

        if (v.first == map.getGoalPoint().first and
            v.second == map.getGoalPoint().second) //Goalpoint reached
        {
            break;
        }

        for (int dx = -1; dx <= 1; ++dx) //Searching for a new node
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                //Checking whether it is possible to visit new vertex

                if (canVisit(v.first, v.second, dx, dy, map, options))
                {
                    ++step_counter;
                    //Checking whether we have to visit new vertex
                    if (distance[v.first + dx][v.second + dy] == -1 or
                        distance[v.first + dx][v.second + dy] > distance[v.first][v.second] + 1)
                    {
                        current_queue.erase({v.first + dx, v.second + dy});
                        distance[v.first + dx][v.second + dy] = distance[v.first][v.second] + 1;
                        current_queue.insert({v.first + dx, v.second + dy});
                    }
                }
            }
        }
    }

    clock_t time_finish = clock();

    std::cout << distance[map.getGoalPoint().first][map.getGoalPoint().second] << std::endl;

    sresult.pathfound = (distance[map.getGoalPoint().first][map.getGoalPoint().second] != -1);
    sresult.nodescreated = visited.size();
    sresult.numberofsteps = step_counter;
    sresult.time = static_cast<double>(time_finish) - static_cast<double>(time_start);
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

/*std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    std::list<Node> successors;
    //need to implement
    return successors;
}*/

/*void ISearch::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void ISearch::makeSecondaryPath()
{
    //need to implement
}*/
