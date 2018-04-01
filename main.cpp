#include "mission.h"
#include <dirent.h>

void read_map(char *map_path) {
    Mission mission(map_path);
    std::cout<<"Parsing the map from XML:"<<std::endl;

    if(!mission.getMap()) {
        std::cout<<"Incorrect map! Program halted!"<<std::endl;
    }
    else {
        std::cout<<"Map OK!"<<std::endl<<"Parsing configurations (algorithm, log) from XML:"<<std::endl;
        if(!mission.getConfig())
            std::cout<<"Incorrect configurations! Program halted!"<<std::endl;
        else {
            std::cout<<"Configurations OK!"<<std::endl<<"Creating log channel:"<<std::endl;

            if(!mission.createLog())
                std::cout<<"Log chanel has not been created! Program halted!"<<std::endl;
            else {
                std::cout<<"Log OK!"<<std::endl<<"Start searching the path:"<<std::endl;

                mission.createEnvironmentOptions();
                mission.createSearch();
                mission.startSearch();

                std::cout<<"Search is finished!"<<std::endl;

                mission.printSearchResultsToConsole();
                mission.saveSearchResultsToLog();

                std::cout<<"Results are saved (if chosen) via created log channel."<<std::endl;
            }
        }
    }
}

const int MAX_BUF = 4096;

int main(int argc, char* argv[])
{
    std::cout << argc << std::endl;
    if(argc < 2) {
        std::cout<<"Error! Pathfinding task file (XML) is not specified!"<<std::endl;
        return 0;
    }

    std::cout<<argv[1]<<std::endl;

    DIR *dir = opendir(argv[1]);

    if (dir != NULL) {
        std::cout << "Directory detected" << std::endl;
        dirent *file;
        char buffer[MAX_BUF];
        while ((file = readdir(dir)) != NULL) {
            if (strcmp(file->d_name, ".") == 0 or strcmp(file->d_name, "..") == 0) {
                continue;
            }
            snprintf(buffer, MAX_BUF, "%s\\%s", argv[1], file->d_name);
            std::cout << buffer << std::endl;
            read_map(buffer);
            //std::cout << file->d_name << std::endl;
        }
    }
    else {
        read_map(argv[1]);
    }
}

