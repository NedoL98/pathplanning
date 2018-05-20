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
    if(argc < 2) {
        std::cout<<"Error! Pathfinding task file (XML) is not specified!"<<std::endl;
        return 0;
    }

    std::cout<< argv[1] <<std::endl;

    DIR *dir = opendir(argv[1]);

    if (dir != NULL) {
        std::cout << "Directory detected" << std::endl;
        dirent *file;
        char buffer[MAX_BUF];
        int map_proc = 0;
        while ((file = readdir(dir)) != NULL) {
            if (strcmp(file->d_name, ".") == 0 or strcmp(file->d_name, "..") == 0
                    or std::string(file->d_name).find("_log.xml") != std::string::npos) {
                continue;
            }
            snprintf(buffer, MAX_BUF, "%s\\%s", argv[1], file->d_name);
            std::cout << buffer << std::endl;
            read_map(buffer);
            ++map_proc;
            std::cout << "Map processed: " << map_proc << std::endl;
        }
    }
    else {
        read_map(argv[1]);
    }
}
