#ifndef THETA_H
#define THETA_H
#include "astar.h"

class Theta: public Astar
{
    public:
        Theta(double hweight, bool breakingties):Astar(hweight, breakingties){}
        ~Theta(void);

    private:
        bool lineOfSight(int i1, int j1, int i2, int j2, const Map *map, bool cutcorners, std::vector<Node> *path = nullptr, bool to_fill = false);
        virtual const Node *getParent(Node *current, Node *parent, const Map &map, const EnvironmentOptions &options) override;
        virtual void makePrimaryPath(Node curNode) override;
        virtual void makeSecondaryPath() override;

};


#endif // THETA_H
