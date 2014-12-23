#ifndef BOTCONNECTOR_H
#define BOTCONNECTOR_H

#include "Aria.h"
#include <vector>

using namespace std;

class BotConnector
{
public:
    BotConnector(int argc, char **argv);
    bool connect();
    int disconnect();
    vector<int> getReadings();
    vector<int> getAngles();
    void moveRobot(double r=1.0, double th=45.0);
    void setRobotVelocity(double vr=200.0, double vl=200.0, unsigned int time=2000);
private:
    ArRobot robot;
    ArArgumentParser *parser;
    ArRobotConnector *robotConnector;
    ArActionGoto *gotoPoseAction;
    int crap;

};

#endif // BOTCONNECTOR_H
