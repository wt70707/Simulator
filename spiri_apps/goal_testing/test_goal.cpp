#include <spiri_api/staterobot.h>
#include <sstream>
#include <iostream>
int main()
{
    Staterobot robot;
    robot.send_goal(0,0,1,true);
    return 0;
}