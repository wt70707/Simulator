#include <spiri_api/staterobot.h>
#include <sstream>
#include <iostream>
int main()
{
    Staterobot robot;
    //Staterobot::state state=robot.get_state();
    //std::cout<<state.position.x;
    
    robot.send_goal(-10,0,1,true);
    //int i=0;
    // save images while moving towards a goal
    /*
    while(robot.wait_goal()==0)
    {
      std::ostringstream oss;
      std::string path="/home/rob/";
      std::string extension=".jpg";
      oss<<i;
      path+=oss.str();
      path+=extension;
      robot.save_image(path,"left");
      i++;
    }
    */
    /*
    robot.send_vel(0,0,1);
    //sleep(1.0);
    robot.send_vel(0,0,0);
    */
    
    /*
    // save only the last image
    
    // have to do this as the image callbacks don't get updated. Need to fix this
    robot.save_image("/home/rob/start.jpg","left");

    while(robot.wait_goal()==0)
    {
    }
    robot.save_image("/home/rob/final.jpg","left");
    std::cout<<"saved the images";
    */
    return 0;
}