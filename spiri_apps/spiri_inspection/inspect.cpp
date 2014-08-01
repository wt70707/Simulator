#include <spiri_api/staterobot.h>
#include <sstream>

int main()
{
    Staterobot robot;
    robot.send_goal(0,0,1,true);
    int i=0;
    // save images while moving towards a goal
    
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