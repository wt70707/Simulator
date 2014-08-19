#include <spiri_api/staterobot.h>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "detect_crack.h"					//cpp code is included in header file

using namespace cv;
using namespace std;

int main()
{
	//makes an object of Staterobot to use for this app
	Staterobot robot;
	
	//initialize local variables
	bool crack_detected = false;	
	bool start = true;	
	Mat frame;
	string save_path;
	int save_img_i = 1;
	int num_trajectories = 8;
	
	//set up an array of [y,z] values, and make a while loop to run through its length
	
	//  [0][n] is y and [1][n] is z
	float trajectory[2][8] = 
	{
		{0.,6.5,0.,5.5,0.,5.5,0.,-.5},	// y values
		{6.,0.,-3., 0.,3., 0.,-6.,0.}	// z values
	};
	
	//Execute specified scanning procedure
	
	cout << "\nWelcome to the Spiri Crack Scanning App!\n";
	cout << "\nSpiri will traverse the wall and take pictures whenever it detects cracks.\n";
	cout << "\nYou can right-click on the spiri model in the World tab to follow its motion.\n";
	
	for(int i=0; i <= num_trajectories; i++)
	{
		if(robot.wait_goal() == 1 || start == true)		//check to make sure two trajectories aren't being sent at the same time
														//the bool 'start' is there because the first time this loop runs wait_goal is not 1 yet
		{
			robot.send_goal(0., trajectory[0][i], trajectory[1][i],true);
			start = false;								//no longer need to trigger the start condition above ^
			
			while(robot.wait_goal() == 0)				//while Spiri is moving with a goal
			{
				frame = robot.get_left_image();			//takes Spiri's left camera from the Staterobot class
				namedWindow("video",CV_WINDOW_AUTOSIZE);
				imshow("video",frame);					//display left camera as video
				if(waitKey(30)==27)
					break;								//hit ESC to exit scanning until next goal is sent
					
				crack_detected = detect_crack(frame);	//calls the function from detect_crack.h, using OpenCV
				
				if(crack_detected)						//take a few pictures and save to desktop
				{
					crack_detected = false;
					cout << "Crack detected!\n";		//notify user
					
					//create numbered image paths
					ostringstream oss;
					oss << save_img_i;
					save_path = "/home/rob/Desktop/crack_";	// <----- change this to chosen path!
					save_path += oss.str();
					save_path += ".jpg";
					save_img_i++;
					
					imwrite(save_path,frame); 			//save the crack image
					sleep(1);							//wait for a second before taking another snapshot (if a crack is still detected)
				}
			}
		}
	}
	
	//final text
	cout << "\nScan Completed! " << save_img_i << " images of detected cracks are saved to your Desktop.\n";
	
	return 0;
}

