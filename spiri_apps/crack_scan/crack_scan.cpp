#include <spiri_api/staterobot.h>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "detect_crack.h"					//cpp code is included in header file
#include "window.h"							//returns information about a window

using namespace cv;
using namespace std;

//rounding function used for converting Spiri's position to grid values
int round_int(float r)
{
	return (r > 0.0) ? (r + 0.5) : (r - 0.5);
}

int main()
{

	//////////////////////////////////////////
	//            INITIALIZATION            //
	//////////////////////////////////////////

	//makes an object of Staterobot to use for this app
	Staterobot robot;
	
	//initialize local variables
	bool crack_detected = false;
	bool reset_detection = true;
	bool going_up = true;	
	bool going_left = false;	
	bool column_unfinished;
	bool left_unfinished;
	bool edges_in_column;	

	int save_img_i = 1;
	int resolution = 2;
	int wall_y, wall_z;

	float y, z;
	float speed = 0.3;
	float finished[2] = {19.5,9.2};
	float y_top = 60.;
	float y_middle = 120.;
	float y_bottom = 180.;
	float x_left = 80.;
	float x_middle = 160.;
	float x_right = 240.;
	float seconds = 0.5;
	float hop = 0.2;

	Window window_pic;
	vector<double> state;
	Mat frame;
	string save_path;

	struct grid
	{
		bool scanned;
		bool edge;
		bool crack;
	};

	grid wall[19][40];

	//initialize the grid values
	for(int i=0; i < 19; i++)
	{
		for(int j=1; j<40; j++)
		{
			wall[i][j].scanned = false;
			wall[i][j].edge = false;
			wall[i][j].crack = false;
		}
	}

	for(int i = 0; i < 19; i++)
	{
	    wall[i][0].scanned = true;
	}
	for(int j = 0; j < 40; j++)
	{
		wall[0][j].scanned = true;
	}

	//check initial state 
	state = robot.get_state();
	y = (float)state[1];
	z = (float)state[2];

	////////////////////////////////////////////
	//           INTRODUCTION OUPUT           //
	////////////////////////////////////////////
	
	//intro 
	cout << "\nWelcome to Spiri's Crack Scanning App!\n";
	cout << "\nSpiri will traverse the generic wall, avoiding windows, and take pictures whenever it detects a crack.\n";		

	////////////////////////////////////////////
	//              SCANNING LOOP             //
	////////////////////////////////////////////

	//start going up
	robot.send_vel(0,0,speed*1.5);

	//keep going up and down, from left to right, avoiding windows, taking pictures of cracks
	//until the right side of the wall is reached	

	while(y < finished[0] || z < finished[1])
	{
		/////////////////////////////////////
		//         UPDATE THE GRID         //
		/////////////////////////////////////

		//check Spiri's state
		state = robot.get_state();
		y = (float)state[1];
		z = (float)state[2];

		//determine which grid block Spiri is in
		wall_y = round_int(y*resolution);
		wall_z = round_int(z*resolution);

		// record whether that block has been scanned by Spiri
		wall[wall_z][wall_y].scanned = true;

	    //get window info from camera and pass it to a window function
		frame = robot.get_left_image();
		Window w = window(frame);

		//output an array map of wall with 1 for scanned, 0 for not scanned
		cout << "Crack Locations:\n";
		
		for (int i = 18; i > 0; i--)
	    {
	        for (int j = 1; j <40; j++)
	        {
	            cout << wall[i][j].crack << ' ';
	        }
	        cout << endl;
	    }

		/////////////////////////////////////////////////////////////
		//			WINDOW AVOIDANCE AND PATH PLANNING LOGIC       //
		/////////////////////////////////////////////////////////////

		//behavioral choices to navigate around windows and scan in between them

		//if a window is detected using OpenCV image processing and highgui
		if(w.detected)
		{	
			//record this to the grid
			wall[wall_z][wall_y].edge = true;

			//if Spiri is going up
			if(going_up && !going_left)
			{
				//if window is above and a bit to the right, go right and down
				if(w.y < y_top && w.x > x_left*3/2 && w.x < (x_right + x_left/4))
				{
					robot.send_vel(0,hop,0);
					robot.send_vel(0,0,-speed);
					going_up = false;
				}

				//if window is left 'ish'
				else if(w.x > x_left*3/4 && w.x < x_left*3/2 && w.y < y_bottom)
				{
					//if blocks above aren't scanned
					if(!wall[wall_z+1][wall_y].scanned)
					//then go right and up around the corner
					{
						robot.send_vel(0,hop,0);
						robot.send_vel(0,0,speed);
					}
					//otherwise go back down
					else
					{
						robot.send_vel(0,hop,0);
						robot.send_vel(0,0,-speed);
						going_up = false;
					}
				}

				//if window is in bottom left and the grid to the left is unscanned
				else if(w.y > y_bottom && w.x < x_left*3/4 
					&& !wall[wall_z][wall_y-1].scanned)
				{
					robot.send_vel(0,-speed,0);
					going_left = true;
				}
			}

			//if Spiri is going down
			else if(!going_up && !going_left)
			{

				//if window is below and a bit to the right, go right and up
				if(w.y > y_bottom && w.x > x_left*3/2 && w.x < (x_right+ x_left/4))
				{
					robot.send_vel(0,hop,0);
					robot.send_vel(0,0,speed);
					going_up = true;
				}

				//if window is a left 'ish'
				else if(w.x > x_left*3/4 && w.x < x_left*3/2 && w.y > y_top)
				{

					//if blocks below aren't scanned
					if(!wall[wall_z-1][wall_y].scanned)
					//then go right and down around the corner
					{
						robot.send_vel(0,hop,0);
						robot.send_vel(0,0,-speed);
					}
					//otherwise go back up
					else
					{
						robot.send_vel(0,hop,0);
						robot.send_vel(0,0,speed);
						going_up = true;
					}
				}

				//if window is in top left and the grid to the left is unscanned
				else if(w.y < y_top && w.x < x_left*3/4
					    && !wall[wall_z][wall_y-1].scanned)
				{
					robot.send_vel(0,-speed,0);
					going_left = true;
				}
			}
		}

		//if Spiri is going left
		if(going_left)
		{
			if(w.detected)
			{
				wall[wall_z][wall_y].edge = true;

				//once Spiri reaches a scanned square, turn up or down again
				if(wall[wall_z][wall_y-1].scanned)
				{
					if(w.y < y_top*3/2)
					{
						robot.send_vel(0,0,-speed);
						going_up = false;
						going_left = false;
					}
					else if(w.y > y_top*5/2)
					{
						robot.send_vel(0,0,speed);
						going_up = true;
						going_left = false;
					}
				}
			}

			//otherwise if Spiri loses track of a window, default go up
			else
			{
				if(wall[wall_z][wall_y-1].scanned)
				{
						robot.send_vel(0,0,speed);
						going_up = true;
						going_left = false;
				}
			}
		}

		/////////////////////////////////////
		//	  TOP AND BOTTOM BOUNDARIES    //
		/////////////////////////////////////

		//hits the top, turn around
		if(z > finished[1] && going_up)
		{
			edges_in_column = false;

			for(int i=0; i < 19; i++)
			{
				if(wall[i][wall_y].edge == true)
					edges_in_column = true;
			}

			//go right and down
			if(!edges_in_column)
			{
				robot.send_vel(0,speed*1.5,0);
				robot.send_vel(0,0,-speed*1.5);
			}
			else
			{
				robot.send_vel(0,speed,0);
				robot.send_vel(0,0,-speed);
			}

			going_up = false;
		}

		//hits the ground, turn around 
		else if(z < 0.7 && !going_up)
		{
			edges_in_column = false;

			for(int i=0; i < 19; i++)
			{
				if(wall[i][wall_y].edge == true)
					edges_in_column = true;
			}

			//go right and up
			if(!edges_in_column)
			{
				robot.send_vel(0,speed*1.5,0);
				robot.send_vel(0,0,speed*1.5);
			}
			else
			{
				robot.send_vel(0,speed,0);
				robot.send_vel(0,0,speed);
			}

			going_up = true;
		}

		/////////////////////////////////////
		//			CRACK DETECTION        //
		/////////////////////////////////////

		//calls boolean function, true if crack is detected
		crack_detected = detect_crack(frame);	

		if(crack_detected)
		{
			//if this is not a repeat view of the same crack in any surrounding block
			if(!wall[wall_z][wall_y-1].crack
				&& !wall[wall_z][wall_y+1].crack
				&& !wall[wall_z-1][wall_y].crack
				&& !wall[wall_z+1][wall_y].crack
				&& !wall[wall_z-1][wall_y-1].crack
				&& !wall[wall_z-1][wall_y+1].crack
				&& !wall[wall_z+1][wall_y-1].crack
				&& !wall[wall_z+1][wall_y+1].crack)
			{
				wall[wall_z][wall_y].crack = true;
			}
		}
		
		//checks to see if it is a new crack in the image
		if(!crack_detected)						
		{
			reset_detection = true;
		}
		
		//takes a picture of the crack and saves to desktop
		if(crack_detected && reset_detection 
			&& wall[wall_z][wall_y].crack)	
		{
			frame = robot.get_left_image();
			reset_detection = false;
			cout << "Crack detected!\n";		//notify user
			
			//create numbered image paths
			ostringstream oss;
			oss << save_img_i;
			save_path = "/home/spiri/Desktop/crack_";	// <----- change this to chosen path!
			save_path += oss.str();
			save_path += ".jpg";
			save_img_i++;
			
			imwrite(save_path,frame); 			//save the crack image
		}
	} 

	//////////////////////////////
	//        TERMINATION       //
	//////////////////////////////

	//steady Spiri
	robot.send_vel(0,0,0);
	
	//final text
	cout << "\nScan Completed! " << save_img_i - 1 << " images of detected cracks are saved to your Desktop.\n(multiple shots of the same crack may have been taken!)\n";
	cout << "\n------------------\n";
	cout << "\nTo run the scan again, click Edit:Reset Model Poses on the top left of Gazebo and type ./crack_scan in terminal.\n\n";
	
	return 0;
}

