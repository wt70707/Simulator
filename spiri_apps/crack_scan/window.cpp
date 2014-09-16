#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "window.h"

using namespace cv;
using namespace std;

Window window(Mat img)
{
	Window frame = {
		false,
		0.0,			//x coordinate in frame
		0.0,			//y coordinate in frame
		0.0				//size of window in frame
	};

	//There are two stages of image processing: RGB to HSV format, and thresholding the HSV image
	Mat imgOriginal = img;		
	Mat imgHSV;
	Mat imgThresholded;

	int iLowH = 0;			//thresholding settings to find red window against brick wall
	int iHighH = 7;

	int iLowS = 176; 
	int iHighS = 193;

	int iLowV = 155;
	int iHighV = 169;
	
	//convert RBG to HSV format
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);	
		
	//threshold the HSV image
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 
	
	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
	//get the 'amount' of white against black in the thresholded image
	Moments oMoments = moments(imgThresholded);
	double dArea = oMoments.m00;
	
	//checks if there are any white blobs worth noticing against the black background of the thresholded image
	if (dArea > 10000)
	{
		frame.detected = true;

		frame.size = oMoments.m00;
		frame.x = oMoments.m10 / frame.size;
		frame.y = oMoments.m01 / frame.size;
	}

	//returns detection and centroid information
	return frame;
}