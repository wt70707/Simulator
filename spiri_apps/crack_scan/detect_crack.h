#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

bool detect_crack(cv::Mat img)
{
	//There are two stages of image processing: RGB to HSV format, and thresholding the HSV image
	Mat imgOriginal = img;		
	Mat imgHSV;
	Mat imgThresholded;

	//initialize the return variable
	bool crack_detected = false;
	
	//thresholding settings to find green against red brick, found by trial and error
	//have to change these values to find objects other than green cracks
	int iLowH = 38;			
	int iHighH = 75;

	int iLowS = 39; 
	int iHighS = 255;

	int iLowV = 7;
	int iHighV = 168;
	
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
	if (dArea > 20000)
		crack_detected = true;
	
	//returns true if there is enough white in the thresholded image
	return crack_detected;
}

