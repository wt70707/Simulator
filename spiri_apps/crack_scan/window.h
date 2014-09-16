#ifndef WINDOW_H
#define WINDOW_H

struct Window {
	bool detected;
	double x;
	double y;
	double size;
};

Window window(cv::Mat img);

#endif


