#include "cv_test.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	CV_test w;
	w.show();
	return a.exec();
}

/*#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main()
{
	// Load an existing image (make sure that the input URL really exists in your computer)
	cv::Mat pic = cv::imread("C:\\somepic.jpg", CV_LOAD_IMAGE_COLOR);

	cv::VideoCapture capture(0);       // open a first USB camera found (0-based camera index)
	if (!capture.isOpened())           // if camera not found, display an error message
		printf("Error: cannot open a camera\n");

	cv::Mat cam;
	while (true)
	{
		if (capture.isOpened())
			capture >> cam;                   // read a next frame from camera live stream

		cv::imshow("Still image", pic);      // display the still image on a window

		if (!cam.empty())
			cv::imshow("Camera image", cam);  // display live camera stream on another window

		if (cv::waitKey(1) == 27)            // exit this loop when ESC was pressed
			break;
	}

	return 0;
}
*/