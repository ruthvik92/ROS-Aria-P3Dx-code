#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <iostream>

using namespace cv;
using namespace std;

int main(int,char**)
{
	VideoCapture cap(0);
	if(!cap.isOpened())
		cout<<"Camera not detected"<<endl;
	while(1)
	{
		Mat frame;
		namedWindow("display",1);
		cap >> frame;
		imshow("display",frame);
		waitKey(1);
	}
		
}
