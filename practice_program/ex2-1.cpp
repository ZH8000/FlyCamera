#include "highgui.h"

int main (int argc, char ** argv) {
    IplImage* img = cvLoadImage(argv[1]);
    cvNamedWindow("Example1", CV_WINDOW_AUTOSIZE);
    cvShowImage("Exmample1", img);
    cvWaitKey(0);
    cvReleaseImage(&img);
    cvDestroyWindow("Example1");
}
