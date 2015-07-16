#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

void thresh_callback(int, void*);

int main(int argc, char** argv) {
    src = imread(argv[1], 1);

    cvtColor(src, src_gray, CV_BGR2GRAY);
    blur(src_gray, src_gray, Size(1, 1));

    char* source_window = "Source";
    namedWindow(source_window, CV_WINDOW_AUTOSIZE);
    imshow(source_window, src);

    createTrackbar(" Canny thresh:", "Source", &thresh, max_thresh, thresh_callback);
    thresh_callback(0, 0);

    waitKey(0);
    return(0);
}

void thresh_callback(int, void*) {
    Mat canny_output;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Canny(src_gray, canny_output, thresh, thresh*2, 3);
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    namedWindow("Canny", CV_WINDOW_AUTOSIZE);
    imshow("Canny", canny_output);

    // get the moments
    vector<Moments> mu( contours.size() );
    for (int x=0; x<contours.size(); x++) {
        mu[x] = moments( contours[x], false );
    }

    // get the mass centers
    vector<Point2f> mc( contours.size() );
    for (int x=0; x<contours.size(); x++) {
        mc[x] = Point2f(mu[x].m10/mu[x].m00, mu[x].m01/mu[x].m00);
    }

    // draw contours
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    for (int x=0; x<contours.size(); x++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, contours, x, color, 1, 8, hierarchy, 0, Point());
    }
    namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    imshow("Contours", drawing);

    // calculate the area with the moments 00 and compare with the result of the OpenCV function
    printf("\t Info: Area and Contour Length \n");
    for (int x=0; x<contours.size(); x++) {
        printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", x, mu[x].m00, contourArea(contours[x]), arcLength( contours[x], true ) );
        //Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        //drawContours(drawing, contours, x, color, 1, 8, hierarchy, 0, Point());
        //circle(drawing, mc[x], 4, color, -1, 8, 0);
    }
}
