#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main() {
    /*
    VideoCapture cap(0);

    if (!cap.isOpened()) {
        cout << "Capture could not be opened successfully" << endl;
        return -1;
    }

    namedWindow("Video");

    while (char(waitKey(1)) != 'q' && cap.isOpened()) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) {
            cout << "Video over" << endl;
            break;
        }
        imshow("Video", frame);
    }

    return 0;
    */
    Mat img = imread("sample.png", 0);
    bitwise_not(img, img);
    vector<Point> points;
    Mat_<uchar>::iterator it = img.begin<uchar>();
    
}
