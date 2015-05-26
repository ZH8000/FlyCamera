#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
  VideoCapture cap(0);
  if (!cap.isOpened())
  {
    cout << "Capture could not be opened successfully" << endl;
    return -1;
  }

  Size S = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH), (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));

  VideoWriter put("output.mpg", CV_FOURCC('M', 'P', 'E', 'G'), 30, S);
  if (!put.isOpened())
  {
    cout << "File could not be created for writing. Check permissions" << endl;
    return -1;
  }

  namedWindow("Video");

  while (char(waitKey(1)) != 'q' && cap.isOpened())
  {
    Mat frame;
    cap >> frame;
    if (frame.empty())
    {
      cout << "Video over" << endl;
      break;
    }
    imshow("Video", frame);
    put << frame;
  }

  return 0;
}
