#include <thread>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <FlyCapture2.h>


using namespace FlyCapture2;
using namespace std;
using namespace cv;

class MultipleCamera {

    public:
        vector<unsigned int> serialNumbers;
        vector<thread*> cameraThread;
        map<unsigned int, Mat*> frameMap;
        map<unsigned int, Camera*> cameraMap;
    
        MultipleCamera();
        ~MultipleCamera();

    private:
        unsigned int numCameras;
        int startMultipleCapture();
        Camera** camArray; 
        void stopMultipleCapture();
        void captureFrame(unsigned int serialNumber);
        void PrintError(FlyCapture2::Error error);
};
