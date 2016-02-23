#include <FlyCapture2.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "MultipleCamera.hpp"

using namespace FlyCapture2;
using namespace std;
using namespace cv;

int RunSingleCamera( PGRGuid guid , unsigned int serialNumber);
int CaptureFrame(Camera* camera, unsigned int serialNumber);

void PrintBuildInfo() {
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    
    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
    cout << version.str() << endl;  
    
    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;  
}

void PrintCameraInfo( CameraInfo* pCamInfo ) {
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number -" << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;
}

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

int main(int /*argc*/, char** /*argv*/) {
    PrintBuildInfo();

    FlyCapture2::Error error;

    MultipleCamera mc;

    char c;
    while( true ) {
        c = waitKey(20);
        if ( c == 'q' ) {
            return 0;
        }

        map<unsigned int, Mat*>::iterator it;
        for (it = mc.frameMap.begin(); it != mc.frameMap.end(); it++) {
            //Mat *image = it->second;
            Mat mat = imread("15110255.png");
            imshow("title", mat);

        }
    }

    return 0;
}
