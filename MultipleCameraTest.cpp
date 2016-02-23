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
//int CaptureFrame(Camera* camera, unsigned int serialNumber);

/*
typedef map<unsigned int, Mat> FrameCollocation;
FrameCollocation M;
*/

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

    //const int k_numImages = 100;
    FlyCapture2::Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    vector<unsigned int> serialNs;

    cout << "Number of cameras detected: " << numCameras << endl; 

    if ( numCameras < 1 ) {
        cout << "Insufficient number of cameras... press Enter to exit." << endl; ;
        cin.ignore();
        return -1;
    }

    Camera** ppCameras = new Camera*[numCameras];

    // Connect to all detected cameras and attempt to set them to
    // a common video mode and frame rate
    for ( unsigned int i = 0; i < numCameras; i++) {
        ppCameras[i] = new Camera();

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex( i, &guid );
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }

        // Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }

        PrintCameraInfo(&camInfo); 

        serialNs.push_back(camInfo.serialNumber);

        stringstream ss;
        ss << camInfo.serialNumber;
        namedWindow(ss.str());

        std::thread t(RunSingleCamera, guid, camInfo.serialNumber);
        t.detach();
    }

    char c;
    while ( true ) {
        c = waitKey(20);

        if ( c == 27) {
            for (unsigned int x = 0; x < numCameras; x++) {
                ppCameras[x]->StopCapture();
                ppCameras[x]->Disconnect();
                delete ppCameras[x];
            }
            delete [] ppCameras;
            return 0;
        }
    }

    return 0;
}

int RunSingleCamera( PGRGuid guid, unsigned int serialNumber ) {
    cout << "RunSingleCamera..." << serialNumber  << endl;

    Camera cam;
    FlyCapture2::Error error;

    error = cam.Connect( &guid );
    if ( error != PGRERROR_OK ) {
        PrintError( error );
        return -1;
    }

    error = cam.StartCapture();
    if ( error != PGRERROR_OK ) {
        PrintError( error );
        return -1;
    }

    Image rawImage;

    while( true ) {
        error = cam.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK ) {
            PrintError( error );
        }
    
        Image rgbImage;
        rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );
    
        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
    
        stringstream ss;
        ss << serialNumber;
        imshow(ss.str(), image);
    }
}
