#include <FlyCapture2.h>

#include <iostream>
#include <sstream>
#include <thread>
#include <map>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "CommonFlySDK.hpp"

using namespace FlyCapture2;
using namespace std;
using namespace cv;

int RunSingleCamera( PGRGuid guid , unsigned int serialNumber );
int initCamera( Camera *cam );

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

void buttonCallback(int state, void* data) {
    cout << "btnCallback" << endl;
}

int main(int /*argc*/, char** /*argv*/) {
    CommonFlySDK sdk;

    sdk.PrintBuildInfo();

    FlyCapture2::Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

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
        sdk.PrintCameraInfo( &camInfo );

        // Set all cameras to a specific mode and frame rate so they
        // can be synchronized
        error = ppCameras[i]->SetVideoModeAndFrameRate( 
            VIDEOMODE_1280x960Y8, 
            FRAMERATE_60 );
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            cout << "Error starting cameras. " << endl;
            cout << "This example requires cameras to be able to set to 1280x960 Y8 at 60fps. " << endl;
            cout << "If your camera does not support this mode, please edit the source code and recompile the application. " << endl;
            cout << "Press Enter to exit. " << endl;

            cin.ignore();
            return -1;
        }

        stringstream ss;
        ss << camInfo.serialNumber;
        namedWindow(ss.str());

        // Start capturing images
        error = ppCameras[i]->StartCapture();
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }
    }

# if 0
    error = Camera::StartSyncCapture( numCameras, (const Camera**)ppCameras );
    if (error != PGRERROR_OK) {
        PrintError( error );
        cout << "Error starting cameras. " << endl;
        cout << "This example requires cameras to be able to set to 640x480 Y8 at 30fps. " << endl;
        cout << "If your camera does not support this mode, please edit the source code and recompile the application. " << endl;
        cout << "Press Enter to exit. " << endl;

        cin.ignore();
        return -1;
    }
# endif

    char c;
    int count = 0;
    while ( true ) {
        c = waitKey(20);

        if ( c == 27) {
            for (unsigned int x = 0; x < numCameras; x++) {
                destroyAllWindows();
                ppCameras[x]->StopCapture();
                ppCameras[x]->Disconnect();
                delete ppCameras[x];
            }
            delete [] ppCameras;
            return 0;
        }

        for ( unsigned int x = 0; x < numCameras; x++) {
	        CameraInfo camInfo;
            ppCameras[x]->GetCameraInfo( &camInfo );

            Image rawImage;
            error = ppCameras[x]->RetrieveBuffer( &rawImage);
            if ( error != PGRERROR_OK ) {
                PrintError( error );
            }

            Image rgbImage;
            rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );

            // convert to OpenCV Mat
            unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
            Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

            stringstream ss;
            ss << camInfo.serialNumber;
            imshow(ss.str(), image);

            TimeStamp timestamp = rawImage.GetTimeStamp();
            cout << "Cam " << x << " - Frame " << (count++) << " - TimeStamp [" << timestamp.cycleSeconds << " " << timestamp.cycleCount << "]" << endl;
        }
    }

    return 0;
}
