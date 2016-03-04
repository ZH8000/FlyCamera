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


#include "MultipleCamera.hpp"

using namespace FlyCapture2;
using namespace std;
using namespace cv;

int RunSingleCamera( PGRGuid guid , unsigned int serialNumber );
int initCamera( Camera *cam );

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

void buttonCallback(int state, void* data) {
    cout << "btnCallback" << endl;
}

void PrintFormat7Capabilities(Format7Info fmt7Info) {
    cout << "*** FORMAT 7 CAPABILITIES ***" << endl;
    cout << "Max image pixel: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
    cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
    cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
    cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
}

int main(int /*argc*/, char** /*argv*/) {
    PrintBuildInfo();

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
        /*
        serialNs.push_back(camInfo.serialNumber);

        stringstream ss;
        ss << camInfo.serialNumber;
        namedWindow(ss.str());
        //char* buttonName = "Bt1";
        //createButton(buttonName, buttonCallback, NULL, CV_PUSH_BUTTON, 0);

		initCamera( ppCameras[i] );

        std::thread t(RunSingleCamera, guid, camInfo.serialNumber);
        t.detach();
        */
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

        //TimeStamp timestamp = rawImage.GetTimeStamp();
        //cout << "Camera " << ss.str() << " - TimeStamp [" << timestamp.cycleSeconds << " " << timestamp.cycleCount << "]" << endl;
    }
}

int initCamera( Camera *cam) {
	CameraInfo camInfo;
	FlyCapture2::Error error;
	
	const Mode k_fmt7Mode = MODE_0;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;
	
    cam->GetCameraInfo( &camInfo );
    // 1. get Format7Info
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    error = cam -> GetFormat7Info(&fmt7Info, &supported);
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }
    PrintFormat7Capabilities(fmt7Info);
    
    // 2. validate Format7ImageSettings
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    error = cam -> ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }
    if (!valid) {
        cout << "Format7 settings are not valid" << endl;
    }
}
