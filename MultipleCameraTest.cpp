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

const char* win_title = "影像";
const char* win_setting = "攝影機 設定";
const char* win_opencv = "OpenCV 設定";

const char* expo_title = "自動曝光 Off/On";
const char* expo_value = "手動曝光值";
const char* shar_title = "自動影像銳利化 Off/On";
const char* shar_value = "手動影像銳利化值";
const char* shut_title = "自動快門 Off/On";
const char* shut_value = "手動快門值";
const char* bina_title = "影像二元化 Off/On";
const char* binv_title = "影像二元化反轉 Off/On";
const char* bina_max = "影像二元化最大接受閥值(+150)"; // binarization max value will between 0(+150) ~ 150(+150)
const char* bina_thresh = "影像二元化閥值";            // binarization thresh will between 0 ~ 150
const char* blur_title = "高斯模糊 Off/On";
const char* blur_value = "高斯模糊 Kernel 大小";
const char* succ_matches = "辦別成功最低Match值";
const char* tess_title = "單張文字辨識 trigger";

stringstream ss_title;
stringstream ss_setting;
stringstream ss_opencv;

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

        serialNs.push_back(camInfo.serialNumber);

        stringstream ss;
        ss << camInfo.serialNumber;
        namedWindow(ss.str());
        
        ss_title << win_title << camInfo.serialNumber;
        //ss_setting << win_setting << camInfo.serialNumber;
        //ss_opencv << win_opencv << camInfo.serialNumber;
        cout << "ss_title " << ss_title.str() << endl;
        cout << "ss_setting " << ss_setting.str() << endl;
        cout << "ss_opencv " << ss_opencv.str() << endl;
        ss_title.str(std::string());
        //namedWindow(ss_title.str(), WINDOW_NORMAL);
        //namedWindow(ss_setting.str(), WINDOW_NORMAL);
        //namedWindow(ss_opencv.str(), WINDOW_NORMAL);

		initCamera( ppCameras[i] );

        std::thread t(RunSingleCamera, guid, camInfo.serialNumber);
        t.detach();
    }

    char c;
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
    }

    return 0;
}

int RunSingleCamera( PGRGuid guid, unsigned int serialNumber ) {
    cout << "RunSingleCamera..." << serialNumber  << endl;

    Camera cam;
    FlyCapture2::Error error;

    // Connect to a camera
    error = cam.Connect( &guid );
    if ( error != PGRERROR_OK ) {
        PrintError( error );
        return -1;
    }

    // Start capturing images
    error = cam.StartCapture();
    if ( error != PGRERROR_OK ) {
        PrintError( error );
        return -1;
    }

    Image rawImage;
    Image rgbImage;

    while( true ) {
        error = cam.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            continue;
        }
        // getCameraProp(&cam);

        rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
        
        // resize to smaller size
        //Size size = Size(640, 480);
        //resize(image, image, size);
        
        threshold(image, image, 30, (250), CV_THRESH_BINARY);

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
