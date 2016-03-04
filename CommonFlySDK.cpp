#include "CommonFlySDK.hpp"

#include <sstream>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <FlyCapture2.h>
#include "CameraProp.hpp"

using namespace FlyCapture2;
using namespace std;
//using namespace cv;

CommonFlySDK::CommonFlySDK() {
    cout << "CommonFlySDK constructor" << endl;
}

CommonFlySDK::~CommonFlySDK() {
}

void CommonFlySDK::PrintBuildInfo() {
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    
    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
    cout << version.str() << endl;  
    
    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;  
}

void CommonFlySDK::PrintCameraInfo( CameraInfo* pCamInfo ) {
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

void CommonFlySDK::PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

void CommonFlySDK::PrintFormat7Capabilities(Format7Info fmt7Info) {
    cout << "*** FORMAT 7 CAPABILITIES ***" << endl;
    cout << "Max image pixel: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
    cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
    cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
    cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
}

int CommonFlySDK::initCamera( Camera *cam) {
	CameraInfo camInfo;
	FlyCapture2::Error error;

	const Mode k_fmt7Mode = MODE_0;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;

    cam->GetCameraInfo( &camInfo );
    // 1. get Format7Info
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    error = cam->GetFormat7Info(&fmt7Info, &supported);
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

    error = cam->ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }

    if (!valid) {
        cout << "Format7 settings are not valid" << endl;
    }
}
