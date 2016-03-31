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

CommonFlySDK::CommonFlySDK() {
    //cout << "CommonFlySDK constructor" << endl;
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
	//FlyCapture2::Error error;

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

void CommonFlySDK::getCameraProp( Camera *cam, unsigned int serialNumber, CameraProp* prop) {
    const unsigned int sk_numProps = 18;
    
//    CameraProp prop;
    Property camProp;
    PropertyInfo camPropInfo;
    
    prop->camId = serialNumber;
    
    for (unsigned int x = 0; x < sk_numProps; x++) {
        const PropertyType k_currPropType = (PropertyType)x;
        camProp.type = k_currPropType;
        camPropInfo.type = k_currPropType;

        FlyCapture2::Error getPropErr = cam->GetProperty( &camProp );
        FlyCapture2::Error getPropInfoErr = cam->GetPropertyInfo( &camPropInfo );
        if ( getPropErr != PGRERROR_OK || getPropInfoErr != PGRERROR_OK ||  camPropInfo.present == false) {
            continue;
        }
        if (camPropInfo.type == BRIGHTNESS) {
            prop->brightnessOnOff = camProp.autoManualMode;
            prop->brightnessValue = camProp.valueA;
//            cout << serialNumber << " BRIGHTNESS " << camProp.valueA << endl;
            
        } else
        if (camPropInfo.type == AUTO_EXPOSURE) {
            prop->exposureOnOff = camProp.autoManualMode;
            prop->exposureValue = camProp.valueA;
//            cout << serialNumber << " AUTO_EXPOSURE " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == SHARPNESS) {
            prop->sharpnessOnOff = camProp.autoManualMode;
            prop->sharpnessValue = camProp.valueA;
//            cout << serialNumber << " SHARPNESS " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == GAMMA) {
            prop->gammaOnOff = camProp.autoManualMode;
            prop->gammaValue = camProp.valueA;
//            cout << serialNumber << " GAMMA " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == SHUTTER) {
            prop->shutterOnOff = camProp.autoManualMode;
            prop->shutterValue = camProp.valueA;
//            cout << serialNumber << " SHUTTER " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == GAIN) {
            prop->gainOnOff = camProp.autoManualMode;
            prop->gainValue = camProp.valueA;
//            cout << serialNumber << " GAIN " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == FRAME_RATE) {
            prop->frameOnOff = camProp.autoManualMode;
            prop->frameValue = camProp.valueA;
///            cout << serialNumber << " FRAME_RATE " << camProp.valueA << endl;
        } else 
        if (camPropInfo.type == TEMPERATURE) {
//            cout << serialNumber << " TEMPERATURE " << camProp.valueA << " K" << endl;
        }
    }
    
    /*
    prop->binaryOnOff = 0;       // binarization
    prop->binaryInvOnOff = 0;    // binarization inverse
    prop->binaryMax = 100;       // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
    prop->oldBinaryMax = 100;
    prop->binaryThresh = 30;
    prop->oldBinaryThresh = 30;
    */
    
//    return prop;
}

void CommonFlySDK::setParamAutoOnOff(FlyCapture2::PropertyType type, int onOff, FlyCapture2::Camera *cam) {
    Property prop;
    prop.type = type;
    error = cam->GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        cout << "setParamAutoOnOff error1" << endl;
        PrintError( error );
    }
    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = onOff; 
    error = cam->SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        cout << "setParamAutoOnOff error2" << endl;
        PrintError ( error );
    }
}

void CommonFlySDK::setParamValue(FlyCapture2::PropertyType type, int value, FlyCapture2::Camera *cam) {
    Property prop;
    prop.type = type;
    error = cam->GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        cout << "setParamValue error1" << endl;
        PrintError( error );
    }

    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = 0;
    prop.valueA = value;
    error = cam->SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        cout << "setParamValue error2" << endl;
        PrintError( error );
    }
}
