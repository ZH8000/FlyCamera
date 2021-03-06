#ifndef COMMON_FLY_SDK_H
#define COMMON_FLY_SDK_H

#include <FlyCapture2.h>
#include "CameraProp.hpp"

class CommonFlySDK {
    FlyCapture2::Error error;

    public:
        CommonFlySDK();
        ~CommonFlySDK();
        void PrintBuildInfo();
        void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo );
        void PrintError( FlyCapture2::Error error );
        void PrintFormat7Capabilities(FlyCapture2::Format7Info fmt7Info);
        int initCamera( FlyCapture2::Camera *cam);
        void getCameraProp( FlyCapture2::Camera *cam, unsigned int serialNumber, CameraProp* prop);
        void setParamAutoOnOff(FlyCapture2::PropertyType type, int onOff, FlyCapture2::Camera *cam);
        void setParamValue(FlyCapture2::PropertyType type, int value, FlyCapture2::Camera *cam);
};
#endif
