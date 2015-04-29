#include <FlyCapture2.h>

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
using namespace FlyCapture2;

const char* win_title = "image";

Camera cam;
static const unsigned int sk_numProps = 18;

const char* expo_title = "Exposure Auto Off/On";
const char* expo_value = "Exposure Value";
int exposureOnOff = 0;
int exposureValue = 0;
int oldExposureValue = 0;
int shutterOnOff = 0;

void on_slider_exposureOnOff(int, void*);
void on_slider_exposureValue(int, void*);

void PrintBuildInfo() {
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf(
        version,
        "FlyCapture2 library version: %d.%d.%d.%d\n",
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

    printf( version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf( timeStamp );
}

void PrintCameraInfo(CameraInfo* pCamInfo) {
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}

void PrintError( Error error ) {
    error.PrintErrorTrace();
}

int RunSingleCamera( PGRGuid guid ) {
    const int k_numImages = 10;

    Error error;

    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    PrintCameraInfo(&camInfo);

    // Get the camera property
    Property camProp;
    PropertyInfo camPropInfo;    
    for (unsigned int x = 0; x < sk_numProps; x++) {
        const PropertyType k_currPropType = (PropertyType)x;
        camProp.type = k_currPropType;
        camPropInfo.type = k_currPropType;

        Error getPropErr = cam.GetProperty( &camProp );
        Error getPropInfoErr = cam.GetPropertyInfo( &camPropInfo );
        if ( getPropErr != PGRERROR_OK || getPropInfoErr != PGRERROR_OK ||  camPropInfo.present == false) {
            continue;
        }
        if (camPropInfo.type == AUTO_EXPOSURE) {
            cout << "EXPOSURE: " << camProp.autoManualMode << "  " << camProp.absValue << endl;
            exposureOnOff = camProp.autoManualMode;
            exposureValue = camProp.valueA;
            oldExposureValue = exposureValue;
            cout << "abs mode:" << camProp.absControl << endl;
            cout << "value A:" << camProp.valueA << endl; // INTEGER value (not in absolute mode)
            cout << "value B:" << camProp.valueB << endl;
            cout << "abs value:" << camProp.absValue << endl;
            setTrackbarPos(expo_title, win_title, exposureOnOff);
            setTrackbarPos(expo_value, win_title, exposureValue);
        }
        
    }


    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    Image rawImage;    
    while (char(waitKey(50)) != 'q') {                
        // Retrieve an image
        error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK) {
            PrintError( error );
            continue;
        }

        // Convert to RGB
        Image rgbImage;
        rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );
        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
        imshow(win_title, image);
//        Mat binaryImage;
//        threshold(image, binaryImage, 30., 255., CV_THRESH_BINARY);
//        imshow("bw image", binaryImage);
    }            

    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }      

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }
    return 0;
}

int main() {
    PrintBuildInfo();

    Error error;
    BusManager busMgr;
    unsigned int numCameras;

    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    namedWindow(win_title, WINDOW_NORMAL);
    // Setup trackbar
    createTrackbar(expo_title, win_title, &exposureOnOff, 1, on_slider_exposureOnOff);
    createTrackbar(expo_value, win_title, &exposureValue, 1022, on_slider_exposureValue);
    createTrackbar("Shutter Auto Off/On", win_title, &shutterOnOff, 1, on_slider_exposureOnOff);

    for (unsigned int i=0; i < numCameras; i++) {
        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }
        RunSingleCamera( guid );
    }
    return 0;
}

void on_slider_exposureOnOff(int, void*) {
    Error error;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    error = cam.GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        PrintError( error );
    }

    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = exposureOnOff; 
    error = cam.SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        PrintError ( error );
    }
}

void on_slider_exposureValue(int, void*) {
    if (exposureOnOff == 1) { // AUTO mode
        setTrackbarPos(expo_value, win_title, oldExposureValue);
    } else {
        Error error;
        Property prop;
        prop.type = AUTO_EXPOSURE;
        error = cam.GetProperty(&prop);
        if ( error != PGRERROR_OK) {
            PrintError( error );
        }

        prop.absControl = false;
        prop.onOff = true;
        prop.autoManualMode = 0;
        prop.valueA = exposureValue;
        error = cam.SetProperty(&prop, false);
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            setTrackbarPos(expo_value, win_title, oldExposureValue);
        } else {
            oldExposureValue = exposureValue;
        }
    }
}
