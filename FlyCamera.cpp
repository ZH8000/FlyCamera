#include <FlyCapture2.h>

#include <iostream>
#include <stdio.h>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tesseract/baseapi.h>
#include <tesseract/strngs.h>

using namespace cv;
using namespace std;
using namespace FlyCapture2;

const char* win_title = "影像";
const char* win_setting = "攝影機 設定";
const char* win_opencv = "OpenCV 設定";

Camera cam;
static const unsigned int sk_numProps = 18;

const char* expo_title = "自動曝光 Off/On";
const char* expo_value = "自動曝光值";
const char* shut_title = "自動快門 Off/On";
const char* shut_value = "快門值";
const char* bina_title = "影像二元化 Off/ On";
const char* bina_max = "影像二元化最大接受閥值(+150)"; // binarization max value will between 0(+150) ~ 150(+150)
const char* bina_thresh = "影像二元化閥值";      // binarization thresh will between 0 ~ 150
const char* tess_title = "文字辨識 trigger";

int exposureOnOff = 0;
int exposureValue = 0;
int oldExposureValue = 0;
int shutterOnOff = 0;
int shutterValue = 0;
int oldShutterValue = 0;
int binaryOnOff = 0;
int binaryMax =  100; // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
int oldBinaryMax = 100;
int binaryThresh = 30;
int oldBinaryThresh = 30;
int ocrOnOff = 0;

void on_slider_exposureOnOff(int, void*);
void on_slider_exposureValue(int, void*);
void on_slider_shutterOnOff(int, void*);
void on_slider_shutterValue(int, void*);
void on_slider_binaryOnOff(int, void*);
void on_slider_binaryMax(int, void*);
void on_slider_binaryThresh(int, void*);
void on_slider_ocrOnOff(int, void*);
void OCR();

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
            exposureOnOff = camProp.autoManualMode;
            exposureValue = camProp.valueA;
            oldExposureValue = exposureValue;
            cout << "EXPOSURE: " << camProp.autoManualMode << endl;
            cout << "-abs mode:" << camProp.absControl << endl;
            cout << "-value A:" << camProp.valueA << endl; // INTEGER value (not in absolute mode)
            cout << "-abs value:" << camProp.absValue << endl;
            setTrackbarPos(expo_title, win_setting, exposureOnOff);
            setTrackbarPos(expo_value, win_setting, exposureValue);
        } else 
        if (camPropInfo.type == SHUTTER) {
            shutterOnOff = camProp.autoManualMode;
            shutterValue = camProp.valueA;
            oldShutterValue = shutterValue;
            cout << "SHUTTER: " << camProp.autoManualMode << endl;
            cout << "-abs mode:" << camProp.absControl << endl;
            cout << "-value A:" << camProp.valueA << endl; // INTEGER value (not in absolute mode)
            cout << "-abs value:" << camProp.absValue << endl;
            setTrackbarPos(shut_title, win_setting, shutterOnOff);
            setTrackbarPos(shut_value, win_setting, shutterValue);
        }
    }


    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    tesseract::TessBaseAPI tess;
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);

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

        if (binaryOnOff == 1) {
            Mat bImage;
            Mat origImage = image.clone();
            threshold(origImage, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY);
        }

        if (ocrOnOff == 1) {
            ocrOnOff = 0;
            setTrackbarPos(tess_title, win_opencv, ocrOnOff); 

            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
            try {
                imwrite("image.png", image, compression_params);
            } catch (runtime_error& ex) {
                fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
                // return 1;
            }
            OCR();

            // tess.SetImage((uchar*)image.data, image.cols, image.rows, 1, image.cols);
            // char* out = tess.GetUTF8Text();
            // cout << "OCR OUTPUT: " << out << endl;
        }

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
    namedWindow(win_setting, WINDOW_NORMAL);
    namedWindow(win_opencv, WINDOW_NORMAL);
    // Setup trackbar
    createTrackbar(expo_title, win_setting, &exposureOnOff, 1, on_slider_exposureOnOff);
    createTrackbar(expo_value, win_setting, &exposureValue, 1023, on_slider_exposureValue);
    createTrackbar(shut_title, win_setting, &shutterOnOff, 1, on_slider_shutterOnOff);
    createTrackbar(shut_value, win_setting, &shutterValue, 1590, on_slider_shutterValue);
    createTrackbar(bina_title, win_opencv, &binaryOnOff, 1, on_slider_binaryOnOff);
    createTrackbar(bina_max, win_opencv, &binaryMax, 150, on_slider_binaryMax);
    createTrackbar(bina_thresh, win_opencv, &binaryThresh, 150, on_slider_binaryThresh);
    createTrackbar(tess_title, win_opencv, &ocrOnOff, 1, on_slider_ocrOnOff);

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
        setTrackbarPos(expo_value, win_setting, oldExposureValue);
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
            setTrackbarPos(expo_value, win_setting, oldExposureValue);
        } else {
            oldExposureValue = exposureValue;
        }
    }
}

void on_slider_shutterOnOff(int, void*) {
    Error error;
    Property prop;
    prop.type = SHUTTER;
    error = cam.GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        PrintError( error );
    }

    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = shutterOnOff;
    error = cam.SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        PrintError ( error );
    }
}

void on_slider_shutterValue(int, void*) {
    if (shutterOnOff == 1) { // AUTO mode
        setTrackbarPos(shut_value, win_setting, oldShutterValue);
    } else {
        Error error;
        Property prop;
        prop.type = SHUTTER;
        error = cam.GetProperty(&prop);
        if ( error != PGRERROR_OK) {
            PrintError( error );
        }

        prop.absControl = false;
        prop.onOff = true;
        prop.autoManualMode = 0;
        prop.valueA = shutterValue;
        error = cam.SetProperty(&prop, false);
        if ( error != PGRERROR_OK ) { 
            PrintError( error );
            setTrackbarPos(shut_value, win_setting, oldShutterValue);
        } else {
            oldShutterValue = shutterValue;
        }
    }   
}

void on_slider_binaryOnOff(int, void*) {}

void on_slider_binaryMax(int, void*) {
    if (binaryOnOff == 0) {
        setTrackbarPos(bina_max, win_opencv, oldBinaryMax);
    } else {
        oldBinaryMax = binaryMax;
    }
}

void on_slider_binaryThresh(int, void*) {
    if (binaryOnOff == 0) {
        setTrackbarPos(bina_thresh, win_opencv, oldBinaryThresh);
    } else {
        oldBinaryThresh = binaryThresh;
    }
}

void on_slider_ocrOnOff(int, void*) {}

void OCR() {
    // tess.SetImage((uchar*)image.data, image.cols, image.rows, 1, image.cols);
    // char* out = tess.GetUTF8Text();
    // cout << "OCR OUTPUT: " << out << endl;
    FILE* fin = fopen("image.png", "rb");
    if (fin == NULL) {
        std::cout << "Cannot open image.png file" << std::endl;
    }
    fclose(fin);

    tesseract::TessBaseAPI tess;
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
    STRING text;
    if (!tess.ProcessPages("image.png", NULL, 0, &text)) {
        cout << "Error during processing." << std::endl;
    } else {
        cout << text.string() << std::endl;
        cout << "--------------------------" << endl;
    }
}
