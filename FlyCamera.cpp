#include <FlyCapture2.h>

#include <ctime>
#include <iostream>
#include <stdio.h>
#include <stdexcept>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tesseract/baseapi.h>
#include <tesseract/strngs.h>

using namespace cv;
using namespace std;
using namespace FlyCapture2;

bool ldown = false, lup = false;
Point corner1, corner2, moving;
Rect box;

const char* win_title = "影像";
const char* win_setting = "攝影機 設定";
const char* win_opencv = "OpenCV 設定";

Camera cam;
static const unsigned int sk_numProps = 18;

const char* expo_title = "自動曝光 Off/On";
const char* expo_value = "手動曝光值";
const char* shar_title = "自動影像銳利化 Off/On";
const char* shar_value = "手動影像銳利化值";
const char* shut_title = "自動快門 Off/On";
const char* shut_value = "手動快門值";
const char* bina_title = "影像二元化 Off/ On";
const char* bina_max = "影像二元化最大接受閥值(+150)"; // binarization max value will between 0(+150) ~ 150(+150)
const char* bina_thresh = "影像二元化閥值";      // binarization thresh will between 0 ~ 150
const char* tess_title = "單張文字辨識 trigger";

int exposureOnOff = 0;         // exposure
int exposureValue = 0;
int oldExposureValue = 0;
int sharpnessOnOff = 0;        // sharpness
int sharpnessValue = 3000;
int oldSharpnessValue = 3000;
int shutterOnOff = 0;          // shutter
int shutterValue = 0;
int oldShutterValue = 0;
int binaryOnOff = 0;           // binarization
int binaryMax =  150;          // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
int oldBinaryMax = 100;
int binaryThresh = 30;
int oldBinaryThresh = 30;
int ocrOnOff = 0;              // OCR

void on_slider_exposureOnOff(int, void*);  // exposure
void on_slider_exposureValue(int, void*);
void on_slider_sharpnessOnOff(int, void*); // sharpness
void on_slider_sharpnessValue(int, void*);
void on_slider_shutterOnOff(int, void*);   // shutter
void on_slider_shutterValue(int, void*);
void on_slider_binaryOnOff(int, void*);    // binarization
void on_slider_binaryMax(int, void*);
void on_slider_binaryThresh(int, void*);
void on_slider_ocrOnOff(int, void*);       // OCR
void OCR();

static void mouse_callback(int event, int x, int y, int, void *);

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

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

int RunSingleCamera( PGRGuid guid ) {
    const int k_numImages = 10;

    FlyCapture2::Error error;

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

        FlyCapture2::Error getPropErr = cam.GetProperty( &camProp );
        FlyCapture2::Error getPropInfoErr = cam.GetPropertyInfo( &camPropInfo );
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

    // setup Tesseract ocr engine
    tesseract::TessBaseAPI tess;
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);

    Image rawImage;
    char c;   
    while (true) {
        c = waitKey(50);
        if (c == 'q') {
            return 0;
        }

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

	 int sigma = 0.3 * ((5 - 1) * 0.5 - 1) + 0.8;
    	GaussianBlur(image, image, Size(3, 3), sigma);
    	//Canny(image, image, 10, 50, 3);

        if (binaryOnOff == 1) {
            Mat bImage;
            Mat origImage = image.clone();
            threshold(origImage, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY);
        }

        // handle mouse click event
        /*
        if (ldown == true && lup == false) {
            Point pt;
            pt.x = x;
            pt.y = y;
        }*/

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

        if (c == 's') {
            /*
            time_t t = time(0);
            struct tm * now = localtime( & t );
            string s;
            stringstream ss(s);
            ss << (now->tm_year + 1900) << "-"
               << (now->tm_mon + 1) << "-"
               << now->tm_mday << "_"
               << now->tm_hour << "-"
               << now->tm_min << "-"
               << now->tm_sec
               << endl;*/
            time_t rawtime;
            struct tm * timeinfo;
            char buffer[80];
            time (&rawtime);
            timeinfo = localtime(&rawtime);
            strftime(buffer, 80, "%Y-%m-%d_%I-%M-%S", timeinfo);
            string str(buffer);
            imwrite(str + ".png", image);
            cout << "Capture image " << str << ".png" << endl;
        }

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

    cout << "Press 'q' to quit" << endl;
    cout << "Press 's' to take a picture" << endl;

    PrintBuildInfo();

    FlyCapture2::Error error;
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
    createTrackbar(shar_title, win_setting, &sharpnessOnOff, 1, on_slider_sharpnessOnOff);
    createTrackbar(shar_value, win_setting, &sharpnessValue, 4095, on_slider_sharpnessValue);
    createTrackbar(shut_title, win_setting, &shutterOnOff, 1, on_slider_shutterOnOff);
    createTrackbar(shut_value, win_setting, &shutterValue, 1590, on_slider_shutterValue);
    createTrackbar(bina_title, win_opencv, &binaryOnOff, 1, on_slider_binaryOnOff);
    createTrackbar(bina_max, win_opencv, &binaryMax, 150, on_slider_binaryMax);
    createTrackbar(bina_thresh, win_opencv, &binaryThresh, 150, on_slider_binaryThresh);
    createTrackbar(tess_title, win_opencv, &ocrOnOff, 1, on_slider_ocrOnOff);

    setMouseCallback(win_title, mouse_callback);

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

static void mouse_callback(int event, int x, int y, int, void *) {
    if (event == EVENT_LBUTTONDOWN) {
        ldown = true;
        corner1.x = x;
        corner1.y = y;
    }
    if (event == EVENT_LBUTTONUP) {
        lup = true;
        corner2.x = x;
        corner2.y = y;
    }
    if (ldown == true && lup == false) {
        moving.x = x;
        moving.y = y;
    }
    if (ldown == true && lup == true) {
        box.width = abs(corner1.x - corner2.x);
        box.height = abs(corner1.y - corner2.y);
        box.x = min(corner1.x, corner2.x);
        box.y = min(corner1.y, corner2.y);

        ldown = false;
        lup = false;
    }
}

// EXPOSURE -start----------------------------
void on_slider_exposureOnOff(int, void*) {
    FlyCapture2::Error error;
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
        FlyCapture2::Error error;
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
// EXPOSURE -end------------------------------

// SHARPNESS -start---------------------------
void on_slider_sharpnessOnOff(int, void*) {
    FlyCapture2::Error error;
    Property prop;
    prop.type = AUTO_EXPOSURE;
    error = cam.GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        PrintError( error );
    }

    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = sharpnessOnOff; 
    error = cam.SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        PrintError ( error );
    }
}

void on_slider_sharpnessValue(int, void*) {
    if (sharpnessOnOff == 1) { // AUTO mode
        setTrackbarPos(shar_value, win_setting, oldSharpnessValue);
    } else {
        FlyCapture2::Error error;
        Property prop;
        prop.type = SHARPNESS;
        error = cam.GetProperty(&prop);
        if ( error != PGRERROR_OK) {
            PrintError( error );
        }

        prop.absControl = false;
        prop.onOff = true;
        prop.autoManualMode = 0;
        prop.valueA = sharpnessValue;
        error = cam.SetProperty(&prop, false);
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            setTrackbarPos(shar_value, win_setting, oldSharpnessValue);
        } else {
            oldSharpnessValue = sharpnessValue;
        }
    }
}
// SHARPNESS -end-----------------------------

// SHUTTER -start-----------------------------
void on_slider_shutterOnOff(int, void*) {
    FlyCapture2::Error error;
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
        FlyCapture2::Error error;
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
// SHUTTER -end-------------------------------

// BINARIZATION -start------------------------
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
// BINARIZATION -end--------------------------

// OCR -start---------------------------------
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
// OCR -end-----------------------------------
