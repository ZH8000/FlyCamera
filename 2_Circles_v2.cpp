#include <FlyCapture2.h>

#include <ctime>
#include <vector>
#include <iostream>

//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace FlyCapture2;

const char* win_title = "影像";
const char* win_setting = "攝影機 設定";
const char* win_opencv = "OpenCV 設定";
const char* win_circle = "圈偵測 設定";

Camera cam;
static const unsigned int sk_numProps = 18;

const char* expo_title =  "自動曝光 Off/On";
const char* expo_value =  "手動曝光值";
const char* shar_title =  "自動影像銳利化 Off/On";
const char* shar_value =  "手動影像銳利化值";
const char* shut_title =  "自動快門 Off/On";
const char* shut_value =  "手動快門值";
const char* bina_title =  "影像二元化 Off/On";
const char* binv_title =  "影像二元化反轉 Off/On";
const char* bina_max=     "影像二元化最大接受閥值(+150)"; // binarization max value will between 0(+150) ~ 150(+150)
const char* bina_thresh = "影像二元化閥值";               // binarization thresh will between 0 ~ 150
const char* cann_title =  "Canny 測邊 Off/On";
const char* cann_max   =  "Canny 測邊最大接受閥值(+150)";
const char* cann_thresh = "Canny 測邊閥值";
const char* circ_title =  "圈偵測";

int exposureOnOff = 0;         // exposure
int exposureValue = 0;
int sharpnessOnOff = 0;        // sharpness
int sharpnessValue = 3000;
int shutterOnOff = 0;          // shutter
int shutterValue = 0;
int binaryOnOff = 0;           // binarization
int binaryInvOnOff = 0;        // binarization inverse
int binaryMax =  100;          // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
int oldBinaryMax = 100;
int binaryThresh = 60;
int oldBinaryThresh = 30;
int cannyOnOff = 0;            // canny
int cannyMax = 50;             // canny max value will between 0(150+) ~ 100(+150), p.s. actually value should plus 150, so 150 ~ 250
int oldCannyMax = 50;
int cannyThresh = 50;
int oldCannyThresh = 50;
int circleOnOff = 0;
int circleMinDist = 120;
int circleParam1 = 60;
int circleParam2 = 60;

void on_slider_exposureOnOff(int, void*);  // exposure
void on_slider_exposureValue(int, void*);
void on_slider_sharpnessOnOff(int, void*); // sharpness
void on_slider_sharpnessValue(int, void*);
void on_slider_shutterOnOff(int, void*);   // shutter
void on_slider_shutterValue(int, void*);
void on_slider_binaryMax(int, void*);      // binarization
void on_slider_binaryThresh(int, void*);
void on_slider_cannyMax(int, void*);       // canny
void on_slider_cannyThresh(int, void*);

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

void getCameraProp(Camera*);
int RunSingleCamera( PGRGuid guid );

class Find_circles : public cv::ParallelLoopBody {
    private:
        cv::Mat inImage;
        cv::Mat& outImage;
        vector<Vec3f>& circles;

    public:
        Find_circles(cv::Mat input, cv::Mat& output, vector<Vec3f>& array) :
                     inImage(input), outImage(output), circles(array) {}

        virtual void operator() (const cv::Range& range) const {
            inImage.copyTo(outImage);
            for (int x = range.start; x < range.end; x++ ) {
                HoughCircles(outImage, circles, CV_HOUGH_GRADIENT, 1, outImage.rows/8, 200, 100, 0, 0 );
            }
        }
};

int main(int argc, char** argv) {
    cout << "Press 'q' to quit" << endl;

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

    createTrackbar(expo_title, win_setting, &exposureOnOff,  1,    on_slider_exposureOnOff);
    createTrackbar(expo_value, win_setting, &exposureValue,  1023, on_slider_exposureValue);
    createTrackbar(shar_title, win_setting, &sharpnessOnOff, 1,    on_slider_sharpnessOnOff);
    createTrackbar(shar_value, win_setting, &sharpnessValue, 4095, on_slider_sharpnessValue);
    createTrackbar(shut_title, win_setting, &shutterOnOff,   1,    on_slider_shutterOnOff);
    createTrackbar(shut_value, win_setting, &shutterValue,   1590, on_slider_shutterValue);
    createTrackbar(bina_title, win_opencv,  &binaryOnOff,    1);
    createTrackbar(binv_title, win_opencv,  &binaryInvOnOff, 1);
    createTrackbar(bina_max,   win_opencv,  &binaryMax,      150,  on_slider_binaryMax);
    createTrackbar(bina_thresh,win_opencv,  &binaryThresh,   150,  on_slider_binaryThresh);
    createTrackbar(cann_title, win_opencv,  &cannyOnOff,     1);
    createTrackbar(cann_max,   win_opencv,  &cannyMax,       150,  on_slider_cannyMax);
    createTrackbar(cann_thresh,win_opencv,  &cannyThresh,     200, on_slider_cannyThresh);
    createTrackbar(circ_title, win_opencv,  &circleOnOff,    1);

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

void getCameraProp(Camera* cam) {
    const int k_numImages = 10;
    FlyCapture2::Error error;

    Property camProp;
    PropertyInfo camPropInfo;
    for (unsigned int x = 0; x < sk_numProps; x++) {
        const PropertyType k_currPropType = (PropertyType)x;
        camProp.type = k_currPropType;
        camPropInfo.type = k_currPropType;

        FlyCapture2::Error getPropErr = cam->GetProperty( &camProp );
        FlyCapture2::Error getPropInfoErr = cam->GetPropertyInfo( &camPropInfo );
        if ( getPropErr != PGRERROR_OK || getPropInfoErr != PGRERROR_OK ||  camPropInfo.present == false) {
            continue;
        }
        if (BRIGHTNESS) {
        } else
        if (camPropInfo.type == AUTO_EXPOSURE) {
            exposureOnOff = camProp.autoManualMode;
            exposureValue = camProp.valueA;

            setTrackbarPos(expo_title, win_setting, exposureOnOff);
            setTrackbarPos(expo_value, win_setting, exposureValue);
        } else
        if (camPropInfo.type == SHARPNESS) {
            sharpnessOnOff = camProp.autoManualMode;
            sharpnessValue = camProp.valueA;

            setTrackbarPos(shar_title, win_setting, sharpnessOnOff);
            setTrackbarPos(shar_value, win_setting, sharpnessValue);
        } else
        if (camPropInfo.type == GAMMA) {
        } else
        if (camPropInfo.type == SHUTTER) {
            shutterOnOff = camProp.autoManualMode;
            shutterValue = camProp.valueA;

            setTrackbarPos(shut_title, win_setting, shutterOnOff);
            setTrackbarPos(shut_value, win_setting, shutterValue);
        } else
        if (camPropInfo.type == GAIN) {
        } else
        if (camPropInfo.type == FRAME_RATE) {
        } else 
        if (camPropInfo.type == TEMPERATURE) {
        }
    }
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

    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    Image rawImage;
    Image rgbImage;
    char c;   
    while (true) {
        // Retrieve an image
        error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK) {
            PrintError( error );
            continue;
        }
        getCameraProp(&cam);

        c = waitKey(30);
        if (c == 'q') {
            return 0;
        }

        // Convert to RGB
        rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        // resize to smaller size
        //Size size = Size(800, 600);
        Size size = Size(640, 480);
        resize(image, image, size);

        if (binaryOnOff == 1) {
            if(binaryInvOnOff == 1) {
                threshold(image, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY_INV);
            } else {
                threshold(image, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY);
            }
        }

        // Canny
        if (cannyOnOff == 1) {
            Mat tmp;
            image.copyTo(tmp);
            Canny(tmp, image, cannyThresh, (cannyMax+150), 3);
        }

        // Contours with circle bound --------
        if (circleOnOff == 1) {
            int thresh = 100;
            RNG rng(12345);
            Mat threshold_output;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;

            Mat src_gray;
            cvtColor( image, src_gray, COLOR_BGR2GRAY );
            blur( src_gray, src_gray, Size(3,3) );
            /// Detect edges using Threshold
            threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
            /// Find contours
            findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

            /// Approximate contours to polygons + get bounding rects and circles
            vector<vector<Point> > contours_poly( contours.size() );
            cout << "contours.size() " << contours.size() << endl;
            //vector<Rect> boundRect( contours.size() );
            vector<Point2f>center( contours.size() );
            vector<float>radius( contours.size() );

            for( size_t i = 0; i < contours.size(); i++ ) {
                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
                minEnclosingCircle( contours_poly[i], center[i], radius[i] );
            }

            /// Draw polygonal contour + bonding rects + circles
            Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            for( size_t i = 0; i< contours.size(); i++ ) {
                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                //drawContours( drawing, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
                circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
                cout << "radius: " << (int)radius[i] << " center: "<< center[i] << endl;
            }

            /// Show in a window
            //namedWindow( "Contours", WINDOW_AUTOSIZE );
            //imshow( "Contours", drawing );

            imshow("detected circles", drawing);
        } else {
            destroyWindow("detected circles");
        }
        imshow(win_title, image);
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

void setParamAutoOnOff(PropertyType type, int onOff) {
    FlyCapture2::Error error;
    Property prop;
    prop.type = type;
    error = cam.GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        PrintError( error );
    }
    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = onOff; 
    error = cam.SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        PrintError ( error );
    }
}

void setParamValue(PropertyType type, int value) {
    FlyCapture2::Error error;
    Property prop;
    prop.type = type;
    error = cam.GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        PrintError( error );
    }

    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = 0;
    prop.valueA = value;
    error = cam.SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        PrintError( error );
    } else {
    }
}


// EXPOSURE -start----------------------------
void on_slider_exposureOnOff(int, void*) {
    setParamAutoOnOff(AUTO_EXPOSURE, exposureOnOff);
}

void on_slider_exposureValue(int, void*) {
    if (exposureOnOff == 0) { // MANUAL mode
        setParamValue(AUTO_EXPOSURE, exposureValue);
    }
}
// EXPOSURE -end------------------------------

// SHARPNESS -start---------------------------
void on_slider_sharpnessOnOff(int, void*) {
    setParamAutoOnOff(SHARPNESS, sharpnessOnOff);
}

void on_slider_sharpnessValue(int, void*) {
    if (sharpnessOnOff == 0) { // MANUAL mode
        setParamValue(SHARPNESS, sharpnessValue);
    }
}
// SHARPNESS -end-----------------------------

// SHUTTER -start-----------------------------
void on_slider_shutterOnOff(int, void*) {
    setParamAutoOnOff(SHUTTER, shutterOnOff);
}

void on_slider_shutterValue(int, void*) {
    if (shutterOnOff == 0) { // MANUAL mode
        setParamValue(SHUTTER, shutterValue);
    }   
}
// SHUTTER -end-------------------------------

// BINARIZATION -start------------------------
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
// CANNY -start-------------------------------
void on_slider_cannyMax(int, void*) {
    if (cannyOnOff == 0) {
        setTrackbarPos(cann_max, win_opencv, oldCannyMax);
        setTrackbarPos(circ_title, win_opencv, 0);
    } else {
        oldCannyMax = cannyMax;
    }
}

void on_slider_cannyThresh(int, void*) {
    if (cannyOnOff == 0) {
        setTrackbarPos(cann_thresh, win_opencv, oldCannyThresh);
    } else {
        oldCannyThresh = cannyThresh;
    }
}
// CANNY -end---------------------------------
