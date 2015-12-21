#include <FlyCapture2.h>

#include <ctime>
#include <vector>
#include <iostream>
#include <list>

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
list<Mat> sampleImages;
const int sampleImagesSize = 10;
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
const char* cont_title1 = "contour 面積下限";
const char* cont_title2 = "contour 面積上限";
const char* cann_title =  "Canny 測邊 Off/On";
const char* cann_max   =  "Canny 測邊最大接受閥值(+150)";
const char* cann_thresh = "Canny 測邊閥值";
const char* circ_title =  "圈偵測";
const char* line_left = "左邊界";                                                                                                                                                                              
const char* line_right = "右邊界";
const char* line_top = "上邊界";
const char* line_bottom = "下邊界";

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
int binaryThresh = 100;
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
int contourAreaFilterLow = 10000;
int contourAreaFilterHigh = 40000;
int leftValue = 0;             // draw lines.
int rightValue = 640;
int topValue = 0;
int bottomValue = 480;

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
 
void drawLine(Mat, Point, Point); // drawLine

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
    createTrackbar(bina_thresh,win_opencv,  &binaryThresh,   200,  on_slider_binaryThresh);

    createTrackbar(cont_title1, win_opencv, &contourAreaFilterLow, 90000);
    createTrackbar(cont_title2, win_opencv, &contourAreaFilterHigh, 90000);

    createTrackbar(line_left,  win_title,   &leftValue,      640);
    createTrackbar(line_right, win_title,   &rightValue,     640);
    createTrackbar(line_top,   win_title,   &topValue,       480);
    createTrackbar(line_bottom,win_title,   &bottomValue,    480);

    // createTrackbar(cann_title, win_opencv,  &cannyOnOff,     1);
    // createTrackbar(cann_max,   win_opencv,  &cannyMax,       150,  on_slider_cannyMax);
    // createTrackbar(cann_thresh,win_opencv,  &cannyThresh,     200, on_slider_cannyThresh);

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
    int flag = -1;
    while (true) {
        // Retrieve an image
        error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK) {
            PrintError( error );
            continue;
        }
        getCameraProp(&cam);

        c = waitKey(100);
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

        // draw lines, will crop the image.
        drawLine(image, Point(leftValue, 0), Point(leftValue, 480));     // left
        drawLine(image, Point(rightValue, 0), Point(rightValue, 480));   // right
        drawLine(image, Point(0, topValue), Point(640, topValue));       // top
        drawLine(image, Point(0, bottomValue), Point(640, bottomValue)); // bottom
        image = image(Rect(leftValue,topValue, rightValue-leftValue, bottomValue-topValue));
        // Contours with circle bound --------
        if (c == 's') {
            // print start time
            time_t t_s = time(0);
            struct tm * now = localtime( &t_s );
            cout << now->tm_hour << ":" << now->tm_min << ":"<< now->tm_sec << "------------ START" << endl;

            clock_t start, end;
            double duration;
            start = clock();

            int thresh = 100;
            RNG rng(12345);
            Mat threshold_output;
            vector< vector<Point> > contours;
            vector<Vec4i> hierarchy;

            Mat src_gray;
            cvtColor(image, src_gray, COLOR_BGR2GRAY);
            blur(src_gray, src_gray, Size(3, 3));

            // Detec edges using Threshold
            threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY);
 
            // Find contours
            findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
 
            /// Approximate contours to polygons + get bounding rects and circles
            vector<vector<Point> > contours_poly( contours.size() );
            vector<Point2f>center( contours.size() );
            vector<float>radius( contours.size() );

            for ( size_t i = 0; i < contours.size(); i++ ) {
                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                minEnclosingCircle( contours_poly[i], center[i], radius[i] );
            }

            vector< vector<Point> > result_contours;
            vector< Point2f > result_center;
            vector< float > result_radius;

            /// Draw polygonal contour + bonding rects + circles
            Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            for ( size_t i = 0; i< contours.size(); i++ ) {
                if (contourArea(contours[i]) < contourAreaFilterHigh && contourArea(contours[i]) > contourAreaFilterLow) {
                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                    drawContours( drawing, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                    //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
                    circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
                    cout << "~~" << i << "~~" << endl;
                    cout << "radius: " << (int)radius[i] << endl;
                    result_contours.push_back(contours[i]);
                    cout << "center: " << center[i] << endl;
                    result_center.push_back(center[i]);
                    cout << "area:   " << contourArea(contours[i]) << endl;
                    result_radius.push_back(radius[i]);
                    cout << "~~~~~" << endl;
                }
            }
            if ( result_contours.size() != 2) {
                Point text = Point(50, 50);
                putText(drawing, "X", text, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
            } else {
                double center_distance = sqrt( pow((result_center.at(0).x-result_center.at(1).x), 2) + pow( pow((result_center.at(0).y-result_center.at(1).y), 2), 2) );
                double radius_diff = abs(result_radius.at(0) - result_radius.at(1));
                cout << "radius 0: " << result_radius.at(0) << endl;
                cout << "radius 1: " << result_radius.at(1) << endl;
                cout << "    diff: " << radius_diff << " ? " << (result_radius.at(0) - result_radius.at(1)) << endl;
                cout << center_distance << "  " << radius_diff << endl;
                double bigger_radius = result_radius.at(0);
                double smaller_radius = result_radius.at(1);
                if (result_radius.at(1) > result_radius.at(0)) { 
                    bigger_radius = result_radius.at(1); 
                    smaller_radius = result_radius.at(0);
                }
                /*
                if ( center_distance > (radius_diff/2) || radius_diff < (bigger_radius/4) ) {
                    Point text = Point(50, 50);
                    putText(drawing, "X", text, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
                } else {
                    Point text = Point(50, 50);
                    putText(drawing, "O", text, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
                }*/
                if (center_distance < (radius_diff/2) && radius_diff > (bigger_radius/4) && radius_diff < (smaller_radius/2)) {
                    cout << "center_distance: " << center_distance << endl;
                    cout << "(radius_diff/2): " << (radius_diff/2) << endl;
                    cout << "radius_diff: " << radius_diff << endl;
                    cout << "(bigger_radius/4): " << (bigger_radius/4) << endl;
                    Point text = Point(50, 50);
                    putText(drawing, "O", text, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
                } else {
                    Point text = Point(50, 50);
                    putText(drawing, "X", text, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
                }
            }

            imshow("detected circles", drawing);

            end = clock();
            duration = (double)(end - start) / CLOCKS_PER_SEC;
            cout << "Duration: "  << duration << endl;
            // print end time
            time_t t_e = time(0);
            struct tm * now_e = localtime( &t_e );
            cout << now->tm_hour << ":" << now->tm_min << ":"<< now->tm_sec << "------------ END" << endl;

            //double result = sqrt( pow((center[1].x-center[2].x), 2) + pow( pow((center[1].y-center[2].y), 2), 2) );
            //cout << "center distance: " << result << endl;
            //cout << "radius difference: " << abs(radius[1] - radius[2]) << endl;
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
// drawLine -start----------------------------
void drawLine(Mat img, Point start, Point end) {
    int thickness = 2;
    int lineType = 8;
    int shift = 0;
    line(img, start, end, Scalar(0, 0, 0), thickness, lineType, shift);
}

