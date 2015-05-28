#include <FlyCapture2.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;
using namespace FlyCapture2;

const float inlier_threshold = 2.5f; 
const float nn_match_ratio = 0.8f;   
const char* win_title = "影像";
const char* win_setting = "攝影機 設定";
const char* win_opencv = "OpenCV 設定";
const char* win_akaze = "AKAZE 比對結果";

int exposureOnOff = 0;         // exposure
int exposureValue = 0;
int sharpnessOnOff = 0;        // sharpness
int sharpnessValue = 3000;
int shutterOnOff = 0;          // shutter
int shutterValue = 0;
int binaryOnOff = 0;           // binarization
int binaryMax =  100;          // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
int oldBinaryMax = 100;
int binaryThresh = 30;
int oldBinaryThresh = 30;
// int blurOnOff = 0;             // Gaussian Blur
// int blurValue = 0;
// int oldBlurValue = 0;
int successMatches = 100;      // AKAZE matches

Camera cam;
static const unsigned int sk_numProps = 18;
bool sampled = false;

const char* expo_title = "自動曝光 Off/On";
const char* expo_value = "手動曝光值";
const char* shar_title = "自動影像銳利化 Off/On";
const char* shar_value = "手動影像銳利化值";
const char* shut_title = "自動快門 Off/On";
const char* shut_value = "手動快門值";
const char* bina_title = "影像二元化 Off/ On";
const char* bina_max = "影像二元化最大接受閥值(+150)"; // binarization max value will between 0(+150) ~ 150(+150)
const char* bina_thresh = "影像二元化閥值";            // binarization thresh will between 0 ~ 150
const char* blur_title = "高斯模糊 Off/On";
const char* blur_value = "高斯模糊 Kernel 大小";
const char* succ_matches = "辦別成功最低Match值";

void on_slider_exposureOnOff(int, void*);  // exposure
void on_slider_exposureValue(int, void*);
void on_slider_sharpnessOnOff(int, void*); // sharpness
void on_slider_sharpnessValue(int, void*);
void on_slider_shutterOnOff(int, void*);   // shutter
void on_slider_shutterValue(int, void*);
void on_slider_binaryMax(int, void*);      // binarization
void on_slider_binaryThresh(int, void*);

Mat sampleImage;
Mat targetImage;

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

void getCameraProp(Camera*);
int RunSingleCamera( PGRGuid guid );

int main(int argc, char** argv)
{
    cout << "Press 'q' to quit" << endl;
    cout << "Press 's' to start sampling" << endl;
    cout << "Press 'r' to reset sample" << endl;

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

    createTrackbar(expo_title, win_setting, &exposureOnOff, 1, on_slider_exposureOnOff);
    createTrackbar(expo_value, win_setting, &exposureValue, 1023, on_slider_exposureValue);
    createTrackbar(shar_title, win_setting, &sharpnessOnOff, 1, on_slider_sharpnessOnOff);
    createTrackbar(shar_value, win_setting, &sharpnessValue, 4095, on_slider_sharpnessValue);
    createTrackbar(shut_title, win_setting, &shutterOnOff, 1, on_slider_shutterOnOff);
    createTrackbar(shut_value, win_setting, &shutterValue, 1590, on_slider_shutterValue);
    createTrackbar(bina_title, win_opencv, &binaryOnOff, 1);
    createTrackbar(bina_max, win_opencv, &binaryMax, 150, on_slider_binaryMax);
    createTrackbar(bina_thresh, win_opencv, &binaryThresh, 150, on_slider_binaryThresh);
    createTrackbar(succ_matches, win_opencv, &successMatches, 4000);

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

void Match() {
   
    // Mat img1 = imread("sample.png", IMREAD_GRAYSCALE);
    // Mat img2 = imread("target.png", IMREAD_GRAYSCALE);
    // Mat img1 = imread("../16v1500uF/0.png", IMREAD_GRAYSCALE);
    // Mat img2 = imread("../400v68uF/3.png", IMREAD_GRAYSCALE);
    // sampleImage = imread("../16v1500uF/0.png", IMREAD_GRAYSCALE);
    // targetImage = imread("../400v68uF/3.png", IMREAD_GRAYSCALE);

#if 0
 int sigma = 0.3 * ((5 - 1) * 0.5 - 1) + 0.8;
    GaussianBlur(imag1, img1, Size(3, 3), sigma);
    GaussianBlur(imag2, img2, Size(3, 3), sigma);

    Canny(img1, img1, 10, 50, 3);
    Canny(img2, img2, 10, 50, 3);
#endif    
    Mat homography;
    FileStorage fs("../H1to3p.xml", FileStorage::READ);
    fs.getFirstTopLevelNode() >> homography;

    vector<KeyPoint> kpts1, kpts2;
    Mat desc1, desc2;

    Ptr<AKAZE> akaze = AKAZE::create();
    akaze->detectAndCompute(sampleImage, noArray(), kpts1, desc1);
    akaze->detectAndCompute(targetImage, noArray(), kpts2, desc2);

    BFMatcher matcher(NORM_HAMMING);
    vector< vector<DMatch> > nn_matches;
    matcher.knnMatch(desc1, desc2, nn_matches, 2);

    vector<KeyPoint> matched1, matched2, inliers1, inliers2;
    vector<DMatch> good_matches;
    for(size_t i = 0; i < nn_matches.size(); i++) {
        DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;

        if(dist1 < nn_match_ratio * dist2) {
            matched1.push_back(kpts1[first.queryIdx]);
            matched2.push_back(kpts2[first.trainIdx]);
        }
    }

    for(unsigned i = 0; i < matched1.size(); i++) {
        Mat col = Mat::ones(3, 1, CV_64F);
        col.at<double>(0) = matched1[i].pt.x;
        col.at<double>(1) = matched1[i].pt.y;

        col = homography * col;
        col /= col.at<double>(2);
        double dist = sqrt( pow(col.at<double>(0) - matched2[i].pt.x, 2) +
                            pow(col.at<double>(1) - matched2[i].pt.y, 2));

        if(dist < inlier_threshold) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            good_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }

    Mat res;
    drawMatches(sampleImage, inliers1, targetImage, inliers2, good_matches, res);
    Point pt = Point(100, 100);
    if (matched1.size() >= successMatches) {
        putText(res, "Succeed!", pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
    } else {
        putText(res, "Failed", pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
    }
    //imwrite("res.png", res);
    namedWindow(win_akaze, WINDOW_NORMAL);
    imshow(win_akaze, res);

    double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
    cout << "Alex Matching Results" << endl;
    cout << "*******************************" << endl;
    cout << "# Keypoints 1:                        \t" << kpts1.size() << endl;
    cout << "# Keypoints 2:                        \t" << kpts2.size() << endl;
    cout << "# Matches:                            \t" << matched1.size() << endl;
    cout << "# Inliers:                            \t" << inliers1.size() << endl;
    cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
    cout << endl;
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

//    getCameraProp(&cam);

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
        c = waitKey(30);

        getCameraProp(&cam);
        if (c == 'q') {
            return 0;
        }
        if (c == 'r') {
            sampled = false;
            cout << "-------Reset Sampling-------" << endl;
        }

        // Retrieve an image
        error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK) {
            PrintError( error );
            continue;
        }

        // Convert to RGB
        rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        if (binaryOnOff == 1) {
            // threshold(origImage, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY_INV);
            threshold(image, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY);
        }

        // handle mouse click event
        /*
        if (ldown == true && lup == false) {
            Point pt;
            pt.x = x;
            pt.y = y;
        }*/

        imshow(win_title, image);

        if (c == 's') {
            if (!sampled) {
                // imwrite("sample.png", image);
                image.copyTo(sampleImage);
                sampled = true;
                cout << "-----Get Sampled Image-----" << endl;
            } else {
                // imwrite("target.png", image);
                image.copyTo(targetImage);

                // print start time
                time_t t_s = time(0);
                struct tm * now = localtime( &t_s );
                cout << now->tm_hour << ":" << now->tm_min << ":"<< now->tm_sec << "------------ START" << endl;

                clock_t start, end;
                double duration;
                start = clock();

                Match();

                end = clock();
                duration = (double)(end - start) / CLOCKS_PER_SEC;
                cout << "Duration: "  << duration << endl;

                // print end time
                time_t t_e = time(0);
                struct tm * now_e = localtime( &t_e );
                cout << now->tm_hour << ":" << now->tm_min << ":"<< now->tm_sec << "------------ END" << endl;

                // namedWindow("AKAZE 比對結果", WINDOW_NORMAL);
                // Mat res = imread("res.png", CV_LOAD_IMAGE_COLOR);
                // imshow("AKAZE 比對結果", res);
            }
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
    if (shutterOnOff == 1) { // MANUAL mode
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
