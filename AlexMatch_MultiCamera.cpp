#include <FlyCapture2.h>

#include <ctime>
#include <vector>
#include <iostream>
#include <list>
#include <sstream>
#include <string>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <tesseract/baseapi.h>
#include <tesseract/strngs.h>

using namespace std;
using namespace cv;
using namespace FlyCapture2;

const float inlier_threshold = 2.5f; 
const float nn_match_ratio = 0.8f;   
const char* win_title = "影像";
const char* win_setting = "攝影機 設定";
const char* win_opencv = "OpenCV 設定";
const char* win_akaze = "AKAZE 比對結果";

stringstream ss_title;
stringstream ss_setting;
stringstream ss_opencv;
stringstream ss_akaze;

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
int binaryThresh = 30;
int oldBinaryThresh = 30;
// int blurOnOff = 0;             // Gaussian Blur
// int blurValue = 0;
// int oldBlurValue = 0;
int successMatches = 100;      // AKAZE matches
int ocrOnOff = 0;              // OCR

int SerialNumber = 0;
Camera cam;
Mat sampleImage;
list<Mat> sampleImages;
const int sampleImagesSize = 10;
int sampleImagesFlag = 0;
Mat targetImage;
static const unsigned int sk_numProps = 18;

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

void on_slider_exposureOnOff(int, void*);  // exposure
void on_slider_exposureValue(int, void*);
void on_slider_sharpnessOnOff(int, void*); // sharpness
void on_slider_sharpnessValue(int, void*);
void on_slider_shutterOnOff(int, void*);   // shutter
void on_slider_shutterValue(int, void*);
void on_slider_binaryMax(int, void*);      // binarization
void on_slider_binaryThresh(int, void*);
void on_slider_ocrOnOff(int, void*);       // OCR
void OCR(Mat*);

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

void PrintFormat7Capabilities(Format7Info fmt7Info) {
    cout << "Max image pixel: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
    cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
    cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
    cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
}

void PrintFormat7ImageSettings(Format7ImageSettings fmt7Settings) {
    cout << "offsetX: " << fmt7Settings.offsetX << endl;
    cout << "offsetY: " << fmt7Settings.offsetY << endl;
    cout << "width:" << fmt7Settings.width << endl;
    cout << "height:" << fmt7Settings.height << endl;
}

void getCameraProp(Camera*);
int RunSingleCamera( PGRGuid guid );

int main(int argc, char** argv)
{
    if (argc != 2) {
        cout << "Please input correct Serial Number of Camera." << endl;
        return -1;
    } else {
        istringstream ss(argv[1]);
        if (!(ss >> SerialNumber)) {
            cerr << "Invalid number " << argv[1] << '\n';
            return -1;
        }
    }

    cout << "Press 'q' to quit" << endl;
    cout << "Press 's' to start sampling" << endl;
    cout << "Press 'r' to reset sample" << endl;

    FlyCapture2::Error error;
    BusManager busMgr;
    unsigned int numCameras;
    const Mode k_fmt7Mode = MODE_0;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;

    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    ss_title << win_title << SerialNumber;
    ss_setting << win_setting << SerialNumber;
    ss_opencv << win_opencv << SerialNumber;

    namedWindow(ss_title.str(), WINDOW_NORMAL);
    namedWindow(ss_setting.str(), WINDOW_NORMAL);
    namedWindow(ss_opencv.str(), WINDOW_NORMAL);

    createTrackbar(expo_title, ss_setting.str(), &exposureOnOff, 1, on_slider_exposureOnOff);
    createTrackbar(expo_value, ss_setting.str(), &exposureValue, 1023, on_slider_exposureValue);
    createTrackbar(shar_title, ss_setting.str(), &sharpnessOnOff, 1, on_slider_sharpnessOnOff);
    createTrackbar(shar_value, ss_setting.str(), &sharpnessValue, 4095, on_slider_sharpnessValue);
    createTrackbar(shut_title, ss_setting.str(), &shutterOnOff, 1, on_slider_shutterOnOff);
    createTrackbar(shut_value, ss_setting.str(), &shutterValue, 1590, on_slider_shutterValue);
    createTrackbar(bina_title, ss_opencv.str(), &binaryOnOff, 1);
    createTrackbar(binv_title, ss_opencv.str(), &binaryInvOnOff, 1);
    createTrackbar(bina_max, ss_opencv.str(), &binaryMax, 150, on_slider_binaryMax);
    createTrackbar(bina_thresh, ss_opencv.str(), &binaryThresh, 150, on_slider_binaryThresh);
    createTrackbar(succ_matches, ss_opencv.str(), &successMatches, 4000);
    createTrackbar(tess_title, ss_opencv.str(), &ocrOnOff, 1);

    Camera** ppCameras = new Camera*[numCameras];
    for (unsigned int i=0; i < numCameras; i++) {
        ppCameras[i] = new Camera();
        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1; 
        }

        // connect to the camera to get S.N.
        error = ppCameras[i]->Connect( &guid );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
        CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
        if (camInfo.serialNumber == SerialNumber) {
            cout << "Find the camera: " << SerialNumber << endl;

            // 1. get Format7Info
            Format7Info fmt7Info;
            bool supported;
            fmt7Info.mode = k_fmt7Mode;
            error = ppCameras[i] -> GetFormat7Info(&fmt7Info, &supported);
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

            error = ppCameras[i] -> ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
            if (error != PGRERROR_OK) {
                PrintError(error);
                return -1;
            }
            if (!valid) {
                cout << "Format7 settings are not valid" << endl;
            }
/*
            unsigned int packetSize;
            float percentage;
            error = ppCameras[i] -> GetFormat7Configuration(&fmt7ImageSettings, &packetSize, &percentage);
            if (error != PGRERROR_OK) {
                PrintError(error);
                return -1;
            }
            cout << "Format7ImageSettings: packetSize: " << packetSize << endl;
            PrintFormat7ImageSettings(fmt7ImageSettings);
 */

            // 3. setFormat7Configuration
            //const unsigned int bestPacketSize = 1968;
            const unsigned int bestPacketSize = 984;
            //error = ppCameras[i] -> SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);
            error = ppCameras[i] -> SetFormat7Configuration(&fmt7ImageSettings, bestPacketSize);
            if (error != PGRERROR_OK) {
                PrintError(error);
                return -1;
            }

            RunSingleCamera( guid );
        } else {
            cout << "Camera NOT corrent: " << camInfo.serialNumber << endl;
        }
    }   

    return 0;
}

void Match(Mat& sample, int idx) {
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
    akaze->detectAndCompute(sample, noArray(), kpts1, desc1);
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
    drawMatches(sample, inliers1, targetImage, inliers2, good_matches, res);
    Point pt = Point(100, 100);
    if (matched1.size() >= successMatches) {
        putText(res, "Succeed!", pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
    } else {
        putText(res, "Failed", pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
    }
    stringstream ss;
    ss << idx;
    namedWindow(ss.str(), WINDOW_NORMAL);
    imshow(ss.str(), res);

    double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
    cout << "Alex Matching Results" << endl;
    cout << "*******************************" << endl;
    cout << "# Keypoints 1:    \t" << kpts1.size() << endl;
    cout << "# Keypoints 2:    \t" << kpts2.size() << endl;
    cout << "# Matches:        \t" << matched1.size() << endl;
    cout << "# Inliers:        \t" << inliers1.size() << endl;
    cout << "# Inliers Ratio:  \t" << inlier_ratio << endl;
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

            setTrackbarPos(expo_title, ss_setting.str(), exposureOnOff);
            setTrackbarPos(expo_value, ss_setting.str(), exposureValue);
        } else
        if (camPropInfo.type == SHARPNESS) {
            sharpnessOnOff = camProp.autoManualMode;
            sharpnessValue = camProp.valueA;

            setTrackbarPos(shar_title, ss_setting.str(), sharpnessOnOff);
            setTrackbarPos(shar_value, ss_setting.str(), sharpnessValue);
        } else
        if (camPropInfo.type == GAMMA) {
        } else
        if (camPropInfo.type == SHUTTER) {
            shutterOnOff = camProp.autoManualMode;
            shutterValue = camProp.valueA;

            setTrackbarPos(shut_title, ss_setting.str(), shutterOnOff);
            setTrackbarPos(shut_value, ss_setting.str(), shutterValue);
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

        if (c == 'r') {
             sampleImages.clear();
             sampleImagesFlag = 0;
             cout << "-------Reset Sampling-------" << endl;
         }

        // Convert to RGB
        rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        // resize to smaller size
        Size size = Size(320, 240);
        resize(image, image, size);

        if (binaryOnOff == 1) {
            if(binaryInvOnOff == 1) {
                threshold(image, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY_INV);
            } else {
                threshold(image, image, binaryThresh, (binaryMax+150), CV_THRESH_BINARY);
            }
        }

        if (c == 's') {
            if (sampleImages.size() < sampleImagesSize) {
                sampleImages.push_back(image);
                image.copyTo(sampleImage);
                cout << "-----Get Sampled Image #" << (++sampleImagesFlag) << "-----" << endl;
            } else {
                image.copyTo(targetImage);

                // print start time
                time_t t_s = time(0);
                struct tm * now = localtime( &t_s );
                cout << now->tm_hour << ":" << now->tm_min << ":"<< now->tm_sec << "------------ START" << endl;

                clock_t start, end;
                double duration;
                start = clock();

                list<Mat>::iterator i;
                for (i = sampleImages.begin(); i != sampleImages.end(); ++i) {
                    int idx =  distance(sampleImages.begin(), i);
                    Match(*i, idx);
                }

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

        imshow(ss_title.str(), image);
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
        setTrackbarPos(bina_max, ss_opencv.str(), oldBinaryMax);
    } else {
        oldBinaryMax = binaryMax;
    }
}

void on_slider_binaryThresh(int, void*) {
    if (binaryOnOff == 0) {
        setTrackbarPos(bina_thresh, ss_opencv.str(), oldBinaryThresh);
    } else {
        oldBinaryThresh = binaryThresh;
    }
}
// BINARIZATION -end--------------------------
// OCR -start---------------------------------
void OCR(Mat *image) {
    Mat newImage;
    RNG rng(12345);
    //Mat tmp = imread("../Lenna.png");;
    Mat tmp;
    image->copyTo(tmp);   
    cvtColor(tmp, tmp, CV_BGR2GRAY);
    tmp.convertTo(tmp, CV_8UC1);

    vector< vector<Point> > contours;
    vector< vector<Point> > contoursLow;
    vector<Vec4i> hierarchy;
    findContours(tmp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    // findContours(tmp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    // approxPolyDP(contours, contoursLow, 1, true);
    for(int x = 0; x < contours.size(); x++) {
        approxPolyDP(contours[x], contours[x], 1, true);
    }
   
    Moments moments;
    double humoments[7];

    Mat drawing = Mat::zeros(image->size(), CV_8UC3);
    for (int x = 0; x < contours.size(); x++) {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, x, color, 2, 8, hierarchy, 0, Point() );
        Rect rect;
        Point pt1, pt2;

        rect = boundingRect(contours[x]);
        pt1.x = rect.x;
        pt2.x = (rect.x + rect.width);
        pt1.y = rect.y;
        pt2.y = (rect.y + rect.height);
        rectangle(drawing, pt1, pt2, color, 1, 8, 0);

        //moments = moments(contours[x]);
        //HuMoments(moments, humoments);
    }
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );


    tesseract::TessBaseAPI tess;
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);

    tess.SetImage((uchar*)image->data, image->cols, image->rows, 1, image->cols);
    char* out = tess.GetUTF8Text();

    ocrOnOff = 0;
    setTrackbarPos(tess_title, ss_opencv.str(), ocrOnOff);

    cout << out << endl;
    cout << "-----------------------------" << endl;
}
// OCR -end-----------------------------------
