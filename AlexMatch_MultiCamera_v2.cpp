#include "AlexMatch_MultiCamera_v2.hpp"

using namespace std;
using namespace cv;
using namespace FlyCapture2;


Camera cam;
map<unsigned int, CameraProp> propMap;
Mat sampleImage;
list<Mat> sampleImages;
const int sampleImagesSize = 10;
int sampleImagesFlag = 0;
Mat targetImage;
static const unsigned int sk_numProps = 18;

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

Camera** ppCameras;
unsigned int numCameras;

int main(int argc, char** argv) {
    cout << "Press 'q' to quit" << endl;
    cout << "Press 's' to start sampling" << endl;
    cout << "Press 'r' to reset sample" << endl;

    CommonFlySDK sdk;

    FlyCapture2::Error error;
    BusManager busMgr;


    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    ppCameras = new Camera*[numCameras];

    // Connect to all detected cameras and attempt to set them to
    // a common video mode and frame rate
    for (unsigned int i=0; i < numCameras; i++) {
        ppCameras[i] = new Camera();

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1; 
        }
        //RunSingleCamera( guid );
        // 1. Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }

        // 2. Get the camera information
        CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }
        sdk.PrintCameraInfo( &camInfo );

        // 3. Set all cameras to a specific mode and frame rate so they can be synchronized
        error = ppCameras[i]->SetVideoModeAndFrameRate( 
            VIDEOMODE_1280x960Y8, 
            FRAMERATE_60 );
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            cout << "Error starting cameras. " << endl;
            cout << "This example requires cameras to be able to set to 1280x960 Y8 at 60fps. " << endl;
            cout << "If your camera does not support this mode, please edit the source code and recompile the application. " << endl;
            cout << "Press Enter to exit. " << endl;

            cin.ignore();
            return -1;
        }

        // 4. show window(s)
        createTrackbars(camInfo.serialNumber, &propMap[camInfo.serialNumber]);

        // 5. Start capturing images
        error = ppCameras[i]->StartCapture();
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }
    }
    
    char c;
    int count = 0;
    while ( true ) {
        c = waitKey(20);
        // clean memory and close all windows
        if ( c == 27) {
            for (unsigned int x = 0; x < numCameras; x++) {
                destroyAllWindows();
                ppCameras[x]->StopCapture();
                ppCameras[x]->Disconnect();
                delete ppCameras[x];
            }
            delete [] ppCameras;
            return 0;
        }
        
        for ( unsigned int x = 0; x < numCameras; x++) {
            CameraInfo camInfo;
            ppCameras[x]->GetCameraInfo( &camInfo );
//            getCameraProp(ppCameras[x]);
            propMap[camInfo.serialNumber] = sdk.getCameraProp(ppCameras[x], camInfo.serialNumber);
            // TODO update trackbar
            updateTrackbars(ppCameras[x], &propMap[camInfo.serialNumber]);
            
            Image rawImage;
            error = ppCameras[x]->RetrieveBuffer( &rawImage);
            if ( error != PGRERROR_OK ) {
                PrintError( error );
            }

            Image rgbImage;
            rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );

            // convert to OpenCV Mat
            unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
            Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

            stringstream ss;
            ss << camInfo.serialNumber;
            imshow(ss.str(), image);

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
    //akaze->detectAndCompute(sampleImage, noArray(), kpts1, desc1);
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
    //drawMatches(sampleImage, inliers1, targetImage, inliers2, good_matches, res);
    drawMatches(sample, inliers1, targetImage, inliers2, good_matches, res);
    Point pt = Point(100, 100);
    if (matched1.size() >= successMatches) {
        putText(res, "Succeed!", pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
    } else {
        putText(res, "Failed", pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
    }
    //imwrite("res.png", res);
    //namedWindow(win_akaze, WINDOW_NORMAL);
    //imshow(win_akaze, res);
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

void createTrackbars(unsigned int id, CameraProp *prop) {
    stringstream ss;

    // 1. for Mat image
    ss.str(std::string());
    ss << win_title << id;
    namedWindow(ss.str(), WINDOW_NORMAL);

    // 2. for Camera settings
    ss.str(std::string());
    ss << win_setting << id;
    namedWindow(ss.str(), WINDOW_NORMAL);
    
    createTrackbar(expo_title, ss.str(), &(prop->exposureOnOff), 1, on_slider_exposureOnOff, prop);
    createTrackbar(expo_value, ss.str(), &(prop->exposureValue), 1023, on_slider_exposureValue, prop);
    createTrackbar(shar_title, ss.str(), &(prop->sharpnessOnOff), 1, on_slider_sharpnessOnOff);
    createTrackbar(shar_value, ss.str(), &(prop->sharpnessValue), 4095, on_slider_sharpnessValue);
    createTrackbar(shut_title, ss.str(), &(prop->shutterOnOff), 1, on_slider_shutterOnOff);
    createTrackbar(shut_value, ss.str(), &(prop->shutterValue), 1590, on_slider_shutterValue);

    // 3. for OpenCV features
    ss.str(std::string());
    ss << win_opencv << id;
    namedWindow(ss.str(), WINDOW_NORMAL);
    createTrackbar(bina_title,  ss.str(), &(prop->binaryOnOff), 1);
    createTrackbar(binv_title,  ss.str(), &(prop->binaryInvOnOff), 1);
    createTrackbar(bina_max,    ss.str(), &(prop->binaryMax), 150, on_slider_binaryMax);
    createTrackbar(bina_thresh, ss.str(), &(prop->binaryThresh), 150, on_slider_binaryThresh);
    createTrackbar(succ_matches, win_opencv, &successMatches, 4000);
    // createTrackbar(tess_title, win_opencv, &ocrOnOff, 1);
}

void updateTrackbars(Camera* camera, CameraProp* prop) {
    const unsigned int sk_numProps = 18;
    
    // for Camera settings
    stringstream ss;
    ss.str(std::string());
    ss << win_setting << prop->camId;
    
    Property camProp;
    PropertyInfo camPropInfo;
    for (unsigned int x = 0; x < sk_numProps; x++) {
        const PropertyType k_currPropType = (PropertyType)x;
        camProp.type = k_currPropType;
        camPropInfo.type = k_currPropType;

        FlyCapture2::Error getPropErr = camera->GetProperty( &camProp );
        FlyCapture2::Error getPropInfoErr = camera->GetPropertyInfo( &camPropInfo );
        if ( getPropErr != PGRERROR_OK || getPropInfoErr != PGRERROR_OK ||  camPropInfo.present == false) {
            continue;
        }
        if (camPropInfo.type == BRIGHTNESS) {
        cout << "updateTrackbars" << endl;
            prop->brightnessOnOff = camProp.autoManualMode;
            prop->brightnessValue = camProp.valueA;
            cout << prop->camId << " BRIGHTNESS " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == AUTO_EXPOSURE) {
            prop->exposureOnOff = camProp.autoManualMode;
            prop->exposureValue = camProp.valueA;
            cout << prop->camId << " AUTO_EXPOSURE " << camProp.valueA << endl;
            cout << ss.str() << endl;
            
            setTrackbarPos(expo_title, ss.str(), prop->exposureOnOff);
            setTrackbarPos(expo_value, ss.str(), prop->exposureValue);
        } else
        if (camPropInfo.type == SHARPNESS) {
            prop->sharpnessOnOff = camProp.autoManualMode;
            prop->sharpnessValue = camProp.valueA;
            cout << prop->camId << " SHARPNESS " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == GAMMA) {
            prop->gammaOnOff = camProp.autoManualMode;
            prop->gammaValue = camProp.valueA;
            cout << prop->camId << " GAMMA " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == SHUTTER) {
            prop->shutterOnOff = camProp.autoManualMode;
            prop->shutterValue = camProp.valueA;
            cout << prop->camId << " SHUTTER " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == GAIN) {
            prop->gainOnOff = camProp.autoManualMode;
            prop->gainValue = camProp.valueA;
            cout << prop->camId << " GAIN " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == FRAME_RATE) {
            prop->frameOnOff = camProp.autoManualMode;
            prop->frameValue = camProp.valueA;
            cout << prop->camId << " FRAME_RATE " << camProp.valueA << endl;
        } else 
        if (camPropInfo.type == TEMPERATURE) {
            cout << prop->camId << " TEMPERATURE " << camProp.valueA << " K" << endl;
        }
    }
}

/*
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
        if (c == 27) {
            cam.StopCapture();
            cam.Disconnect();
            delete &cam;
            destroyAllWindows();
            return 0;
        }

        if (c == 'r') {
             sampleImages.clear();
             sampleImagesFlag = 0;
             cout << "-------Reset Sampling-------" << endl;
             for (int x = 0; x < sampleImagesSize; x++) {
                stringstream ss;
                ss << x;
                destroyWindow(ss.str());
             }
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
*/

void setParamAutoOnOff(PropertyType type, int onOff, unsigned int camId) {
    FlyCapture2::Error error;
    
    unsigned int cam = 0;
    
    for (int x = 0; x < numCameras; x++) {
        CameraInfo camInfo;
        error = ppCameras[x]->GetCameraInfo( &camInfo );
        if (error != PGRERROR_OK) {
            PrintError( error );
        }
        if (camInfo.serialNumber == camId) {
            cam = x;
            
            cout << "serial number " << camInfo.serialNumber << endl;
            break;
        }
    }
        
    Property prop;
    prop.type = type;
    error = ppCameras[cam]->GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        PrintError( error );
        cout << "setParamAutoOnOff error1" << endl;
    }
    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = onOff; 
    error = ppCameras[cam]->SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        PrintError ( error );
        cout << "setParamAutoOnOff error2" << endl;
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
void on_slider_exposureOnOff(int, void* userdata) {
    setParamAutoOnOff(AUTO_EXPOSURE, ((CameraProp*)userdata)->exposureOnOff, ((CameraProp*)userdata)->camId);
    //cout << "AUTO_EXPOSURE on/off " << ((CameraProp*)userdata)->exposureOnOff << endl;
}

void on_slider_exposureValue(int, void* userdata) {
    cout << "adsfadfadsfadsf " << ((CameraProp*)userdata)->exposureOnOff << endl;
    if ( ((CameraProp*)userdata)->exposureOnOff == 0 ) { // MANUAL mode
        setParamValue(AUTO_EXPOSURE, ((CameraProp*)userdata)->exposureValue);
    }
}
// EXPOSURE -end------------------------------

// SHARPNESS -start---------------------------
void on_slider_sharpnessOnOff(int, void*) {
    //setParamAutoOnOff(SHARPNESS, sharpnessOnOff);
}

void on_slider_sharpnessValue(int, void*) {
    if (sharpnessOnOff == 0) { // MANUAL mode
        setParamValue(SHARPNESS, sharpnessValue);
    }
}
// SHARPNESS -end-----------------------------

// SHUTTER -start-----------------------------
void on_slider_shutterOnOff(int, void*) {
    //setParamAutoOnOff(SHUTTER, shutterOnOff);
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
// OCR -start---------------------------------
/*
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
    setTrackbarPos(tess_title, win_opencv, ocrOnOff);

    cout << out << endl;
    cout << "-----------------------------" << endl;
}
*/
// OCR -end-----------------------------------
