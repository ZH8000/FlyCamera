#include "AlexMatch_MultiCamera_v2.hpp"

extern "C" {
    #include <stdio.h>
    #include <stdlib.h>
    #include <mpsse.h>
    #include <unistd.h>
}

using namespace std;
using namespace cv;
using namespace FlyCapture2;

//int sampleImagesFlag = 0;
unsigned int match_result = 1;
unsigned int counting = 0;
const int sampleImagesSize = 1;
const Mode k_fmt7Mode = MODE_1;
const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;
map<unsigned int, Camera*> cameraMap;
map<unsigned int, CameraProp> propMap;
map<unsigned int, OpenCVProp> propCVMap;
map<unsigned int, vector<Mat> > sampledImagesMap;
map<unsigned int, Mat> targetImagesMap;
list<unsigned int> cameraList;

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

Camera** ppCameras;
unsigned int numCameras;

int main(int argc, char** argv) {
    cout << "按下 'ESC' 離開相機程式" << endl;
    cout << "按下 's' 擷取樣本" << endl;
    cout << "按下 'r' 清除樣本" << endl;

    CommonFlySDK sdk;

    FlyCapture2::Error error;
    BusManager busMgr;


    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK) {
        PrintError( error );
        return -1;
    }

    ppCameras = new Camera*[numCameras];

    // init file
    ofstream outfile0("../gpio_out_cpp");
    if (outfile0.is_open() ) {
        outfile0 << 1 << endl;
        outfile0.close();
    }

    // for AKAXE matching
    //FileStorage fs("../H1to3p.xml", FileStorage::READ);
    //fs.getFirstTopLevelNode() >> homography;

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
        // FIXME U-Shine disable
        sdk.PrintCameraInfo( &camInfo );
        cameraList.push_back(camInfo.serialNumber);
        cameraMap[camInfo.serialNumber] = ppCameras[i];
        
/*
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
*/

        // 3. Set all cameras by using Format7
        // 3.1 get and print Format7 info.
        Format7Info fmt7Info;
        bool supported;
        fmt7Info.mode = k_fmt7Mode;
        error = ppCameras[i] -> GetFormat7Info(&fmt7Info, &supported);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }
        // FIXME U-Shine disable
        sdk.PrintFormat7Capabilities(fmt7Info);
        // 3.2 validate Format7ImageSettings
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

		error = ppCameras[i] -> SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket );
	    if (error != PGRERROR_OK) {
	        PrintError( error );
     	    return -1;
    	}

        // 4. init OpenCV prop &show window(s)
        initOpenCVProp(camInfo.serialNumber, &propCVMap[camInfo.serialNumber]);
        createTrackbars(camInfo.serialNumber, &propMap[camInfo.serialNumber], &propCVMap[camInfo.serialNumber]);

        // 5. Start capturing images
        error = ppCameras[i]->StartCapture();
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }
    }

    char c;
    int oldValue = 0;
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

        if ( c == 'r') {
            /*
            for(unsigned int x = 0; x < numCameras; x++) {
                CameraInfo camInfo;
                ppCameras[x]->GetCameraInfo( &camInfo );
                sampledImagesMap[camInfo.serialNumber].clear();
            }
            */
            list<unsigned int>::iterator ii;
            for(ii = cameraList.begin(); ii != cameraList.end(); ++ii) {
                //cout << distance(cameraList.begin(), ii) << " "<< *ii<< endl;
				for (std::vector<Mat>::iterator iter = sampledImagesMap[*ii].begin(); iter != sampledImagesMap[*ii].end(); ++iter) {
					(*iter).release();
				}
				sampledImagesMap[*ii].clear();

                for (int x = 0; x < sampleImagesSize; x++) {
                    stringstream ss;
                    ss << *ii << sampled_title << x;
                    destroyWindow(ss.str());
                }
            }
            
            //sampleImagesFlag = 0;
            cout << "-------Reset Sampled Images-------" << endl;
            
        }
/* FIXME disable for demo
        for (list<unsigned int>::iterator camIdIt = cameraList.begin(); camIdIt != cameraList.end(); ++camIdIt) {
            // 1. get camera's prop and update
            sdk.getCameraProp(cameraMap[*camIdIt], *camIdIt, &propMap[*camIdIt]);
            updateTrackbars(cameraMap[*camIdIt], &propMap[*camIdIt]);
        }
 */

        ifstream infile("../gpio_in_python");
        string line;
        getline(infile, line);
//		cout << "#0 is here segmentation fault?????????????????????? " << line << endl;
        

        if (line == "0") {              // no signal now,
            oldValue = 0;               // because it "no signal now", change oldValue = 0
        } else if (line == "1") {       // has signal now,
            if (oldValue == 0) {        // no signal before,
                c = 's';                // do algorithms
                oldValue = 1;           // change oldValue = 1;
                cout << "SLEEP 2 SEC to wait it coming..." << endl;
                sleep(2);
                cout << "OVER OVER OVER OVER OVER OVER OVER" << endl;
            } else if (oldValue == 1) { // had signal before...
            }
        }

        // init match_result
        match_result = 1;
        ofstream resultFile("../gpio_result_cpp");
        if (resultFile.is_open() ) {
            resultFile << match_result << endl;
            resultFile.close();
        }
        for (list<unsigned int>::iterator camIdIt = cameraList.begin(); camIdIt != cameraList.end(); ++camIdIt) {
//			cout << "#1 is here segmentation fault??????????????????????" << endl;
        
            // 1. get camera's prop and update
/* FIXME disable for demo
            sdk.getCameraProp(cameraMap[*camIdIt], *camIdIt, &propMap[*camIdIt]);
            updateTrackbars(cameraMap[*camIdIt], &propMap[*camIdIt]);
*/

            // 2. get image
            Image rawImage;
            error = cameraMap[*camIdIt]->RetrieveBuffer( &rawImage);
            if ( error != PGRERROR_OK ) {
                PrintError( error );
            }

            Image rgbImage;
            rawImage.Convert( PIXEL_FORMAT_BGR, &rgbImage );

            // convert to OpenCV Mat
            unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
            Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

            image = image(
                        Rect(propCVMap[*camIdIt].leftValue, 
                        propCVMap[*camIdIt].topValue, 
                        propCVMap[*camIdIt].rightValue - propCVMap[*camIdIt].leftValue, 
                        propCVMap[*camIdIt].bottomValue - propCVMap[*camIdIt].topValue));
            
            // BINARY ON/OFF
            if (propCVMap[*camIdIt].binaryOnOff == 1) {
                threshold(image, image, propCVMap[*camIdIt].binaryThresh, (propCVMap[*camIdIt].binaryMax+150), CV_THRESH_BINARY);
            }

            if( c == 's') {
                ofstream outfile("../gpio_out_cpp");
                if (outfile.is_open() ) {
                    outfile << 0 << endl;
                    outfile.close();
                    cout << "write gpio_out_cpp = 0, let machine pause." << endl;
                }
                if(sampledImagesMap[*camIdIt].size() < sampleImagesSize) { // NOT compare
                    sampledImagesMap[*camIdIt].push_back(image);
                    //cout << "-----Get Sampled Image #" << (++sampleImagesFlag) << "-----" << endl;
                    cout << "-----" << *camIdIt << " 取得樣本影像 第#" << sampledImagesMap[*camIdIt].size() << "-----" << endl;
                    stringstream ss;
                    ss << *camIdIt << "_sample_img_" << sampledImagesMap[*camIdIt].size() << ".jpg";
                    imwrite(ss.str(), image);

					imshow("樣本＃", *sampledImagesMap[*camIdIt].begin());
                    
                } else { // START compare!
                    image.copyTo(targetImagesMap[*camIdIt]);
					imshow("比較", image);

					imshow("樣本＃", *sampledImagesMap[*camIdIt].begin());
                    // print start time
                    time_t t_s = time(0);
                    struct tm * now = localtime( &t_s );
                    cout << "********" << *camIdIt << " START " << now->tm_hour << ":" << now->tm_min << ":"<< now->tm_sec << "********" << endl;
                
                    clock_t start, end;
                    double duration;
                    start = clock();
                
                    // list<Mat>::iterator i;
                    //for(i = sampledImagesMap[*camIdIt].begin(); i != sampledImagesMap[*camIdIt].end(); ++i) {
					for(int x = 0; x < sampledImagesMap[*camIdIt].size(); x++) {
                        //int idx =  distance(sampledImagesMap[*camIdIt].begin(), i);
						int idx = x;
                        //cout << idx << endl;
                        if (*camIdIt == 16043260) {
                            cout << "ignore" << endl;
                            break;
                        }
						stringstream ss;
						ss << "HIHI " << idx;
						//imshow(ss.str(), *i);
                        Match(sampledImagesMap[*camIdIt].at(x), idx, *camIdIt);
                    }
					imshow("asdf", *sampledImagesMap[*camIdIt].begin());
                    
                    end = clock();
                    duration = (double)(end - start) / CLOCKS_PER_SEC;
                    cout << "Duration: "  << duration << endl;
                    
                    // print end time
                    time_t t_e = time(0);
                    struct tm * now_e = localtime( &t_e );
                    cout << "********" << *camIdIt << " END " << now->tm_hour << ":" << now->tm_min << ":"<< now->tm_sec << "********" << endl;

					sleep(3);
                }				
                ofstream outfile2("../gpio_out_cpp");
                if (outfile2.is_open() ) {
                    outfile2 << 1 << endl;
                    outfile2.close();
                    cout << "write gpio_out_cpp = 1, let machine resume." << endl;
                }
                ofstream resultFile("../gpio_result_cpp");
                if (resultFile.is_open() ) {
                    resultFile << match_result << endl;
                    resultFile.close();
                }
            }

            stringstream ss;
            ss << win_title << *camIdIt;
            imshow(ss.str(), image);
        }
    }
//    Close(io);
    return 0;
}
    
inline void Match(Mat& sampledImage, int idx, unsigned int camId) {
//    cout << "camId: " << camId << " " << idx << endl;

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
    akaze->detectAndCompute(sampledImage, noArray(), kpts1, desc1);
    akaze->detectAndCompute(targetImagesMap[camId], noArray(), kpts2, desc2);

	stringstream ss;
	ss << "sampledImage " << idx;
	imshow(ss.str(), sampledImage);
	imshow("targetImagesMap", targetImagesMap[camId]);
/*
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
    drawMatches(sampledImage, inliers1, targetImagesMap[camId], inliers2, good_matches, res);
    Point pt = Point(100, 100);
    if (matched1.size() >= successMatches) {
        stringstream ss;
        ss << "OOOOO " << matched1.size();
        putText(res, ss.str(), pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
        match_result *= 1;
    } else {
        stringstream ss;
        ss << "XXXXX " << matched1.size();
        putText(res, ss.str(), pt, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
        match_result *= 0;
    }
    //imwrite("res.png", res);
    //namedWindow(win_akaze, WINDOW_NORMAL);
    //imshow(win_akaze, res);
    stringstream ss;
    ss << camId << sampled_title << idx;
    namedWindow(ss.str(), WINDOW_NORMAL);
    imshow(ss.str(), res);

    stringstream ss2;
    ss2 << camId << "_sample_" << counting++ << ".jpg";
    imwrite(ss2.str(), res);

    double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
    cout << camId << " Alex Matching Results #" << idx << endl;
    cout << "*******************************" << endl;
    cout << "# Keypoints 1:    \t" << kpts1.size() << endl;
    cout << "# Keypoints 2:    \t" << kpts2.size() << endl;
    cout << "# Matches:        \t" << matched1.size() << endl;
    cout << "# Inliers:        \t" << inliers1.size() << endl;
    cout << "# Inliers Ratio:  \t" << inlier_ratio << endl;
    cout << endl;
*/
}

void createTrackbars(unsigned int id, CameraProp *prop, OpenCVProp *propCV) {
    stringstream ss;

    // 1. for Mat image
    ss.str(std::string());
    ss << win_title << id;
    namedWindow(ss.str(), WINDOW_NORMAL);
    createTrackbar(left_title, ss.str(), &(propCV->leftValue), 640);
    createTrackbar(right_title, ss.str(), &(propCV->rightValue), 640);
    createTrackbar(top_title, ss.str(), &(propCV->topValue), 512);
    createTrackbar(bottom_title, ss.str(), &(propCV->bottomValue), 512);

    // 2. for Camera settings
/* FIXME disable for demo
    ss.str(std::string());
    ss << win_setting << id;
    namedWindow(ss.str(), WINDOW_NORMAL);
    // exposure
    createTrackbar(expo_title, ss.str(), &(prop->exposureOnOff), 1, on_slider_exposureOnOff, prop);
    createTrackbar(expo_value, ss.str(), &(prop->exposureValue), 1023, on_slider_exposureValue, prop);
    // sharpness
    createTrackbar(shar_title, ss.str(), &(prop->sharpnessOnOff), 1, on_slider_sharpnessOnOff, prop);
    createTrackbar(shar_value, ss.str(), &(prop->sharpnessValue), 4095, on_slider_sharpnessValue, prop);
    // shutter
    createTrackbar(shut_title, ss.str(), &(prop->shutterOnOff), 1, on_slider_shutterOnOff, prop);
    createTrackbar(shut_value, ss.str(), &(prop->shutterValue), 1590, on_slider_shutterValue, prop);
*/

    // 3. for OpenCV features
    ss.str(std::string());
    ss << win_opencv << id;
    namedWindow(ss.str(), WINDOW_NORMAL);
    createTrackbar(bina_title,   ss.str(), &(propCV->binaryOnOff), 1, on_slider_binaryOnOff, propCV);
    //createTrackbar(binv_title,   ss.str(), &(propCV->binaryInvOnOff), 1);
    //createTrackbar(bina_max,     ss.str(), &(propCV->binaryMax), 150, on_slider_binaryMax, propCV);
    createTrackbar(bina_thresh,  ss.str(), &(propCV->binaryThresh), 150, on_slider_binaryThresh, propCV);
    createTrackbar(succ_matches, ss.str(), &successMatches, 4000);
    // createTrackbar(tess_title, win_opencv, &ocrOnOff, 1);
}

inline void updateTrackbars(Camera* camera, CameraProp* prop) {
    const unsigned int sk_numProps = 18;

    // for Camera settings
/* FIXME - disable for demo
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
        } else
        if (camPropInfo.type == AUTO_EXPOSURE) {

//            cout << prop->camId << " AUTO_EXPOSURE " << camProp.exposureOnOff << " " << camProp.exposureValue << endl;
            setTrackbarPos(expo_title, ss.str(), prop->exposureOnOff);
            setTrackbarPos(expo_value, ss.str(), prop->exposureValue);
        } else
        if (camPropInfo.type == SHARPNESS) {
//            cout << prop->camId << " SHARPNESS " << camProp.valueA << endl;
            setTrackbarPos(shar_title, ss.str(), prop->sharpnessOnOff);
            setTrackbarPos(shar_value, ss.str(), prop->sharpnessValue);
        } else
        if (camPropInfo.type == GAMMA) {
//            cout << prop->camId << " GAMMA " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == SHUTTER) {
//            cout << prop->camId << " SHUTTER " << camProp.valueA << endl;
            setTrackbarPos(shut_title, ss.str(), prop->shutterOnOff);
            setTrackbarPos(shut_value, ss.str(), prop->shutterValue);
        } else
        if (camPropInfo.type == GAIN) {
//            cout << prop->camId << " GAIN " << camProp.valueA << endl;
        } else
        if (camPropInfo.type == FRAME_RATE) {
//            cout << prop->camId << " FRAME_RATE " << camProp.valueA << endl;
        } else 
        if (camPropInfo.type == TEMPERATURE) {
//            cout << prop->camId << " TEMPERATURE " << camProp.valueA << " K" << endl;
        }
    }
*/
    
    // for OpenCV settings
    stringstream ss_cv;
    ss_cv.str(std::string());
    ss_cv << win_opencv << prop->camId;
}

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

void setParamValue(PropertyType type, int value, unsigned int camId) {
    FlyCapture2::Error error;

    unsigned int cam = 0;

    list<unsigned int>::iterator ii;
    for(ii = cameraList.begin(); ii != cameraList.end(); ++ii) {
        if (*ii == camId) {
            cam = distance(cameraList.begin(), ii);
            break;
        }
    }

    Property prop;
    prop.type = type;
    error = ppCameras[cam]->GetProperty(&prop);
    if ( error != PGRERROR_OK) {
        PrintError( error );
        cout << "setparamValue error1" << endl;
    }

    prop.absControl = false;
    prop.onOff = true;
    prop.autoManualMode = 0;
    prop.valueA = value;
    error = ppCameras[cam]->SetProperty(&prop, false);
    if ( error != PGRERROR_OK ) {
        PrintError( error );
        cout << "setparamValue error2" << endl;
    }
}


void initOpenCVProp(unsigned int camId, OpenCVProp *propCV) {
    propCV->camId = camId;             // Camera's id
    propCV->binaryOnOff = 0;           // binarization
    propCV->binaryInvOnOff = 0;        // binarization inverse
    propCV->binaryMax =  100;          // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
    propCV->oldBinaryMax = 100;
    propCV->binaryThresh = 30;
    propCV->oldBinaryThresh = 30;
    // propCV->blurOnOff = 0;             // Gaussian Blur
    // propCV->blurValue = 0;
    // propCV->oldBlurValue = 0;
    propCV->successMatches = 100;      // AKAZE matches
    // propCV->ocrOnOff = 0;              // OCR

    propCV->leftValue = 0;
    propCV->rightValue = 640;
    propCV->topValue = 0;
    propCV->bottomValue = 512;
}
// EXPOSURE -start----------------------------
void on_slider_exposureOnOff(int, void* userdata) {
    setParamAutoOnOff(AUTO_EXPOSURE, ((CameraProp*)userdata)->exposureOnOff, ((CameraProp*)userdata)->camId);
}

void on_slider_exposureValue(int, void* userdata) {
    if ( ((CameraProp*)userdata)->exposureOnOff == 0 ) { // MANUAL mode
        setParamValue(AUTO_EXPOSURE, ((CameraProp*)userdata)->exposureValue, ((CameraProp*)userdata)->camId);
    }
}
// EXPOSURE -end------------------------------

// SHARPNESS -start---------------------------
void on_slider_sharpnessOnOff(int, void* userdata) {
    setParamAutoOnOff(SHARPNESS, ((CameraProp*)userdata)->sharpnessOnOff, ((CameraProp*)userdata)->camId);
}

void on_slider_sharpnessValue(int, void* userdata) {
    if ( ((CameraProp*)userdata)->sharpnessOnOff == 0 ) { // MANUAL mode
        setParamValue(SHARPNESS, ((CameraProp*)userdata)->sharpnessValue, ((CameraProp*)userdata)->camId);
    }
}
// SHARPNESS -end-----------------------------

// SHUTTER -start-----------------------------
void on_slider_shutterOnOff(int, void* userdata) {
    setParamAutoOnOff(SHUTTER, ((CameraProp*)userdata)->shutterOnOff, ((CameraProp*)userdata)->camId);
}

void on_slider_shutterValue(int, void* userdata) {
    if ( ((CameraProp*)userdata)->shutterOnOff == 0 ) { // MANUAL mode
        setParamValue(SHUTTER, ((CameraProp*)userdata)->shutterValue, ((CameraProp*)userdata)->camId);
    }   
}
// SHUTTER -end-------------------------------

// BINARIZATION -start------------------------
void on_slider_binaryOnOff(int, void* userdata) {
}

void on_slider_binaryMax(int, void* userdata) {
    stringstream ss;
    ss.str(std::string());
    ss << win_opencv << ((OpenCVProp*)userdata)->camId;

    if ( ((OpenCVProp*)userdata)->binaryOnOff == 0 ) {
        setTrackbarPos(bina_max, ss.str(), ((OpenCVProp*)userdata)->oldBinaryMax);
    } else {
        ((OpenCVProp*)userdata)->oldBinaryMax = ((OpenCVProp*)userdata)->binaryMax;
    }
}

void on_slider_binaryThresh(int, void* userdata) {
    stringstream ss;
    ss.str(std::string());
    ss << win_opencv << ((OpenCVProp*)userdata)->camId;

    if ( ((OpenCVProp*)userdata)->binaryOnOff == 0 ) {
        setTrackbarPos(bina_thresh, ss.str(), ((OpenCVProp*)userdata)->oldBinaryThresh);
    } else {
        ((OpenCVProp*)userdata)->oldBinaryThresh = ((OpenCVProp*)userdata)->binaryThresh;
    }
}
// BINARIZATION -end--------------------------
