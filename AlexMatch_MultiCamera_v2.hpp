#ifndef ALEX_MATCH_MULTI_H
#define ALEX_MATCH_MULTI_H

#include <FlyCapture2.h>

#include <ctime>
#include <vector>
#include <iostream>
#include <list>
#include <map>
#include <sstream>
#include <string>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <tesseract/baseapi.h>
#include <tesseract/strngs.h>

#include "CommonFlySDK.hpp"
#include "CameraProp.hpp"
#include "OpenCVProp.hpp"

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
int binaryInvOnOff = 0;        // binarization inverse
int binaryMax =  100;          // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
int oldBinaryMax = 100;
int binaryThresh = 30;
int oldBinaryThresh = 30;
// int blurOnOff = 0;             // Gaussian Blur
// int blurValue = 0;
// int oldBlurValue = 0;
int successMatches = 100;      // AKAZE matches
// int ocrOnOff = 0;              // OCR

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
// const char* blur_title = "高斯模糊 Off/On";
// const char* blur_value = "高斯模糊 Kernel 大小";
const char* succ_matches = "辦別成功最低Match值";
// const char* tess_title = "單張文字辨識 trigger";
const char* sampled_title = " 樣本 ";

void createTrackbars(unsigned int id, CameraProp* prop, OpenCVProp* propCV);
void updateTrackbars(FlyCapture2::Camera* camera, CameraProp* prop);
void initOpenCVProp(unsigned int camId, OpenCVProp* propCV);

void on_slider_exposureOnOff(int, void*);  // exposure
void on_slider_exposureValue(int, void*);
void on_slider_sharpnessOnOff(int, void*); // sharpness
void on_slider_sharpnessValue(int, void*);
void on_slider_shutterOnOff(int, void*);   // shutter
void on_slider_shutterValue(int, void*);
void on_slider_binaryOnOff(int, void*);
void on_slider_binaryMax(int, void*);      // binarization
void on_slider_binaryThresh(int, void*);
void on_slider_ocrOnOff(int, void*);       // OCR
// void OCR(Mat*);

// void getCameraProp(Camera*);
// int RunSingleCamera( PGRGuid guid );

#endif
