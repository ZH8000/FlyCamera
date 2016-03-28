#ifndef CIRCLE_MULTI_H
#define CIRCLE_MULTI_H

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

const char* win_title_head =   "頭部影像";
const char* win_setting_head = "頭部設定";
const char* win_title_tail =   "尾部影像";
const char* win_setting_tail = "尾部設定";

const char* bina_title =  "影像二元化 Off/On";
const char* bina_thresh = "影像二元化閥值";               // binarization thresh will between 0 ~ 150
const char* cont_title1 = "contour 面積下限";
const char* cont_title2 = "contour 面積上限";

const char* line_left_head =   "左邊界";
const char* line_right_head =  "右邊界";
const char* line_top_head =    "上邊界";
const char* line_bottom_head = "下邊界";
const char* line_left_tail =   "左邊界";
const char* line_right_tail =  "右邊界";
const char* line_top_tail =    "上邊界";
const char* line_bottom_tail = "下邊界";

int binaryOnOff_head = 0;           // binarization
int binaryThresh_head = 71;
int oldBinaryThresh_head = 71;
int binaryOnOff_tail = 0;
int binaryThresh_tail = 43;
int oldBinaryThresh_tail = 43;

bool RunHeadCheck();
#endif
