#ifndef FOR_DEMO_H
#define FOR_DEMO_H

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
int ocrOnOff = 0;              // OCR

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
int rightValue = 320;
int topValue = 0;
int bottomValue = 240;

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

#endif
