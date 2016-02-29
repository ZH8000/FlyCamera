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

#include "webcam.h"

#define XRES 640
#define YRES 480

using namespace cv;
using namespace std;

bool ldown = false, lup = false;
Point corner1, corner2, moving;
Rect box;

const char* win_title = "影像";
const char* win_setting = "攝影機 設定";
const char* win_opencv = "OpenCV 設定";

const char* brig_value = "亮度值";
const char* cont_value = "對比度";
const char* satu_value = "飽和度";
const char* shar_value = "影像銳利化值";

const char* bina_title = "影像二元化 Off/ On";
const char* bina_max = "影像二元化最大接受閥值(+150)"; // binarization max value will between 0(+150) ~ 150(+150)
const char* bina_thresh = "影像二元化閥值";      // binarization thresh will between 0 ~ 150
const char* tess_title = "單張文字辨識 trigger";

int brightnessValue = 0;       // brightness
int contrastValue = 0;         // contrast
int saturationValue = 0;       // saturation
int sharpnessValue = 0;        // sharpness

int binaryOnOff = 0;           // binarization
int binaryMax =  150;          // binarization max value will between 0(+150) ~ 150(+150), p.s. actually value should plus 150, so 150~300
int oldBinaryMax = 100;
int binaryThresh = 30;
int oldBinaryThresh = 30;
int ocrOnOff = 0;              // OCR

void calibrateParams();
void getCapParams();
void on_slider_binaryOnOff(int, void*);    // binarization
void on_slider_binaryMax(int, void*);
void on_slider_binaryThresh(int, void*);
void on_slider_ocrOnOff(int, void*);       // OCR
void OCR();

static void mouse_callback(int event, int x, int y, int, void *);

// VideoCapture cap(0);
unsigned char bigbuffer[(2560*1920*3)];

int main() {

    cout << "Press 'q' to quit" << endl;
    cout << "Press 's' to take a picture" << endl;

    Webcam webcam("/dev/video0", XRES, YRES);

    namedWindow("test", WINDOW_NORMAL);
    while(1>0) { 
        auto frame = webcam.frame();

        Mat display(YRES, XRES, CV_8UC3, bigbuffer);
        int i, newi, newsize=0;

        for(i=0, newi=0; i<frame.size; i=i+4, newi=newi+2) {
            // Y1=first byte and Y2=third byte
            bigbuffer[newi]=frame.data[i];
            bigbuffer[newi+1]=frame.data[i+2];
            cout << i << " " << newi << endl;
        }
        imshow("test", display);
    }

    /*
    if (!cap.isOpened()) {
        cout << "Capture could not be opened successfully" << endl;
        return -1; 
    }*/

    // getCapParams();

    namedWindow(win_title, WINDOW_NORMAL);
    namedWindow(win_setting, WINDOW_NORMAL);
    namedWindow(win_opencv, WINDOW_NORMAL);

    // Setup trackbar
    createTrackbar(brig_value, win_setting, &brightnessValue, 100);
    createTrackbar(cont_value, win_setting, &contrastValue, 100);
    createTrackbar(satu_value, win_setting, &saturationValue, 100);
    createTrackbar(shar_value, win_setting, &sharpnessValue, 20);
    createTrackbar(bina_title, win_opencv, &binaryOnOff, 1, on_slider_binaryOnOff);
    createTrackbar(bina_max, win_opencv, &binaryMax, 150, on_slider_binaryMax);
    createTrackbar(bina_thresh, win_opencv, &binaryThresh, 150, on_slider_binaryThresh);
    createTrackbar(tess_title, win_opencv, &ocrOnOff, 1, on_slider_ocrOnOff);

    return 0;
}

// GET CAP PARAMS -start----------------------
/*
void getCapParams() {
    brightnessValue = cap.get(CV_CAP_PROP_BRIGHTNESS) * 100;
    cout << "brightness:" << brightnessValue << endl;
    contrastValue = cap.get(CV_CAP_PROP_CONTRAST) * 100;
    cout << "contrast: " << contrastValue << endl;
    saturationValue = cap.get(CV_CAP_PROP_SATURATION) * 100;
    cout << "saturation: " << saturationValue << endl;
    // I cant get sharpness ... TOSHIBA Z30A
    //sharpnessValue = cap.get(CV_CAP_PROP_SHARPNESS) * 100;
    //cout << "sharpness:" << sharpnessValue << endl;
}
*/
// GET CAP PARAMS -end------------------------

// CALIBRATE PARAMS -start--------------------
/*
void calibrateParams() {
    //cap.set(CV_CAP_PROP_BRIGHTNESS, float(brightnessValue/100));
    cout << "brightness 2: " << float(brightnessValue/100) << endl;
    //cap.set(CV_CAP_PROP_CONTRAST, float(contrastValue/100));
    cout << "contrast 2: " << float(contrastValue/100) << endl;
    //cap.set(CV_CAP_PROP_SATURATION, float(saturationValue/100));
    cout << "saturation 2: " << float(saturationValue/100) << endl;
    getCapParams();
}
*/
// CALIBRATE PARAMS -end----------------------

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
