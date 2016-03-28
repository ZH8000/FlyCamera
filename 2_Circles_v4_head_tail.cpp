#include "2_Circles_v4_head_tail.hpp"

using namespace std;
using namespace cv;
using namespace FlyCapture2;

void PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

const Mode k_fmt7Mode = MODE_0;
const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;
const unsigned int snHead = 16043253;
const unsigned int snTail = 13021042;

Camera** ppCameras;
Camera* cam_Head;
Camera* cam_Tail;

unsigned int numCameras;
int leftValue_head = 0;             // draw lines.
int rightValue_head = 640;
int topValue_head = 0;
int bottomValue_head = 480;
int leftValue_tail = 0;
int rightValue_tail = 640;
int topValue_tail = 0;
int bottomValue_tail = 480;

int main(int argc, char** argv) {
    cout << "Press 'ESC' to quit" << endl;
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
        // 4. Start capturing images
        error = ppCameras[i]->StartCapture();
        if (error != PGRERROR_OK) {
            PrintError( error );
            return -1;
        }
		// 5. setup cam to head or tail.
		if ( camInfo.serialNumber == snHead ) {
			cout << "Head" << endl;
			cam_Head = ppCameras[i];
		} else {
			cout << "Tail" << endl;
			cam_Tail = ppCameras[i];
		}
    }

	namedWindow(win_title_head, WINDOW_NORMAL);
    namedWindow(win_title_tail, WINDOW_NORMAL);
	namedWindow(win_setting_head, WINDOW_NORMAL);
	namedWindow(win_setting_tail, WINDOW_NORMAL);

	createTrackbar(line_left_head,   win_title_head, &leftValue_head,   640);
    createTrackbar(line_right_head,  win_title_head, &rightValue_head,  640);
    createTrackbar(line_top_head,    win_title_head, &topValue_head,    480);
    createTrackbar(line_bottom_head, win_title_head, &bottomValue_head, 480);
	createTrackbar(line_left_tail,   win_title_tail, &leftValue_tail,   640);
    createTrackbar(line_right_tail,  win_title_tail, &rightValue_tail,  640);
    createTrackbar(line_top_tail,    win_title_tail, &topValue_tail,    480);
    createTrackbar(line_bottom_tail, win_title_tail, &bottomValue_tail, 480);

    Image rawImage_Head;
    Image rawImage_Tail;
    Image rgbImage_Head;
    Image rgbImage_Tail;

	char c;
	while ( true ) {
		c = waitKey(20);

		// show camera image.
		error = cam_Head->RetrieveBuffer( &rawImage_Head );
        if (error != PGRERROR_OK) {
            PrintError( error );
            continue;
        }
		error = cam_Tail->RetrieveBuffer( &rawImage_Tail );
        if (error != PGRERROR_OK) {
            PrintError( error );
            continue;
        }

        // Convert to RGB
        rawImage_Head.Convert( PIXEL_FORMAT_BGR, &rgbImage_Head );
		rawImage_Tail.Convert( PIXEL_FORMAT_BGR, &rgbImage_Tail );

        // convert to OpenCV Mat
		unsigned int rowBytes_Head = (double)rgbImage_Head.GetReceivedDataSize()/(double)rgbImage_Head.GetRows();
		unsigned int rowBytes_Tail = (double)rgbImage_Tail.GetReceivedDataSize()/(double)rgbImage_Tail.GetRows();       
        Mat image_Head = Mat(rgbImage_Head.GetRows(), rgbImage_Head.GetCols(), CV_8UC3, rgbImage_Head.GetData(),rowBytes_Head);
		Mat image_Tail = Mat(rgbImage_Tail.GetRows(), rgbImage_Tail.GetCols(), CV_8UC3, rgbImage_Tail.GetData(),rowBytes_Tail);
        // resize to smaller size
        Size size = Size(640, 480);
        resize(image_Head, image_Head, size);
        resize(image_Tail, image_Tail, size);
		// scope image
		image_Head = image_Head(Rect(leftValue_head,topValue_head, rightValue_head-leftValue_head, bottomValue_head-topValue_head));
		image_Tail = image_Tail(Rect(leftValue_tail,topValue_tail, rightValue_tail-leftValue_tail, bottomValue_tail-topValue_tail));

		// clean memory and close all windows
        if ( c == 27 ) {
            for (unsigned int x = 0; x < numCameras; x++) {
                destroyAllWindows();
                ppCameras[x]->StopCapture();
                ppCameras[x]->Disconnect();
                delete ppCameras[x];
            }
            delete [] ppCameras;
			delete cam_Head;
			delete cam_Tail;
            return 0;
        }

		if ( c == 's' ) {
			// 1. run head first.
			bool resultHead = RunHeadCheck();
			// 2. if head is OK, run tail.
			cout << resultHead << endl;
		}

		imshow(win_title_head, image_Head);
		imshow(win_title_tail, image_Tail);
	}
}

bool RunHeadCheck() {
	
	return false;
}
