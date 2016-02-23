#include <FlyCapture2.h>
#include "MultipleCamera.hpp"

using namespace FlyCapture2;

MultipleCamera::MultipleCamera() {
    startMultipleCapture();
}

MultipleCamera::~MultipleCamera() {
    stopMultipleCapture();
}

void MultipleCamera::PrintError( FlyCapture2::Error error ) {
    error.PrintErrorTrace();
}

int MultipleCamera::startMultipleCapture() {

    FlyCapture2::Error error;
    
    BusManager busMgr;
    error = busMgr.GetNumOfCameras(&numCameras);
    if ( error != PGRERROR_OK ) {
        PrintError( error);
        return -1;
    }

    cout << numCameras << " cameras found!" << endl;

    camArray = new Camera*[numCameras];

    std::thread *t;
    for (unsigned int x = 0; x < numCameras; x++) {
        camArray[x] = new Camera();

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex( x, &guid );
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            return -1;
        }

        error = camArray[x]->Connect( &guid );
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            return -1;
        }

        CameraInfo camInfo;
        error = camArray[x]->GetCameraInfo( &camInfo );
        if ( error != PGRERROR_OK ) {
            PrintError( error );
            return -1;
        }

        cameraMap[camInfo.serialNumber] = camArray[x];

        t = new thread(&MultipleCamera::captureFrame, this, camInfo.serialNumber);
        cameraThread.push_back(t);
    }
    return 0;
}

void MultipleCamera::stopMultipleCapture() {
    map<unsigned int, Camera*>::iterator it;
    for (it = cameraMap.begin(); it != cameraMap.end(); it++) {
        it->second->Disconnect();
    }
}

void MultipleCamera::captureFrame(unsigned int serialNumber) {
    FlyCapture2::Error error;

    error = cameraMap[serialNumber]->StartCapture();
    if ( error != PGRERROR_OK ) {
        PrintError( error );
    }

    Image rawImage;
    error = cameraMap[serialNumber]->RetrieveBuffer( &rawImage);
    if ( error != PGRERROR_OK) {
        PrintError( error );
    }

    Image rgbImage;
    rawImage.Convert( PIXEL_FORMAT_RGB, &rgbImage );

    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
    frameMap[serialNumber] = &image;
    //image.release();

    /*
    FlyCapture2::Error error;

    error = cam->StartCapture();
    if ( error != PGRERROR_OK ) {
        PrintError( error );
    }
    
    Image rawImage;
    error = cam->RetrieveBuffer( &rawImage );
    if ( error != PGRERROR_OK ) {
        PrintError( error );
    }
    
    Image rgbImage;
    rawImage.Convert( PIXEL_FORMAT_RGB, &rgbImage );

    // convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
    frameMap[serialNumber] = &image;
    image.release();
    */
}
