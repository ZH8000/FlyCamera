#include <flycapture/FlyCapture2.h>
#include <iostream>

using namespace FlyCapture2;
using namespace std;

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

int main() {
  BusManager busMgr;
  unsigned int numCameras;
  Error error = busMgr.GetNumOfCameras(&numCameras);
  PGRGuid guid;

  if (numCameras >= 1) {

    busMgr.GetCameraFromIndex(0, &guid);

    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    Camera cam;
    cam.Connect(&guid);
    TriggerMode triggerMode;
    error = cam.GetTriggerMode( &triggerMode );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Set camera to trigger mode 0
    triggerMode.onOff = true;
    triggerMode.mode = 0;
    triggerMode.parameter = 0;
    triggerMode.source = 0;

    error = cam.SetTriggerMode( &triggerMode );

    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Camera is ready, start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }   

    cout << "Trigger the camera by sending a trigger pulse to GPIO" << triggerMode.source << endl; 

    Image image;
    for ( int imageCount=0; imageCount < 10; imageCount++ )
    {
        // Grab image        
        cout << "Start retreive:" << imageCount << endl;
        error = cam.RetrieveBuffer( &image );
        cout << "end retreive:" << imageCount << endl;
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            break;
        }
        cout<<"."<<endl; 
    }

    // Turn trigger mode off.
    triggerMode.onOff = false;    
    error = cam.SetTriggerMode( &triggerMode );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    cout << endl;
    cout << "Finished grabbing images" << endl; 

    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }      

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
  }
}

