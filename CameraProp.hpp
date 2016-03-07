#ifndef CAMERAPROP_H
#define CAMERAPROP_H

struct CameraProp {
    // Camera SN
    unsigned int camSN;

    // BRIGHTNESS
    int brightnessOnOff;
    int brightnessValue;

    // EXPOSURE
    int exposureOnOff;
    int exposureValue;

    // SHARPNESS
    int sharpnessOnOff;
    int sharpnessValue;
    
    // GAMMA
    int gammaOnOff;
    int gammaValue;

    // SHUTTER
    int shutterOnOff;
    int shutterValue;

    // GAIN
    int gainOnOff;
    int gainValue;

    // FRAME RATE
    int frameOnOff;
    int frameValue;
    
    // BINARY(INV) - OpenCV
    int binaryOnOff;
    int binaryInvOnOff;
    int binaryMax;
    int binaryThresh;
};

#endif
