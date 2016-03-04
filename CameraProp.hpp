#ifndef CAMERAPROP_H
#define CAMERAPROP_H

struct CamProp {
    // BRIGHTNESS
    int brightnessOnOff;
    int brightnessValue;

    // EXPOSURE
    int exposureOnOff;
    int exposureValue;

    // SHARPNESS
    int sharpnessOnOff;
    int sharpnessValue;

    // SHUTTER
    int shutterOnOff;
    int shutterValue;

    // GAIN
    int gainOnOff;
    int gainValue;

    // FRAME RATE
    int frameOnOff;
    int frameValue;
};

#endif
