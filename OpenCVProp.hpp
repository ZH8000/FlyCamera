#ifndef OPENCVPROP_H
#define OPENCVPROP_H

struct OpenCVProp {
    // Camera SN
    unsigned int camId;

    // BINARY(INV) - OpenCV
    int binaryOnOff;
    int binaryInvOnOff;
    int binaryMax;
    int oldBinaryMax;
    int binaryThresh;
    int oldBinaryThresh;
    
    // AKAZE matches
    int successMatches;

	// screen size
	int leftValue;
	int rightValue;
	int topValue;
	int bottomValue;
};

#endif
