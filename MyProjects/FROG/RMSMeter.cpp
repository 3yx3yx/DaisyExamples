//
// Created by ilia on 11.6.24..
//

#include "RMSMeter.h"



RMSMeter::RMSMeter() {
    for (int i = 0; i < RMS_BUF_SIZE; ++i) buf[i] = .0f;
}

float RMSMeter::process(float in) {
    static float prevOutSquared = 0.0f;
    static unsigned pos = 0;
    float outSquared = 0;
    float oldestSampleSquared = 0;
    float inSquared = in * in;
    const float lenInverted = 1.0f / RMS_BUF_SIZE;

    oldestSampleSquared = buf[pos];
    buf[pos] = inSquared;
    ++pos;
    if (pos >= RMS_BUF_SIZE) {pos = 0;}

    outSquared = prevOutSquared + lenInverted * (inSquared - oldestSampleSquared);
    prevOutSquared = outSquared;
    currRMS = sqrt(outSquared);
    return currRMS;
}

float RMSMeter::get() {
    return currRMS;
}
