//
// Created by ilia on 10.6.24..
//

#ifndef FROG_RGBLEDS_H
#define FROG_RGBLEDS_H
#include "FROG.h"
using namespace daisy;
using namespace daisysp;
using namespace daisy::seed;

#define N_LEDS (8)
class MyLedRgb {
public:
    void init(I2CHandle* i2CHandle);


    void update();
    void setSelected (int i);
    void setEnvelopeAmplitude (int i, float amp);
    void setVU (float L, float R);
    void enableVU (bool en, bool red);
    void setLed (int i, float val);

private:
    I2CHandle* i2c;
    int selected = 0;
    float amplitude[N_LEDS] = {};
    bool VUEnabled = false;
    bool VURed = false;
    float VUL = 0.0f;
    float VUR = 0.0f;
    uint8_t buf[8*3] = {};
};


#endif //FROG_RGBLEDS_H
