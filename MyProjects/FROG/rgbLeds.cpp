//
// Created by ilia on 10.6.24..
//

#include "rgbLeds.h"
#include "FROG.h"

extern DaisySeed hw;


int GLeds [N_LEDS] = {0, 3, 6, 9, 12, 15, 18, 21};
int RLeds [N_LEDS] = {1, 4, 7, 10, 13, 16, 19, 22};
int BLeds [N_LEDS] = {2, 5, 8, 11, 14, 17, 20, 23};



void MyLedRgb::init(I2CHandle *i2CHandle) {

    for (int i = 0; i < N_LEDS * 3; ++i) {
        buf[i] = 0;
    }
    ws2812_send(buf, N_LEDS * 3);
}


void MyLedRgb::setSelected(int i) {
    selected = i;
}


void MyLedRgb::update() {

    for (int i = 0; i < N_LEDS * 3; ++i) {
        buf[i] = 0;
    }

    if (VUEnabled) {
        float VULed[8];
        VULed[0] = 4.0f * (VUL - 0.75f);
        VULed[1] = 4.0f * (VUL - 0.5f);
        VULed[2] = 4.0f * (VUL - 0.25f);
        VULed[3] = 4.0f * (VUL - 0.0f);
        VULed[7] = 4.0f * (VUR - 0.75f);
        VULed[6] = 4.0f * (VUR - 0.5f);
        VULed[5] = 4.0f * (VUR - 0.25f);
        VULed[4] = 4.0f * (VUR - 0.0f);

        for (int i = 0; i < 8; i++) {
            if (VULed[i] > 1.0f) VULed[i] = 1.0f;
        }

        if (VURed) {
            const float RPortion[4] = {1.0f, 0.99f, 0.99f, 0.9f};
            const float GPortion[4] = {.0f, 0.4f, 0.5f, 0.7f};
            const float BPortion[4] = {.0f, 0.0f, 0.0f, 0.9f};

            for (int i = 0; i < 4; i++) {
                setLed(RLeds[i], VULed[i] * RPortion[i]);
                setLed(GLeds[i], VULed[i] * GPortion[i]);
                setLed(BLeds[i], VULed[i] * BPortion[i]);
            }
            for (int i = 7; i > 3; i--) {
                setLed(RLeds[i], VULed[i] * RPortion[7 - i]);
                setLed(GLeds[i], VULed[i] * GPortion[7 - i]);
                setLed(BLeds[i], VULed[i] * BPortion[7 - i]);
            }
        } else {
            for (int i = 0; i < 8; i++) {
                setLed(RLeds[i], VULed[i] * 1.2f);
                setLed(GLeds[i], VULed[i] * 0.7f );
                setLed(BLeds[i], VULed[i] * 0.9f);
            }
        }

    } else {
        for (int i = 0; i < N_LEDS; ++i) {
            if (i != selected) {
                setLed(GLeds[i], amplitude[i] * 0.5f);
            } else {
                float a  = 0.6f + 0.4f * amplitude[i];
                setLed(GLeds[i], a * 0.65f);
                setLed(RLeds[i], a * 1.0f);
                setLed(BLeds[i], a * 0.9f);
            }
        }
    }


    ws2812_send(buf, N_LEDS * 3);
}

void MyLedRgb::setEnvelopeAmplitude(int i, float amp) {
    amplitude[i] = amp;
}

void MyLedRgb::setVU(float L, float R) {
    VUR = R;
    VUL = L;
}

void MyLedRgb::enableVU(bool en, bool red) {
    VUEnabled = en;
    VURed = red;
}

void MyLedRgb::setLed(int i, float val) {
    buf[i] = val * 0xff;
}






