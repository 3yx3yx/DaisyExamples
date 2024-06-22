//
// Created by ilia on 10.6.24..
//

#include "rgbLeds.h"

extern DaisySeed hw;
LedDriverPca9685 <2,true> pca9685;
static LedDriverPca9685<2, true>::DmaBuffer DMA_BUFFER_MEM_SECTION
        led_dma_buffer_a,
        led_dma_buffer_b;


int RLeds [N_LEDS] = {3, 0, 6, 14, 11, 8, 18, 21};
int GLeds [N_LEDS] = {4, 1, 7, 13, 10, 16, 19, 22};
int BLeds [N_LEDS] = {5, 2, 15, 12, 9, 17, 20, 23};


void MyLedRgb::init(I2CHandle *i2CHandle) {
    i2c = i2CHandle;
    uint8_t   addr[2] = {0x40, 0x41};
    pca9685.Init(*i2c, addr, led_dma_buffer_a, led_dma_buffer_b);
    pca9685.SetAllToRaw(0);
    pca9685.SwapBuffersAndTransmit();
}


void MyLedRgb::setSelected(int i) {
    selected = i;
}


void MyLedRgb::update() {

    pca9685.SetAllToRaw(0);


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
                pca9685.SetLed(GLeds[i], amplitude[i] * 0.5f);
            } else {
                float a  = 0.6f + 0.4f * amplitude[i];
                setLed(GLeds[i], a * 0.65f);
                setLed(RLeds[i], a * 1.0f);
                setLed(BLeds[i], a * 0.9f);
            }
        }
    }

    for (int i = 0; i < N_LEDS; i++) {

    }

    pca9685.SwapBuffersAndTransmit();
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
    pca9685.SetLed(i, val);
    if (val == 0.0f) {
        pca9685.SetLedRaw(i, 0);
    }
}






