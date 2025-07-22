//
// Created by ilia on 26.4.24
//

#ifndef FROG_FROG_H
#define FROG_FROG_H
#include "daisy_seed.h"
#include "daisysp.h"
#include <cstring>
#include "per/gpio.h"

void handleMidiCC (uint8_t cc, uint8_t  val);
void initEnvelopes();
void initFilters();
void setSeq(int band, int steps, int pos);
void processSeq();
void ws2812_send(uint8_t* data, uint16_t led_count);
void initSeq();


#endif //FROG_FROG_H
