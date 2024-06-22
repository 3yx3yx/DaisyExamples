#pragma once

#include "FROG.h"
using namespace daisy;
using namespace daisysp;
using namespace daisy::seed;
#define EEPROM_ADDR_DEFAULT 0xA0

enum eepromParamAddr {
    POPULATE=0,
    POSITION=POPULATE+8,
    ATTACK=POSITION+8,
    DECAY=ATTACK+8
};
class Eeprom {
public:
    void init(I2CHandle* i2CHandle, uint16_t i2cAddr = EEPROM_ADDR_DEFAULT);
    void writeByte (uint16_t addr, uint8_t byte);
    uint8_t readByte (uint16_t addr);
    void saveParam (uint16_t addr, uint8_t value);
    uint8_t readParam (uint16_t addr);
private:
    I2CHandle* i2c;
    uint16_t eepromAddr = EEPROM_ADDR_DEFAULT;
};