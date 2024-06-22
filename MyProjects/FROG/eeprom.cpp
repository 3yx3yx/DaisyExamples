#include "eeprom.h"
extern DaisySeed hw;

void Eeprom::writeByte (uint16_t addr, uint8_t byte) {
    I2CHandle::Result res;
    res = i2c->WriteDataAtAddress(eepromAddr, addr, 2, &byte, 1, 1000);
    if (res == daisy::I2CHandle::Result::ERR) {
        __NOP();
    }
}

uint8_t Eeprom::readByte(uint16_t addr) {
    I2CHandle::Result res;
    uint8_t data = 0;
    res = i2c->ReadDataAtAddress(eepromAddr, addr, 2, &data, 1, 1000);
    if (res == daisy::I2CHandle::Result::ERR) {
        hw.DelayMs(10);
        i2c->ReadDataAtAddress(eepromAddr, addr, 2, &data, 1, 1000);
    }
    return data;
}

void Eeprom::init(I2CHandle *i2CHandle, uint16_t i2cAddr) {
    i2c = i2CHandle;
    eepromAddr = i2cAddr;
}

void Eeprom::saveParam(uint16_t addr, uint8_t value) {
    this->writeByte(addr, value);
}

uint8_t Eeprom::readParam(uint16_t addr) {
    return this->readByte(addr);
}



