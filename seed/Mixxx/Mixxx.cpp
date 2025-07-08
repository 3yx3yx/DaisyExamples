#include "daisy_seed.h"
#include "per/gpio.h"
#include "per/spi.h"
#include <cstdint>

using namespace daisy;

DaisySeed hw;
SpiHandle spi_handle;
SpiHandle::Config spi_conf;
GPIO latch595, latch165;
GPIO MuxPin[3];
GPIO clkInPin, clkOutPin;
MidiUsbHandler midi;
Encoder Enc;

#define SHIFT_595_N (11)
#define SHIFT_165_N (3)
bool shift595buf[SHIFT_595_N][8] = {};
bool shift165buf[SHIFT_165_N][8] = {};

void muxSet(uint8_t chan) {
    bool A = chan & 0b1;
    bool B = chan & 0b10;
    bool C = chan & 0b100;
    MuxPin[0].Write(A);
    MuxPin[1].Write(B);
    MuxPin[2].Write(C);
}

void shiftWrite(int i, bool state) {
    i = SHIFT_595_N * 8 - i - 1;
    int reg = i / 8;
    int out = i % 8;
    if (reg < SHIFT_595_N) {
        shift595buf[reg][out] = state;
    }
}

void shiftUpdate() {
    uint8_t buf[SHIFT_595_N] = {};
    for (int i = 0; i < SHIFT_595_N; ++i) {
        for (int j = 0; j < 8; ++j) {
            buf[i] |= (uint8_t) shift595buf[i][j] << (7 - j);
        }
    }
    latch595.Write(false);
    spi_handle.BlockingTransmit(buf, 4);
    latch595.Write(true);
}

void shiftRead() {
    uint8_t buf[SHIFT_165_N] = {};
    latch165.Write(true);
    spi_handle.BlockingReceive(buf, SHIFT_165_N, 1000);
    latch165.Write(false);
    for (int i = 0; i < SHIFT_165_N; ++i) {
        for (int j = 0; j < 8; ++j) {
            shift165buf[i][j] = buf[i] & ((uint8_t) 0x1 << j);
        }
    }
}

bool getBtn(int i) {
    int reg = i / 8;
    int in = i % 8;
    if (reg < SHIFT_165_N) {
        return !shift165buf[reg][in];
    }
    return false;
}

void sendNote(uint8_t note, uint8_t val) {
    uint8_t msg[3] = {};
    msg[0] = 0x90;
    msg[1] = note;
    msg[2] = val;
    midi.SendMessage(msg, 3);
}

void sendCC(uint8_t cc, uint8_t val) {
    uint8_t msg[3] = {};
    msg[0] = 0xB0;
    msg[1] = cc;
    msg[2] = val;
    midi.SendMessage(msg, 3);
}

void handleEncoder() {
    static bool encWasPressed = 0;
    Enc.Debounce();
    int inc = Enc.Increment();
    if (inc != 0) {
        if (inc > 0) {
            sendNote(0x19, 0x7F);
        } else {
            sendNote(0x1A, 0x7F);
        }
    }
    if (Enc.Pressed()) {
        if (!encWasPressed) {
            encWasPressed = true;
            sendNote(0x22, 0x7F);
        }
    } else {
        encWasPressed = false;
    }
}

void handleClkIn() {
    static bool prevState = false;
    if (clkInPin.Read()) {
        if (!prevState) {
            sendNote(0x30, 0x7F);    // send on rising edge
        }
        prevState = true;
    } else {
        prevState = false;
    }
}

#define ADC_MUX_N (2)
#define ADC_IN_N (ADC_MUX_N*8)
#define POT_READ_INTERVAL_MS (10)
static uint8_t potVal[ADC_IN_N] = {0};

void readPots() {
    static int chan = 0;
    muxSet(chan);

    for (int i = 0; i < ADC_MUX_N; i++) {
        int pot_i = (chan + i * 8) % ADC_IN_N;
        uint8_t newPotVal = hw.adc.GetFloat(i) * 127;
        if (newPotVal != potVal[pot_i]) {
            potVal[pot_i] = newPotVal;
            sendCC(pot_i, newPotVal);
        }

    }

    ++chan;
    if (chan >= 8) {
        chan = 0;
    }
}

void drawVU(int chan, int val) {
    int start = chan * (16 + 6);
    for (int i = 0; i < 16; i++) {
        if (val >  0) {
            shiftWrite(start + i, true);
        } else {
            shiftWrite(start + i, false);
        }
        val -= 127/16;
    }
}

void handleNote(uint8_t note, uint8_t val) {

    if (note > 0 && note <= 0x18) {
        uint8_t btn = (note - 1) % 6;
        uint8_t chan = (note - 1) / 6;
        uint8_t led = chan * (16 + 6) + btn + 16;

        if (val == 0x7F) {
            shiftWrite(led, true);
        } else {
            shiftWrite(led, false);
        }
    }

    if (note == 0x32) { // beat clock
        clkOutPin.Toggle();
    }

    if (note >= 0x45 && note <= 0x48) {
        int chan = note - 0x45;
        drawVU(chan, val);
    }
}

#define BUTTONS_N (SHIFT_165_N * 8)
#define BUTTONS_READ_INTERV_MS (100)

void handleButtons() {
    static bool btnStatePrev[BUTTONS_N] = {};
    shiftRead();
    for (int i = 0; i < BUTTONS_N; i++) {
        if (getBtn(i)) {
            if (!btnStatePrev[i]) {
                sendNote(i + 1, 0x7F);
            }
            btnStatePrev[i] = true;
        } else {
            btnStatePrev[i] = false;
        }
    }
}

void handleMidi() {
    midi.Listen();

    /** When there are messages waiting in the queue... */
    while (midi.HasEvents()) {
        /** Pull the oldest one from the list... */
        auto msg = midi.PopEvent();
        if (msg.type == NoteOn) {
            auto note_msg = msg.AsNoteOn();
            uint8_t note = note_msg.note;
            uint8_t val = note_msg.velocity;
            //hw.PrintLine("note: %d, val: %d", note, val);
            handleNote(note, val);
        }
    }
}

void shiftClear() {
    for (int i = 0; i < SHIFT_595_N * 8; i++) {
        shiftWrite(i, false);
    }
    shiftUpdate();
}

int main(void) {
    hw.Init();
    MidiUsbHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
    midi.Init(midi_cfg);

    spi_conf.periph = daisy::SpiHandle::Config::Peripheral::SPI_1;
    spi_conf.mode = daisy::SpiHandle::Config::Mode::MASTER;
    spi_conf.direction = daisy::SpiHandle::Config::Direction::TWO_LINES;
    spi_conf.nss = daisy::SpiHandle::Config::NSS::SOFT;
    spi_conf.pin_config.sclk = Pin(PORTG, 11);
    spi_conf.pin_config.miso = Pin(PORTB, 4);
    spi_conf.pin_config.mosi = Pin(PORTB, 5);
    //spi_conf.pin_config.nss = Pin(PORTG, 10);
    spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_64;
    // Initialize the handle using our configuration
    spi_handle.Init(spi_conf);

    latch165.Init(seed::D25, GPIO::Mode::OUTPUT);
    latch595.Init(seed::D24, GPIO::Mode::OUTPUT);
    MuxPin[0].Init(seed::D28, GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);
    MuxPin[1].Init(seed::D27, GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);
    MuxPin[2].Init(seed::D26, GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);
    clkInPin.Init(seed::D3, GPIO::Mode::INPUT, GPIO::Pull::PULLDOWN);
    clkOutPin.Init(seed::D4, GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);

    const int num_adc_channels = 4;
    AdcChannelConfig my_adc_config[num_adc_channels];
    my_adc_config[0].InitSingle(seed::A0);
    my_adc_config[1].InitSingle(seed::A1);
    my_adc_config[2].InitSingle(seed::A2);
    my_adc_config[3].InitSingle(seed::A3);
    hw.adc.Init(my_adc_config, num_adc_channels);
    hw.adc.Start();
    Enc.Init(seed::D21, seed::D20, seed::D22);

    //hw.StartLog(false);

    uint32_t now = 0;
    uint32_t btnRdTime = 0;
    uint32_t potRdTime = 0;

    shiftClear();

    while (1) {

        handleMidi();
        handleButtons();
        handleEncoder();
        handleClkIn();

        now = System::GetNow();
        if (now - btnRdTime > BUTTONS_READ_INTERV_MS) {
            btnRdTime = now;
            handleButtons();
        }
        if (now - potRdTime > POT_READ_INTERVAL_MS) {
            readPots();
        }

        shiftUpdate();


    }
}