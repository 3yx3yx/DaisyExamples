#include "daisy_seed.h"
#include "per/gpio.h"
#include "per/spi.h"
#include <cstdint>
//#include "daisysp.h"

using namespace daisy;
//using namespace daisysp;

DaisySeed hw;
SpiHandle spi_handle;
SpiHandle::Config spi_conf;
GPIO latch595, latch165;
MidiUsbHandler midi;


// void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
// {
// 	for (size_t i = 0; i < size; i++)
// 	{
// 		out[0][i] = in[0][i];
// 		out[1][i] = in[1][i];
// 	}
// }

#define SHIFT_595_N (4)
#define SHIFT_165_N (2)
bool shift595buf [SHIFT_595_N][8] = {};
bool shift165buf [SHIFT_165_N][8] = {};

void shiftWrite (int i, bool state){
	i = SHIFT_595_N*8 - i - 1;
	int reg = i / 8;
	int out = i % 8;
	if (reg < SHIFT_595_N) {
		shift595buf[reg][out] = state;
	}
}

void shiftUpdate () {
	uint8_t buf[SHIFT_595_N] = {};

	for (int i = 0; i < SHIFT_595_N; ++i) {
		for (int j = 0; j < 8; ++j) {
			buf[i] |= (uint8_t)shift595buf[i][j] << (7-j);
		}
	}

	latch595.Write(false);
	spi_handle.BlockingTransmit(buf, 4);
	latch595.Write(true);

 }


void shiftRead () {
	uint8_t buf[SHIFT_165_N] = {};
	latch165.Write(true);
	spi_handle.BlockingReceive(buf, SHIFT_165_N, 1000);
	latch165.Write(false);
	for (int i = 0; i < SHIFT_165_N; ++i) {
		for (int j = 0; j < 8; ++j) {
			shift165buf[i][j] = buf[i] & ((uint8_t)0x1 << j);
		}
	}
}

bool getBtn (int i) {
	int reg = i / 8;
	int in = i % 8;
	if (reg < SHIFT_165_N) {
		return !shift165buf[reg][in];
	}
	return false;
}


int main(void)
{
	hw.Init();
	MidiUsbHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
    midi.Init(midi_cfg);

	// hw.SetAudioBlockSize(4); // number of samples handled per callback
	// hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	// hw.StartAudio(AudioCallback);

	// Set some configurations
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
	
	int led = 0;


	while(1) {
		
		  midi.Listen();

        /** When there are messages waiting in the queue... */
        while(midi.HasEvents())
        {
            /** Pull the oldest one from the list... */
            auto msg = midi.PopEvent();
			
            // switch(msg.type)
            // {
            //     case NoteOn:
            //     {
            //         /** and change the frequency of the oscillator */
            //         auto note_msg = msg.AsNoteOn();
            //         if(note_msg.velocity != 0)
            //             osc.SetFreq(mtof(note_msg.note));
            //     }
            //     break;
            //         // Since we only care about note-on messages in this example
            //         // we'll ignore all other message types
            //     default: break;
            // }
        }

		shiftRead();

		if (getBtn(0)){
			shiftWrite(16, 1);
			shiftWrite(17, 0);
			shiftUpdate();
			uint8_t msg[] = {0x90, 32, 127};
			midi.SendMessage(msg, 3);
		}

		if (getBtn(1)){
			shiftWrite(16, 0);
			shiftWrite(17, 1);
			shiftUpdate();
			uint8_t msg[] = {0x90, 32, 0};
			midi.SendMessage(msg, 3);
		}

		hw.DelayMs(25);

		

    }
}
