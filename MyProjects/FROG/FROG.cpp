
#include "FROG.h"
#include "eeprom.h"
#include "rgbLeds.h"
#include "RMSMeter.h"

using namespace daisy;
using namespace daisysp;
using namespace daisy::seed;
DaisySeed hw;
I2CHandle i2c;
Eeprom eeprom;
MyLedRgb ledRgb;
static Metro tick;
static Overdrive driveL, driveR;
static Oscillator osc, lfo, lfo2, lfo3, lfoStereo2, lfoStereo;
static RMSMeter rmsMeter;
static float volumeMaster = 0;
static bool AudioInConnected = false;

#define SEQ_STEPS (16)
#define N_BANDS (8)
Svf filt[N_BANDS];
static float filterGains[N_BANDS] = {};
const float filterCenterFreq[N_BANDS]
        = {110.0f, 160.0f, 240.0f, 350.0f, 525.0f, 775.0f, 1200.0f, 1800.0f};
const float filterLfoMaxRange[N_BANDS]
        = {100.0f, 120.0f, 180.0f, 210.0f, 290.0f, 350.0f, 400.0f, 400.0f};
AdEnv env[N_BANDS];
static bool seq[N_BANDS][SEQ_STEPS] = {};
static int seqPos[N_BANDS] = {};
static int seqSteps[N_BANDS] = {};
static int seqTickPos = 0;
bool seqStopped = true;
uint32_t seqStartedTime = 0;
static int selectedBand = 0;
bool noteNeedOn[N_BANDS] = {};
bool noteIsOn[N_BANDS] = {};

MappedFloatValue volumeParam(0.001, 1.0f, 1.0f, MappedFloatValue::Mapping::log);
MappedFloatValue oscFreqParam(10.0f, 440.0f, 100.0f, MappedFloatValue::Mapping::log);
MappedFloatValue overdriveParam(0.1f, 1.0f, 0.0f);
MappedFloatValue lfoAmtParam(0.0, 1.0, 0.0f);
MappedFloatValue stereoModParam(0.01, 0.9, 0.1, MappedFloatValue::Mapping::lin);
MappedFloatValue tempo(1.0f, 20.0f, 10.0f, MappedFloatValue::Mapping::log);
MappedFloatValue attackParam(0.001f, 0.5f, 0.05f);
MappedFloatValue releaseParam(0.05f, 1.0f, 0.1f);
MappedFloatValue filterGain(0.001f, 1.0f, 1.0f, MappedFloatValue::Mapping::log);

MidiHandler<MidiUartTransport> midi;
FIFO<MidiEvent, 128> event_log;
typedef enum {
    CC_BAND_LEVEL_1 = 32,
    CC_BAND_LEVEL_2,
    CC_BAND_LEVEL_3,
    CC_BAND_LEVEL_4,
    CC_BAND_LEVEL_5,
    CC_BAND_LEVEL_6,
    CC_BAND_LEVEL_7,
    CC_BAND_LEVEL_8,

    CC_VOLUME = 7,
    CC_LFO_AMT = 1,
    CC_OSC_FREQ = 74,
    CC_OVERDRIVE = 71,
    CC_ATTACK = 73,
    CC_RELEASE = 72,

    CC_POPULATE = 16,
    CC_POSITION = 17,
} FrogMidiCC;
bool pitchWasUpdFromMidi = false;
bool lfoWasUpdFromMidi = false;
bool odWasUpdFromMidi = false;
bool volWasUpdFromMidi = false;
bool bandWasUpdFromMidi[N_BANDS] = {0};

void updateTick(float freq) {
    tick.SetFreq(freq);
    lfoStereo2.SetFreq(freq / 8);
    lfo.SetFreq(freq / 16);
    lfo2.SetFreq(freq / 7);
    lfo3.SetFreq(freq);
    lfoStereo.SetFreq(freq / 32);
}

void updateLfoAmp(float a) {
    lfo.SetAmp(a);
    lfo2.SetAmp(a);
    lfo3.SetAmp(a);
}

void handleMidiCC(uint8_t cc, uint8_t val) {

    float value0To1 = (float) val / 127;
    int steps = 0;
    int pos = 0;

    if (cc >= CC_BAND_LEVEL_1 && cc <= CC_BAND_LEVEL_8) {
        filterGain.SetFrom0to1(value0To1);
        selectedBand = cc - CC_BAND_LEVEL_1;
        filterGains[selectedBand] = filterGain.Get();
        if (filterGains[selectedBand] < 0.002) filterGains[selectedBand] = 0;
        bandWasUpdFromMidi[selectedBand] = true;
        return;
    }

    switch (cc) {
        case CC_VOLUME:
            volumeParam.SetFrom0to1(value0To1);
            volumeMaster = volumeParam.Get();
            if (volumeMaster < 0.0012f) volumeMaster = 0;
            volumeMaster = volumeMaster * (1.0f + 18.0f * (1.0f - 0.55f * overdriveParam.Get()));
            volWasUpdFromMidi = true;
            break;
        case CC_LFO_AMT:
            lfoAmtParam.SetFrom0to1(value0To1);
            updateLfoAmp(lfoAmtParam.Get());
            lfoWasUpdFromMidi = true;
            break;
        case CC_OSC_FREQ:
            oscFreqParam.SetFrom0to1(value0To1);
            stereoModParam.SetFrom0to1(value0To1);
            osc.SetFreq(oscFreqParam.Get());
            pitchWasUpdFromMidi = true;
            break;
        case CC_OVERDRIVE:
            overdriveParam.SetFrom0to1(value0To1);
            driveL.SetDrive(overdriveParam.Get());
            driveR.SetDrive(overdriveParam.Get());
            odWasUpdFromMidi = true;
            break;
        case CC_ATTACK:
            attackParam.SetFrom0to1(value0To1);
            env[selectedBand].SetTime(ADSR_SEG_ATTACK, attackParam.Get());
            eeprom.saveParam(eepromParamAddr::ATTACK + selectedBand, (uint8_t) (value0To1 * UINT8_MAX));
            break;
        case CC_RELEASE:
            releaseParam.SetFrom0to1(value0To1);
            env[selectedBand].SetTime(ADSR_SEG_DECAY, releaseParam.Get());
            eeprom.saveParam(eepromParamAddr::DECAY + selectedBand, (uint8_t) (value0To1 * UINT8_MAX));
            break;

        case CC_POPULATE:
            steps = val;
            if (steps > SEQ_STEPS) { steps = SEQ_STEPS; }
            setSeq(selectedBand, steps, seqPos[selectedBand]);
            eeprom.saveParam(eepromParamAddr::POPULATE + selectedBand, steps);
            break;

        case CC_POSITION:
            pos = val;
            if (pos > SEQ_STEPS - 1) { pos = SEQ_STEPS - 1; }
            setSeq(selectedBand, seqSteps[selectedBand], pos);
            eeprom.saveParam(eepromParamAddr::POSITION, pos);
            break;

        default:
            break;

    }
}

void initEnvelopes() {
    for (int i = 0; i < N_BANDS; ++i) {
        env[i].Init(hw.AudioSampleRate());
        env[i].SetMax(1.0f);
        env[i].SetMin(0.0f);
        env[i].SetTime(ADENV_SEG_ATTACK, 0.05f);
        env[i].SetTime(ADENV_SEG_DECAY, 0.1f);
        env[i].SetCurve(0.0f); // adjust for log curve
    }
}

void initFilters() {
    for (int i = 0; i < N_BANDS; ++i) {
        filt[i].Init(hw.AudioSampleRate());
        filt[i].SetDrive(0.5); // adjust this
        filt[i].SetFreq(filterCenterFreq[i]);
        filt[i].SetRes(0.92f); // adjust this
        filterGains[i] = 1.0f;
    }
}

void setSeq(int band, int steps, int pos) {

    band %= N_BANDS;
    steps %= SEQ_STEPS + 1;
    pos %= SEQ_STEPS;

    seqPos[band] = pos;
    seqSteps[band] = steps;

    for (int i = 0; i < SEQ_STEPS; ++i) {
        seq[band][i] = false;
    }

    if (steps == 0) {
        return;
    }

    int space = SEQ_STEPS / steps;

    for (int i = pos; i < SEQ_STEPS + pos; i += space) {
        // flip
        int step = i % SEQ_STEPS;
        seq[band][step] = true;
    }
}


void processSeq() // put this to audio callback
{
    if (seqStopped) {
        return;
    }

    if (tick.Process()) { // if tick event
        ++seqTickPos;
        seqTickPos %= SEQ_STEPS; // flip if exceed seq steps

        for (int band = 0; band < N_BANDS; ++band) {
            if (seq[band][seqTickPos]) { // if trigger is at the current step of the band sequence, trigger the envelope
                env[band].Trigger();
                noteNeedOn[band] = true;
            }
        }
    }
}


void processFilters(float in, float *outL, float *outR) {

    float lfoVal = 0.6f * lfo.Process() + 0.4f * lfo3.Process();
    float stereoMod = lfoStereo.Process();
    float stereoMod2 = lfoStereo2.Process();
    float L = 0;
    float R = 0;
    float stereoDepth = stereoModParam.Get();
    float stereoModL = 0.5f + 0.25f * stereoDepth * (stereoMod2 + stereoMod);
    float stereoModR = stereoModL * -1.0f;

    for (int i = 0; i < N_BANDS; ++i) {
        env[i].Process();

        filt[i].SetFreq(filterCenterFreq[i] + lfoVal * filterLfoMaxRange[i]);
        filt[i].Process(in * env[i].GetValue());

        if (i == 0) {
            L += filt[i].Low() * filterGains[i] * stereoModL;
            R += filt[i].Low() * filterGains[i] * stereoModR;
        } else {
            L += filt[i].Band() * filterGains[i] * stereoModL;
            R += filt[i].Band() * filterGains[i] * stereoModR;
        }
    }
    R /= (float) N_BANDS;
    L /= (float) N_BANDS;

    *outL = L;
    *outR = R;
}


void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size) {
    processSeq();

    for (size_t i = 0; i < size; i++) {
        float sig = osc.Process();
        if (AudioInConnected) {
            sig = 0.5f * (in[0][i] + in[1][i]);
        }
        float sigL = 0;
        float sigR = 0;

        processFilters(sig, &sigL, &sigR);

        sigL = driveL.Process(sigL);
        sigR = driveR.Process(sigR);

        out[0][i] = sigL * volumeMaster;
        out[1][i] = sigR * volumeMaster;
        rmsMeter.process(out[0][i]);
    }
}

void initSeq() {
    for (int i = 0; i < N_BANDS; ++i) {
        setSeq(i, 0, 0);
    }
}

void GetMidiTypeAsString(MidiEvent &msg, char *str) {
    switch (msg.type) {
        case NoteOff:
            strcpy(str, "NoteOff");
            break;
        case NoteOn:
            strcpy(str, "NoteOn");
            break;
        case PolyphonicKeyPressure:
            strcpy(str, "PolyKeyPres.");
            break;
        case ControlChange:
            strcpy(str, "CC");
            break;
        case ProgramChange:
            strcpy(str, "Prog. Change");
            break;
        case ChannelPressure:
            strcpy(str, "Chn. Pressure");
            break;
        case PitchBend:
            strcpy(str, "PitchBend");
            break;
        case SystemCommon:
            strcpy(str, "Sys. Common");
            break;
        case SystemRealTime:
            strcpy(str, "Sys. Realtime");
            break;
        case ChannelMode:
            strcpy(str, "Chn. Mode");
            break;
        default:
            strcpy(str, "Unknown");
            break;
    }
}

static uint32_t prev_timestamp = 0;
static uint16_t tt_count = 0;
static uint32_t last_MIDI_clk_upd_time = 0;

#define TTEMPO_MIN 20
#define TTEMPO_MAX 320

float bpm_to_freq(uint32_t tempo);

void HandleSystemRealTime(uint8_t srt_type);

float bpm_to_freq(uint32_t tempo) {
    return tempo / 60.0f;
}

static uint32_t fus_to_bpm(uint32_t us) {
    float fus = static_cast<float>(us);
    float val = roundf(60000000.0f / fus);
    return static_cast<uint32_t>(val);
}

//#define  DEBUGPRINT_SRT
//#define  DEBUGPRINT
//#define  DEBUGPRINT_BPM

void HandleSystemRealTime(uint8_t srt_type) {
    switch (srt_type) {
        // 0xFA - start
        case Start:
            seqStopped = false;
            seqStartedTime = System::GetNow();
            break;

            // 0xFC - stop
        case Stop:
            seqStopped = true;
            seqStartedTime = 0;
            break;

#ifdef DEBUGPRINT
            default:
            hw.PrintLine("MIDI SystemRealTime: %x", srt_type);
            break;
#endif

            // MIDI Clock -  24 clicks per quarter note
        case TimingClock:
            tt_count++;
            if (tt_count == 24) {
                uint32_t now = System::GetUs();
                uint32_t diff = now - prev_timestamp;
                uint32_t bpm = fus_to_bpm(diff);
#ifdef DEBUGPRINT_SRT
                hw.PrintLine("usec=%d, diff=%d, BPM=%d", now, diff, bpm);
                hw.PrintLine("BPM=%d", bpm);
#endif
                if (bpm >= TTEMPO_MIN && bpm <= TTEMPO_MAX && System::GetNow() - seqStartedTime > 1000) {
                    updateTick(bpm_to_freq(bpm) * 4.0f); // 16th notes
                }

                prev_timestamp = now;
                last_MIDI_clk_upd_time = System::GetNow();
                tt_count = 0;
            }
            break;
    }
}

GPIO LeftBtn, RightBtn, StartBtn;
GPIO MuxPin[3];
GPIO JackDetectPin;

Encoder EncPopulate, EncPosition, EncAttack, EncDecay;

void muxSet(uint8_t chan) {

    bool A = chan & 0b1;
    bool B = chan & 0b10;
    bool C = chan & 0b100;

    MuxPin[0].Write(A);
    MuxPin[1].Write(B);
    MuxPin[2].Write(C);

}

void initControls() {
    LeftBtn.Init(D18);
    RightBtn.Init(D17);
    StartBtn.Init(D1);
    JackDetectPin.Init(D28);

    MuxPin[0].Init(D26, GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);
    MuxPin[1].Init(D25, GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);
    MuxPin[2].Init(D24, GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);

    const int num_adc_channels = 2;
    AdcChannelConfig my_adc_config[num_adc_channels];
    my_adc_config[0].InitSingle(A0);
    my_adc_config[1].InitSingle(A1);
    hw.adc.Init(my_adc_config, num_adc_channels);
    hw.adc.Start();
    muxSet(0);

    EncPopulate.Init(D20, D21, D19);
    EncPosition.Init(D10, D8, D9);
    EncAttack.Init(D6, D5, D7);
    EncDecay.Init(D4, D2, D3);

    for (int i = 0; i < N_BANDS; ++i) {
        setSeq(i, eeprom.readParam(eepromParamAddr::POPULATE + i), eeprom.readParam(eepromParamAddr::POSITION + i));
        float attack = eeprom.readParam(eepromParamAddr::ATTACK + i);
        attackParam.SetFrom0to1(attack / UINT8_MAX);
        float decay = eeprom.readParam(eepromParamAddr::DECAY + i);
        releaseParam.SetFrom0to1(decay / UINT8_MAX);
        env[i].SetTime(ADSR_SEG_ATTACK, attackParam.Get());
        env[i].SetTime(ADSR_SEG_DECAY, releaseParam.Get());
    }

}

#define BTN_READ_DELAY_MS (30)

void readButtons() {

    static bool leftBtnHeld = false;
    static bool rightBtnHeld = false;
    static bool startBtnHeld = false;

    bool leftBtnPressed = !LeftBtn.Read();
    bool rightBtnPressed = !RightBtn.Read();
    bool startBtnPressed = !StartBtn.Read();

    if (leftBtnPressed) {
        if (!leftBtnHeld) {
            leftBtnHeld = true;
            --selectedBand;
            if (selectedBand < 0) {
                selectedBand = 0;
            }
            env[selectedBand].Trigger();
        }
    } else {
        leftBtnHeld = false;
    }

    if (rightBtnPressed) {
        if (!rightBtnHeld) {
            rightBtnHeld = true;
            ++selectedBand;
            if (selectedBand >= N_BANDS) {
                selectedBand = N_BANDS - 1;
            }
            env[selectedBand].Trigger();
        }
    } else {
        rightBtnHeld = false;
    }

    if (startBtnPressed) {
        if (!startBtnHeld) {
            startBtnHeld = true;
            seqStopped = !seqStopped;
            seqTickPos = 0;
            tick.Reset();
            if (seqStopped) {
                uint8_t msg[] = {0xFC};
                midi.SendMessage(msg, 1);
            } else {
                uint8_t msg[] = {0xFA};
                midi.SendMessage(msg, 1);
                seqStartedTime = System::GetNow();
            }
        }
    } else {
        startBtnHeld = false;
    }

    AudioInConnected = !JackDetectPin.Read();

}

#define ADC_READ_DELAY_MS (1)
#define ADC_AVG_SAMPLES (3)
enum Pots {
    POT_PITCH,
    POT_LFO,
    POT_VOL,
    POT_OD,
    POT_TEMPO
};

void readPots() {
    static int chan = 0;
    static float adc1Prev[8] = {};
    static float adc2Prev[8] = {};
    float adc1f = hw.adc.GetFloat(0);
    float adc2f = 1 - hw.adc.GetFloat(1); // these pots are reversed
    static float adc1buf[8] = {};
    static float adc2buf[8] = {};
    adc1buf[chan] = adc1f;
    adc2buf[chan] = adc2f;
    const float threshold = 0.001f;
    bool adc1ThresholdCrossed = false;
    bool adc2ThresholdCrossed = false;
    uint32_t now = System::GetNow();
    static uint32_t VUWasEnabledTime = 0;

    if (adc1f < threshold) adc1f = 0.0f;
    if (adc2f < threshold) adc2f = 0.0f;
    if (abs(adc1f - adc1Prev[chan]) > threshold) {
        if (chan < 5) {
            adc1ThresholdCrossed = true;
        }
    }
    if (abs(adc2f - adc2Prev[chan]) > threshold) {
        adc2ThresholdCrossed = true;
    }
    adc1Prev[chan] = adc1buf[chan];
    adc2Prev[chan] = adc2buf[chan];

    //filtering----------------------
    static float filtBuf1[8][ADC_AVG_SAMPLES] = {};
    static float filtBuf2[8][ADC_AVG_SAMPLES] = {};
    static int filtIdx = 0;
    filtBuf1[chan][filtIdx] = adc1f;
    filtBuf2[chan][filtIdx] = adc2f;

    float adcAvg1 = 0;
    float adcAvg2 = 0;
    for (int i = 0; i < ADC_AVG_SAMPLES; ++i) {
        adcAvg1 += filtBuf1[chan][i];
        adcAvg2 += filtBuf2[chan][i];
    }
    adc1f = adcAvg1 / ADC_AVG_SAMPLES;
    adc2f = adcAvg2 / ADC_AVG_SAMPLES;
    //--------------------------------



    if (adc2ThresholdCrossed || !bandWasUpdFromMidi[chan]) {
        filterGain.SetFrom0to1(adc2f);
        filterGains[chan] = filterGain.Get();
        if (filterGains[chan] < 0.002) filterGains[chan] = 0;
        bandWasUpdFromMidi[chan] = false;
    }


    switch (chan) {
        case POT_PITCH:
            if (!pitchWasUpdFromMidi || adc1ThresholdCrossed) {
                oscFreqParam.SetFrom0to1(adc1f);
                osc.SetFreq(oscFreqParam.Get());
                pitchWasUpdFromMidi = false;
            }
            break;
        case POT_LFO:
            if (!lfoWasUpdFromMidi || adc1ThresholdCrossed) {
                lfoWasUpdFromMidi = false;
                lfoAmtParam.SetFrom0to1(adc1f);
                stereoModParam.SetFrom0to1(adc1f);
                updateLfoAmp(lfoAmtParam.Get());
            }
            break;
        case POT_VOL:
            if (!volWasUpdFromMidi || adc1ThresholdCrossed) {
                volWasUpdFromMidi = false;
                volumeParam.SetFrom0to1(adc1f);
                if (adc1ThresholdCrossed) {
                    ledRgb.enableVU(true, false);
                    VUWasEnabledTime = now;
                }

                float od01 = overdriveParam.GetAs0to1();
                if (od01 > 0.6) od01 = 0.6;
                float v01 = volumeParam.GetAs0to1();
                v01 = v01 - od01;
                volumeParam.SetFrom0to1(v01);
                volumeMaster = volumeParam.Get() * 60.0f;

            }
            break;
        case POT_OD:
            if (!odWasUpdFromMidi || adc1ThresholdCrossed) {
                odWasUpdFromMidi = false;
                overdriveParam.SetFrom0to1(adc1f);
                driveL.SetDrive(overdriveParam.Get());
                driveR.SetDrive(overdriveParam.Get());
                if (adc1ThresholdCrossed) {
                    ledRgb.enableVU(true, true);
                    VUWasEnabledTime = now;
                }

            }
            break;
        case POT_TEMPO: {
            if ((now - last_MIDI_clk_upd_time > 3000 || now <= 3000) && now - seqStartedTime > 1000) {
                tempo.SetFrom0to1(adc1f);
                updateTick(tempo.Get());
            }
        }
            break;

        default:
            break;
    }


    if (now - VUWasEnabledTime > 1800 || now < 2500 || seqStopped) {
        ledRgb.enableVU(false, false);
    }

    ++chan;
    if (chan >= 8) {
        chan = 0;

        ++filtIdx;
        if (filtIdx >= ADC_AVG_SAMPLES) {
            filtIdx = 0;
        }
    }

    muxSet(chan);
}

#define ENCODER_FLOAT_INCREMENT (0.025f)

void readEncoders() {
    int inc;
    EncPopulate.Debounce();
    EncPosition.Debounce();
    EncAttack.Debounce();
    EncDecay.Debounce();

    inc = -1 * EncPopulate.Increment();
    if (inc != 0) {
        int steps = seqSteps[selectedBand];
        steps += inc;
        if (steps < 0) {
            steps = 0;
        } else if (steps > 5 && steps < 8) {
            if (inc > 0)
                steps = 8;
            else
                steps = 5;
        } else if (steps > 8) {
            if (inc > 0)
                steps = 16;
            else
                steps = 8;
        }
        setSeq(selectedBand, steps, seqPos[selectedBand]);
        eeprom.saveParam(eepromParamAddr::POPULATE + selectedBand, steps);
    }
    if (EncPopulate.Pressed()) {
        setSeq(selectedBand, 0, seqPos[selectedBand]);
    }

    inc = -1 * EncPosition.Increment();
    if (inc != 0) {
        int pos = seqPos[selectedBand];
        pos += inc;
        if (pos < 0) {
            pos = 0;
        } else if (pos > SEQ_STEPS) {
            pos = SEQ_STEPS;
        }
        setSeq(selectedBand, seqSteps[selectedBand], pos);
        eeprom.saveParam(eepromParamAddr::POSITION + selectedBand, pos);
    }
    if (EncPosition.Pressed()) {
        setSeq(selectedBand, seqSteps[selectedBand], 0);
    }

    inc = -1 * EncAttack.Increment();
    if (inc != 0) {
        float val = attackParam.GetAs0to1();
        val += (float) inc * ENCODER_FLOAT_INCREMENT;
        if (val > 1) { val = 1; }
        if (val < 0) { val = 0; }
        eeprom.saveParam(eepromParamAddr::ATTACK + selectedBand, (uint8_t) (val * UINT8_MAX));
        attackParam.SetFrom0to1(val);
        env[selectedBand].SetTime(ADSR_SEG_ATTACK, attackParam.Get());
    }
    if (EncAttack.Pressed()) {
        attackParam.ResetToDefault();
        float val = attackParam.GetAs0to1();
        eeprom.saveParam(eepromParamAddr::ATTACK + selectedBand, (uint8_t) (val * UINT8_MAX));
        env[selectedBand].SetTime(ADSR_SEG_ATTACK, attackParam.Get());
    }

    inc = -1 * EncDecay.Increment();
    if (inc != 0) {
        float val = releaseParam.GetAs0to1();
        val += (float) inc * ENCODER_FLOAT_INCREMENT;
        if (val > 1) { val = 1; }
        if (val < 0) { val = 0; }
        eeprom.saveParam(eepromParamAddr::DECAY + selectedBand, (uint8_t) (val * UINT8_MAX));
        releaseParam.SetFrom0to1(val);
        env[selectedBand].SetTime(ADSR_SEG_DECAY, releaseParam.Get());
    }
    if (EncDecay.Pressed()) {
        releaseParam.ResetToDefault();
        float val = releaseParam.GetAs0to1();
        eeprom.saveParam(eepromParamAddr::DECAY + selectedBand, (uint8_t) (val * UINT8_MAX));
        env[selectedBand].SetTime(ADSR_SEG_DECAY, releaseParam.Get());
    }

}

void updateLeds() {
    ledRgb.setSelected(selectedBand);
    for (int i = 0; i < 8; i++) {
        ledRgb.setEnvelopeAmplitude(i, env[i].GetValue());
    }
    ledRgb.setVU(rmsMeter.get(), rmsMeter.get());
    ledRgb.update();
}

void initOsc() {
    float sample_rate = hw.AudioSampleRate();
    osc.Init(sample_rate);
    osc.SetFreq(50.0f);
    osc.SetWaveform(Oscillator::WAVE_SQUARE);
    osc.SetAmp(0.5f);
    osc.SetPw(0.5f);

    lfo.Init(sample_rate);
    lfo.SetAmp(0.9f);
    lfo.SetWaveform(Oscillator::WAVE_TRI);
    lfo.SetFreq(0.99f);

    lfo2.Init(sample_rate);
    lfo2.SetAmp(0.9f);
    lfo2.SetWaveform(Oscillator::WAVE_TRI);
    lfo2.SetFreq(0.99f);
    lfo2.PhaseAdd(0.4f);

    lfo3.Init(sample_rate);
    lfo3.SetAmp(0.9f);
    lfo3.SetWaveform(Oscillator::WAVE_TRI);
    lfo3.SetFreq(0.99f);
    lfo3.PhaseAdd(-0.2f);

    lfoStereo2.Init(hw.AudioSampleRate());
    lfoStereo2.SetFreq(1.0f);
    lfoStereo2.SetAmp(1.0f);
    lfoStereo2.SetWaveform(Oscillator::WAVE_TRI);
    lfoStereo2.PhaseAdd(0.3f);

    lfoStereo.Init(hw.AudioSampleRate());
    lfoStereo.SetFreq(1.0f);
    lfoStereo.SetAmp(1.0f);
    lfoStereo.SetWaveform(Oscillator::WAVE_TRI);
    lfoStereo.PhaseAdd(0.7f);
}

int main(void) {
    hw.Configure();
    hw.Init();
    I2CHandle::Config i2c_conf;
    i2c_conf.periph = I2CHandle::Config::Peripheral::I2C_1;
    i2c_conf.speed = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_conf.mode = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_conf.pin_config.scl = {DSY_GPIOB, 8};
    i2c_conf.pin_config.sda = {DSY_GPIOB, 9};
    i2c.Init(i2c_conf);
    eeprom.init(&i2c);
    ledRgb.init(&i2c);

    hw.SetAudioBlockSize(16);
    float sample_rate = hw.AudioSampleRate();
    float callbackrate = hw.AudioCallbackRate();
    tick.Init(8.0f, callbackrate);
    initOsc();
    initSeq();
    initEnvelopes();
    initFilters();

    hw.StartAudio(AudioCallback);
    hw.StartLog(false);

    MidiUartHandler::Config midiConfig;
    midi.Init(midiConfig);
    midi.StartReceive();

    uint32_t now = System::GetNow();
    uint32_t log_time = now;
    uint32_t btnReadTime = now;
    uint32_t adcReadTime = now;
    initControls();

//    System::ResetToBootloader()

    while (1) {
        midi.Listen();
        while (midi.HasEvents()) {
            MidiEvent msg = midi.PopEvent();
            switch (msg.type) {
                case NoteOn: {
                    uint8_t band = msg.data[0] % 12;
                    if (band >= N_BANDS) {
                        band = N_BANDS - 1;
                    }
                    env[band].Trigger();
                }
                    break;
                case ControlChange: {
                    handleMidiCC(msg.data[0], msg.data[1]);
                }
                    break;

                case SystemRealTime: {
                    HandleSystemRealTime(msg.srt_type);
                }
                default:
                    break;
            }
#ifdef PRINT_MIDI_MSG
            event_log.PushBack(msg);
#endif
        }

        static uint32_t midiClkUSecPrev = 0;
        static uint32_t midiClkUSecInterval = 0;
        // tick has 16th notes freq, we need 24 midi clk per 4th note
        midiClkUSecInterval = 1000000.0f * (1.0f / 6.0f *(tick.GetFreq()));
        if (!seqStopped) {
            uint32_t uSecNow = System::GetUs();
            if (uSecNow - midiClkUSecPrev >= midiClkUSecInterval) {
                uint8_t clkMsg[] = {0xF8};
                midi.SendMessage(clkMsg, 1);
                midiClkUSecPrev = System::GetUs();
            }
        }


        static int seqTickPosPrev = 0;
        if (seqTickPos != seqTickPosPrev) {
            seqTickPosPrev = seqTickPos;
            for (int i = 0; i < N_BANDS; ++i) {
                if (noteIsOn[i]) {
                    uint8_t noteOffMsg[3] = {};
                    noteOffMsg[0] = 0x90;
                    noteOffMsg[1] = i;
                    noteOffMsg[2] = 0;
                    midi.SendMessage(noteOffMsg, 3);
                    noteIsOn[i] = false;
                }
                if (noteNeedOn[i]) {
                    noteNeedOn[i] = false;
                    uint8_t noteOnMsg[3] = {};
                    noteOnMsg[0] = 0x90;
                    noteOnMsg[1] = i;
                    noteOnMsg[2] = 0x7f;
                    midi.SendMessage(noteOnMsg, 3);
                    noteIsOn[i] = true;
                }
            }
        }

        now = System::GetNow();
        if (now - btnReadTime > BTN_READ_DELAY_MS) {
            readButtons();
            btnReadTime = now;
        }
        if (now - adcReadTime > ADC_READ_DELAY_MS) {
            readPots();
            adcReadTime = now;
        }
        readEncoders();
        updateLeds();


#ifdef PRINT_MIDI_MSG
        /** Now separately, every 5ms we'll print the top message in our queue if there is one */
        if (now - log_time > 5) {
            log_time = now;
            if (!event_log.IsEmpty()) {
                auto msg = event_log.PopFront();
                char outstr[128];
                char type_str[16];
                GetMidiTypeAsString(msg, type_str);
                sprintf(outstr,
                        "time:\t%ld\ttype: %s\tChannel:  %d\tData MSB: "
                        "%d\tData LSB: %d\n",
                        now,
                        type_str,
                        msg.channel,
                        msg.data[0],
                        msg.data[1]);
                hw.PrintLine(outstr);
            }
        }
#endif
    }
}
