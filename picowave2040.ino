#include <PWMAudio.h>
//#include "ppg/ppg.h"
//#include "ppg/ppg_osc.h"
#include "ppg/ppg_data.h"
#include "ppg/ppg_data.c"
#include "filter.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#include <pico/rand.h>

int midiNoteFreq[] = {8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 19, 21, 22, 23, 24, 26, 28, 29, 31, 33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902, 8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544};

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

PWMAudio pwm_channel1(0, true); // GP0 = left, GP1 = right
PWMAudio pwm_channel2(2, true); // GP0 = left, GP1 = right
PWMAudio pwm_channel3(4, true); // GP0 = left, GP1 = right
PWMAudio pwm_channel4(6, true); // GP0 = left, GP1 = right

//const int freq = 48000; // Output frequency for PWM
const int freq = 32470; //12bit - 133mhz / 4096
const int base_freq = 0xffffffff / freq;
const uint8_t N = 7; //Number of bits in wavetable

static filter1pole filter;
static int8_t filter_cutoff = 63;

unsigned int lfo = 0;
int16_t modulation = 0;
uint8_t modulation_step = 13;

uint8_t wta_sel = 0;
uint8_t wtb_sel = 28;

const int notes[] = { 784/2, 440, 784/4, 220, 784/8, 110, 784/16, 55};
const int dly[] =   { 400, 600, 600, 400, 400, 400, 400, 400};

const int noteCnt = sizeof(notes) / sizeof(notes[0]);

#define ADSR_COUNT 3

#define ADSR_IDLE 0
#define ADSR_ATTACK 1
#define ADSR_DECAY 2
#define ADSR_SUSTAIN 3
#define ADSR_RELEASE 4

typedef struct {
  uint8_t attack = 50;
  uint8_t decay = 50;
  uint16_t sustain = 200;
  uint8_t release = 50;
  bool linear = true;
} adsr_eg;

typedef struct {
  adsr_eg adsr[ADSR_COUNT];
} patch;

patch current_patch;

typedef struct {
  uint8_t id;

  uint8_t waveA;
  uint8_t waveB;
  bool gate = false;

  uint8_t  adsr_state[ADSR_COUNT] = {0,0,0};
  uint16_t adsr_value[ADSR_COUNT] = {0,0,0};

  uint8_t amp;
  uint8_t note;
  int freq;
  uint32_t phase_step = 0;
  uint32_t phase_accum = 0x00000000;
  uint32_t sub_phase_step = 0;
  uint32_t sub_phase_accum = 0x00000000;
  int16_t output;
} voice_state;

const uint8_t VOICE_COUNT = 8;
voice_state voices[8];

uint8_t voice_alloc_head = 0;
uint8_t voice_alloc_tail = 1;
uint8_t voice_allocation[8] = { 0, 1, 2, 3, 4, 5, 6, 7};

uint8_t  address_pointer = 0x00;

uint8_t sineTable[128]; // Precompute sine wave in 128 steps
uint8_t wavetableA[128]; // Precompute sine wave in 128 steps
uint8_t wavetableB[128]; // Precompute sine wave in 128 steps

void update_adsr(voice_state *voice, uint32_t cnt) {
  patch *curr = &current_patch;
  for (int i = 0; i < ADSR_COUNT; i++) {
    adsr_eg *adsr = &curr->adsr[i];
    switch(voice->adsr_state[i]) {
      case ADSR_ATTACK:
        voice->adsr_value[i] += adsr->attack;
        if (voice->adsr_value[i] >= 2047) {
          voice->adsr_value[i] = 2047;
          voice->adsr_state[i] = ADSR_DECAY;
        }
        if (!voice->gate) {
          voice->adsr_state[i] = ADSR_RELEASE;
        }
        break;
      case ADSR_DECAY:
        if (voice->adsr_value[i] > adsr->sustain && voice->adsr_value[i] >= adsr->decay) {
          voice->adsr_value[i] -= adsr->decay;
        } else {
          voice->adsr_value[i] = adsr->sustain;
          voice->adsr_state[i] = ADSR_SUSTAIN;
        }
        if (!voice->gate) {
          voice->adsr_state[i] = ADSR_RELEASE;
        }
        break;
      case ADSR_SUSTAIN:
        if (!voice->gate) {
          voice->adsr_state[i] = ADSR_RELEASE;
        }
        break;
      case ADSR_RELEASE:
        if (voice->adsr_value[i] > adsr->release) {
          voice->adsr_value[i] -= adsr->release;
        } else {
          voice->adsr_value[i] = 0;
          voice->adsr_state[i] = ADSR_IDLE;
        }
        break;
      case ADSR_IDLE:
      default:
        if (voice->gate) {
          voice->adsr_state[i] = ADSR_ATTACK;
        }
        break;
    }
    if (voice->id == 0 && i == 0 && cnt%64 == 0) {
      Serial.printf("ADSR %d: %d - %d >> A%d D%d S%d R%d \n",cnt, voice->adsr_state[i],voice->adsr_value[i],adsr->attack,adsr->decay,adsr->sustain,adsr->release);
    }
  }
  
}

void inline update_voice(voice_state *voice) {
  uint8_t phase;
  uint8_t subPhase;

  voice->phase_accum += voice->phase_step;
  //voice->sub_phase_accum += voice->sub_phase_step;
  // 32 accumulator bits - 7 wavetable bits
  phase = voice->phase_accum >> (32-N);
  //subPhase = voice->sub_phase_accum >> (32-1); //1Bit SUB
  voice->output = ((wavetableA[phase]) * lfo);
  voice->output += ((wavetableB[phase]) * (127-lfo));
  //voice->output += subPhase * 127;
  //output = filter1pole_feed(&filter, (eg>>4), output);
  voice->output = (voice->output>>8) * (voice->adsr_value[0]>>3);
}

void inline update_channel(PWMAudio &pwm_channel, voice_state *voiceA, voice_state *voiceB) {
  uint8_t phaseA, phaseB;
  uint8_t subPhaseA, subPhaseB;
  while (pwm_channel.availableForWrite()) {
    //Sinewave LFO
    lfo = sineTable[((modulation>>8)&127)];

    update_voice(voiceA);
    update_voice(voiceB);

    //voiceA->output -= rosc_hw->randombit;
    //voiceB->output -= rosc_hw->randombit;

    pwm_channel.write(voiceA->output);
    pwm_channel.write(voiceB->output);
  }
}

void channel1_cb() {
  modulation += modulation_step;
  update_channel(pwm_channel1, &voices[0], &voices[1]);
}

void channel2_cb() {
  update_channel(pwm_channel2, &voices[2], &voices[3]);
}

void channel3_cb() {
  update_channel(pwm_channel3, &voices[4], &voices[5]);
}

void channel4_cb() {
  update_channel(pwm_channel4, &voices[6], &voices[7]);
}

void load_wavetable(uint8_t table[], uint8_t index, bool print = false) {
  int offset = ppg_wavetable_offsets[index];
  for (int i = 0; i < 64; i++) {
    table[i] = ppg_waveforms_data[offset+i];
  }
  for (int i = 0; i < 64; i++) {
    table[i+64] = 0xFF - ppg_waveforms_data[offset+i];
  }
  if (print){
    for (int i = 0; i < 128; i++) {
      //Serial.println(table[i]);
    }
  } else {
    //Serial.printf("loaded wavetable %d\n",index);
  }
}

uint8_t assign_voice = 0;
void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Log when a note is pressed.
  //Serial.print("Note on: channel = ");
  //Serial.print(channel);

  //Serial.print(" pitch = ");
  //Serial.print(pitch);

  //Serial.print(" velocity = ");
  //Serial.println(velocity);
  
  bool assigned = false;
  for (int i = 0; i < 8 && !assigned; i++) {
    voice_state *voice = &voices[voice_alloc_head];
    if (!voice->gate) {
      assign_voice = voice_alloc_head;
      assigned = true;
    }
    voice_alloc_head++;
    voice_alloc_head %= VOICE_COUNT;
  }

  voice_state *voice = &voices[assign_voice];
  
  //Serial.print(" voice alloc = ");
  //Serial.println(assign_voice);

  voice->note = pitch;
  voice->freq = midiNoteFreq[pitch];
  voice->gate = true;
  voice->amp = (uint8_t)velocity<<4;
  voice->phase_step = voice->freq * base_freq;
  voice->sub_phase_step = voice->phase_step/2;
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Log when a note is released.
  //Serial.print("Note off: channel = ");
  //Serial.print(channel);

  //Serial.print(" pitch = ");
  //Serial.print(pitch);

  //Serial.print(" velocity = ");
  //Serial.println(velocity);
  for (int i = 0; i < 8; i++) {
    voice_state *voice = &voices[i];
    if (voice->note == pitch) {
      voice->gate = false;
    }
  }
}

void handleCC(byte channel, byte control, byte value)
{
  patch *patch = &current_patch;
  if (channel != 1) { return; }
  if (control == 0) {
    load_wavetable(wavetableA, (int)value%28,true);
  }
  if (control == 2) {
    load_wavetable(wavetableB, (int)value%28,true);
  }
  if (control == 1) {
    modulation_step = (int) value;
  }
  if (control == 42) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->attack = 128 - value;
  }
  if (control == 44) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->decay = 128 - value;
  }
  if (control == 11) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->sustain = value<<4;
  }
  if (control == 5) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->release = 128 - value;
  }
}

void setup() {
  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
    // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
    TinyUSB_Device_Init(0);
  #endif
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff(); //otherwise it echoes in to out

  //MIDI.setHandleClock(handleClock);

  // Attach the handleNoteOn function to the MIDI Library. It will
  // be called whenever the Bluefruit receives MIDI Note On messages.
  MIDI.setHandleNoteOn(handleNoteOn);
  // Do the same for MIDI Note Off messages.
  MIDI.setHandleNoteOff(handleNoteOff);

  MIDI.setHandleControlChange(handleCC);

  Serial.begin(115200);
  
  // Set up sine table for waveform generation
  for (int i = 0; i < 128; i++) {
    sineTable[i] = (int) 128 * sin(i * 2.0 * 3.14159 / 128.0);
  }
  load_wavetable(wavetableA, wta_sel,true);
  load_wavetable(wavetableB, wtb_sel);

  pwm_channel1.setBuffers(4, 32); // Give larger buffers since we're are 48khz sample rate
  pwm_channel1.onTransmit(channel1_cb);
  pwm_channel1.begin(freq);

  pwm_channel2.setBuffers(4, 32); // Give larger buffers since we're are 48khz sample rate
  pwm_channel2.onTransmit(channel2_cb);
  pwm_channel2.begin(freq);

  pwm_channel3.setBuffers(4, 32); // Give larger buffers since we're are 48khz sample rate
  pwm_channel3.onTransmit(channel3_cb);
  pwm_channel3.begin(freq);

  pwm_channel4.setBuffers(4, 32); // Give larger buffers since we're are 48khz sample rate
  pwm_channel4.onTransmit(channel4_cb);
  pwm_channel4.begin(freq);

  for (int i = 0; i < 8; i++) {
      voice_state *voice = &voices[i];
      voice->id = i;
  }

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
}

uint32_t last_loop_time = 0;
uint32_t cnt = 0;
void loop() {
  //delay(2000);
  //Serial.println("working...");
  MIDI.read();
  if (millis() - last_loop_time > 5) {
    last_loop_time = millis();
    for (int i = 0; i < 8; i++) {
      voice_state *voice = &voices[i];
      update_adsr(voice,cnt++);
      if (!voice->gate && voice->amp > 0) {
        voice->amp--;
      }
    }
    last_loop_time = millis();
  }
}

