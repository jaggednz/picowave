#include <PWMAudio.h>
//#include "ppg/ppg.h"
//#include "ppg/ppg_osc.h"
#include "ppg/ppg_data.h"
#include "ppg/ppg_data.c"
#include "filter.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

int midiNoteFreq[] = {8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 19, 21, 22, 23, 24, 26, 28, 29, 31, 33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902, 8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544};

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

PWMAudio pwm_channel1(0, true); // GP0 = left, GP1 = right
PWMAudio pwm_channel2(2, true); // GP0 = left, GP1 = right

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

uint32_t phase_step = 0;
uint32_t phase_accum = 0x00000000;

typedef struct {
  uint8_t waveA;
  uint8_t waveB;
  bool gate = false;
  uint8_t amp;
  uint8_t note;
  int freq;
  uint32_t phase_step = 0;
  uint32_t phase_accum = 0x00000000;
  int16_t output;
} voice_state;

voice_state voices[8];

uint8_t voice_alloc_head = 0;
uint8_t voice_alloc_tail = 1;
uint8_t voice_allocation[8] = { 0, 1, 2, 3 };

uint8_t  address_pointer = 0x00;

uint8_t sineTable[128]; // Precompute sine wave in 128 steps
uint8_t wavetableA[128]; // Precompute sine wave in 128 steps
uint8_t wavetableB[128]; // Precompute sine wave in 128 steps

void channel1_cb() {
  modulation += modulation_step;
  while (pwm_channel1.availableForWrite()) {
    lfo = sineTable[((modulation>>8)&127)];
    voice_state *voiceA = &voices[0];
    voiceA->phase_accum += voiceA->phase_step;
    // 32 accumulator bits - 7 wavetable bits
    uint8_t phaseA = voiceA->phase_accum >> (32-N);
    voiceA->output = ((wavetableA[phaseA]) * lfo);
    voiceA->output += ((wavetableB[phaseA]) * (127-lfo));
    //output = filter1pole_feed(&filter, (eg>>4), output);
    voiceA->output = (voiceA->output>>8) * (voiceA->amp);

    voice_state *voiceB = &voices[1];
    voiceB->phase_accum += voiceB->phase_step;
    // 32 accumulator bits - 7 wavetable bits
    uint8_t phaseB = voiceB->phase_accum >> (32-N);
    voiceB->output = ((wavetableA[phaseB]) * lfo);
    voiceB->output += ((wavetableB[phaseB]) * (127-lfo));
    //output = filter1pole_feed(&filter, (eg>>4), output);
    voiceB->output = (voiceB->output>>8) * (voiceB->amp);

    pwm_channel1.write(voiceA->output);
    pwm_channel1.write(voiceB->output);
  }
}

void channel2_cb() {
  while (pwm_channel2.availableForWrite()) {
    voice_state *voiceA = &voices[2];
    voiceA->phase_accum += voiceA->phase_step;
    // 32 accumulator bits - 7 wavetable bits
    uint8_t phaseA = voiceA->phase_accum >> (32-N);
    voiceA->output = ((wavetableA[phaseA]) * lfo);
    voiceA->output += ((wavetableB[phaseA]) * (127-lfo));
    //output = filter1pole_feed(&filter, (eg>>4), output);
    voiceA->output = (voiceA->output>>8) * (voiceA->amp);

    voice_state *voiceB = &voices[3];
    voiceB->phase_accum += voiceB->phase_step;
    // 32 accumulator bits - 7 wavetable bits
    uint8_t phaseB = voiceB->phase_accum >> (32-N);
    voiceB->output = ((wavetableA[phaseB]) * lfo);
    voiceB->output += ((wavetableB[phaseB]) * (127-lfo));
    //output = filter1pole_feed(&filter, (eg>>4), output);
    voiceB->output = (voiceB->output>>8) * (voiceB->amp);

    pwm_channel2.write(voiceA->output);
    pwm_channel2.write(voiceB->output);
  }
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
      Serial.println(table[i]);
    }
  } else {
    //Serial.printf("loaded wavetable %d\n",index);
  }
}

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Log when a note is pressed.
  Serial.print("Note on: channel = ");
  Serial.print(channel);

  Serial.print(" pitch = ");
  Serial.print(pitch);

  Serial.print(" velocity = ");
  Serial.println(velocity);

 
  voice_state *voice = &voices[voice_allocation[voice_alloc_head]];
  voice_alloc_head++;
  voice_alloc_head %= 4;

  voice->note = pitch;
  voice->freq = midiNoteFreq[pitch];
  voice->gate = true;
  voice->amp = (uint8_t)velocity<<4;
  set_phase_step(voice,voice->freq);
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Log when a note is released.
  Serial.print("Note off: channel = ");
  Serial.print(channel);

  Serial.print(" pitch = ");
  Serial.print(pitch);

  Serial.print(" velocity = ");
  Serial.println(velocity);
}

void handleCC(byte channel, byte control, byte value)
{
  if (control == 0) {
    load_wavetable(wavetableA, (int)value%28,true);
  }
  if (control == 2) {
    load_wavetable(wavetableB, (int)value%28,true);
  }
  if (control == 1) {
    modulation_step = (int) value;
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

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
}

void set_phase_step(voice_state *voice, int freqout) {
  voice->phase_step = freqout * base_freq;
}

void loop() {
  //delay(2000);
  //Serial.println("working...");
  MIDI.read(); 

  delay(5);
  for (int i = 0; i < 8; i++) {
    voice_state *voice = &voices[i];
    if (voice->amp > 0) {
      voice->amp--;
    }
  }
}

