#include "pwmdac/PWMAudio.cpp"
#include "filter.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#include <LittleFS.h>

//#include <pico/rand.h>

#include "screen.h"

int midiNoteFreq[] = {8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 19, 21, 22, 23, 24, 26, 28, 29, 31, 33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902, 8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544};

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

PWMAudio pwm_channel1(0, true); // GP0 = left, GP1 = right
PWMAudio pwm_channel2(2, true); // GP2 = left, GP3 = right
PWMAudio pwm_channel3(4, true); // GP4 = left, GP5 = right
PWMAudio pwm_channel4(6, true); // GP8 = left, GP7 = right

//const int freq = 48000; // Output frequency for PWM
const int sample_freq = 32470; //12bit - 133mhz / 4096
const int base_freq = 0xffffffff / sample_freq;
const uint8_t N = 8; //Number of bits in wavetable
const uint8_t N_ADJ = 32-N; 

#define WAVETABLE_FILE_SAMPLES 64*256*2 //64 x 256 16bit samples

char loaded_wavetable_filename[32] = "FM_-_COM.WAV";
uint16_t loaded_wavetable_file[WAVETABLE_FILE_SAMPLES];

//static filter1pole filter;
static int8_t filter_cutoff = 127;

unsigned int lfo = 0;
unsigned int inverse_lfo = 0;
int16_t modulation = 0;
uint8_t modulation_step = 13;

uint8_t wta_sel = 0;
uint8_t wtb_sel = 1;

#define ADSR_COUNT 3

#define ADSR_IDLE 0
#define ADSR_ATTACK 1
#define ADSR_DECAY 2
#define ADSR_SUSTAIN 3
#define ADSR_RELEASE 4

const uint16_t ADSR_MAX = 2047;

typedef struct {
  uint8_t attack = 50;
  uint8_t decay = 25;
  uint16_t sustain = ADSR_MAX;
  uint8_t release = 25;
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
  uint16_t wave_mix;
  uint16_t wave_mix_inv;
  bool gate = false;

  uint8_t  adsr_state[ADSR_COUNT] = {0,0,0};
  uint16_t adsr_value[ADSR_COUNT] = {0,0,0};

  uint8_t amp;
  uint8_t note;
  int freq;
  uint32_t phase_step = 0;
  uint32_t phase_accum = 0x00000000;

  uint32_t filter;

  uint32_t sub_phase_step = 0;
  uint32_t sub_phase_accum = 0x00000000;
  uint32_t output;
} voice_state;

const uint8_t VOICE_COUNT = 8;
voice_state voices[8];

uint8_t voice_alloc_head = 0;
uint8_t voice_alloc_tail = 1;

uint8_t  address_pointer = 0x00;

uint8_t sineTable[128]; // Precompute sine wave in 128 steps
uint16_t wavetable[256*2]; // Precompute sine wave in 128 steps
uint8_t screen_wavetable[512];

void update_adsr(voice_state *voice) {
  patch *curr = &current_patch;
  for (int i = 0; i < ADSR_COUNT; i++) {
    adsr_eg *adsr = &curr->adsr[i];
    switch(voice->adsr_state[i]) {
      case ADSR_ATTACK:
        voice->adsr_value[i] += adsr->attack;
        if (voice->adsr_value[i] >= ADSR_MAX) {
          voice->adsr_value[i] = ADSR_MAX;
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
        //go into idle and check for new gate!
      case ADSR_IDLE:
      default:
        if (voice->gate) {
          voice->adsr_state[i] = ADSR_ATTACK;
        }
        break;
    }
  }
  
}

void inline update_voice(voice_state *voice) {
  uint8_t phase;

  voice->phase_accum += voice->phase_step;
  // 32 accumulator bits - 7 wavetable bits
  phase = voice->phase_accum >> N_ADJ;

  //voice->sub_phase_accum += voice->sub_phase_step;
  //subPhase = voice->sub_phase_accum >> (32-1); //1Bit SUB
  voice->output =  (//wavetable[phase]);
     (wavetable[phase] * (2047-voice->adsr_value[1]))
     +
     (wavetable[phase+255] * (voice->adsr_value[1]-1))
    ) >> 11;
  //voice->output += subPhase * 127;
  voice->output = filter1pole_feed(&voice->filter, filter_cutoff, voice->output);
  voice->output = (voice->output * voice->adsr_value[0])>>11;
}

void inline update_channel(PWMAudio &pwm_channel, voice_state *voiceA, voice_state *voiceB) {
  while (pwm_channel.unsafeAvailableForWrite()) {
    //Sinewave LFO
    lfo = sineTable[((modulation>>8)&127)];
    inverse_lfo = 127-lfo;

    update_voice(voiceA);
    update_voice(voiceB);

    //voiceA->output -= rosc_hw->randombit;
    //voiceB->output -= rosc_hw->randombit;
    pwm_channel.writeStereo(voiceA->output, voiceB->output, false);
  }
}

void channel1_cb() {
  update_channel(pwm_channel1, &voices[0], &voices[1]);
  modulation += modulation_step;
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

void load_wavetable_file(){

  uint8_t buff[64];
  uint8_t msb;
  bool hold = false;
  uint16_t j = 0;
  uint32_t i = 0;
  int16_t tmp;

  //Open the Default wavetable
  //File f = LittleFS.open("/wavetables/PPG_BES.WAV","r");
  //if (!f) {
  //Serial.println("Default wavetable not found");
  //Open first file in the wavetables directory
  Dir d = LittleFS.openDir("/wavetables");
  d.rewind();

//FIXME only some wavetables open 
  d.next();
  d.next();
  d.next();
  //d.next();d.next();d.next();d.next();d.next();d.next();
  Serial.print("Opening ");
  Serial.println(d.fileName());
  Serial.flush();
  File f = d.openFile("r");
  //}
  if (!f) {
    screen_file_error();
    Serial.println("Error opening wavetable");
  } else {
    Serial.println("File opened");
  }
  //File f = LittleFS.open("/wavetables/FM_-_COM.WAV","r");
  //File f = LittleFS.open("/wavetables/CYBERNET.WAV","r");
  //Blindly seek to the end of the RIFF header
  f.seek(0x2B);
  while (f.available() > 0) {
    j = f.read(buff,64);
    for (int k = 0; k < j; k++) {
      if (hold) {
        tmp = (int16_t)(msb<<8| buff[k]);
        //Don't over fill the wavetable array, that would be bad!
        if (i < (64*256*2)) {
          loaded_wavetable_file[i] = (uint16_t) tmp+32768;
          i++;
        }
        hold = false;
      } else {
        msb = buff[k];
        hold=true;
      }
    }
  }
  f.close();
  Serial.print("Read  ");
  Serial.print(i);
  Serial.println(" uint16 from file.");
}

void load_wavetable(uint16_t table[], uint8_t position, uint8_t index, bool draw = false) {
  uint load_offset = index*0xFF;
  uint load_position = position*0xFF;
  for (int i = 0; i < 256; i++) {
    table[load_position + i] = loaded_wavetable_file[load_offset+i];
  }
  
  //For display we only need every second sample, scaled to 0-64
  for (int i = 0; i < 256; i++) {
    screen_wavetable[i] = table[i*2]>>10;
  }
  if (draw) {
    screen_set_show_wavetable(position);
  }
}

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
  voice_state *voice;
  for (int i = 0; i < VOICE_COUNT && !assigned; i++) {
    voice = &voices[voice_alloc_head];
    if (!voice->gate) {
      assigned = true;
    }
    voice_alloc_head++;
    voice_alloc_head %= VOICE_COUNT;
  }
  
  Serial.print(" voice alloc = ");
  Serial.println(voice_alloc_head);

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
  for (int i = 0; i < VOICE_COUNT; i++) {
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
    load_wavetable(wavetable,0, (int)value%64,true);
  }
   if (control == 1) {
    modulation_step = (int) value;
  }
  if (control == 2) {
    load_wavetable(wavetable,1, (int)value%64,true);
  }
  if (control == 3) {
    filter_cutoff = value;
  }
  if (control == 42) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->attack = 128 - value;
  }
  if (control == 43) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->decay = 128 - value;
  }
  if (control == 44) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->sustain = value<<4;
  }
  if (control == 45) {
    adsr_eg *adsr = &patch->adsr[0];
    adsr->release = 128 - value;
  }

  if (control == 46) {
    adsr_eg *adsr = &patch->adsr[1];
    adsr->attack = 128 - value;
  }
  if (control == 47) {
    adsr_eg *adsr = &patch->adsr[1];
    adsr->decay = 128 - value;
  }
  if (control == 48) {
    adsr_eg *adsr = &patch->adsr[1];
    adsr->sustain = value<<4;
  }
  if (control == 49) {
    adsr_eg *adsr = &patch->adsr[1];
    adsr->release = 128 - value;
  }
}

void midi_init() {
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff(); //otherwise it echoes in to out

  //MIDI.setHandleClock(handleClock);

  // Attach the handleNoteOn function to the MIDI Library. It will
  // be called whenever the Bluefruit receives MIDI Note On messages.
  MIDI.setHandleNoteOn(handleNoteOn);
  // Do the same for MIDI Note Off messages.
  MIDI.setHandleNoteOff(handleNoteOff);

  MIDI.setHandleControlChange(handleCC);
}

bool boot_complete = false;
void setup() {
  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
    // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
    TinyUSB_Device_Init(0);
  #endif
  
  //Start MIDI
  midi_init();
  //Start Serial
  Serial.begin(115200);
  //Start FS
  LittleFSConfig cfg;
  cfg.setAutoFormat(false);
  LittleFS.setConfig(cfg);
  LittleFS.begin();

  // Serial.println("ls /wavetables");
  // Dir dir = LittleFS.openDir("/wavetables");
  // // or Dir dir = LittleFS.openDir("/data");
  // while (dir.next()) {
  //     Serial.print(dir.fileName());
  //     if(dir.fileSize()) {
  //         File f = dir.openFile("r");
  //         Serial.print(" - ");
  //         Serial.println(f.size());
  //     }
  // }
  Serial.println("Loading default wavetable");
  Serial.flush();
  load_wavetable_file();
  Serial.println("Loading complete");
  // Set up sine table for waveform generation
  for (int i = 0; i < 128; i++) {
    sineTable[i] = (int) 64 * (sin(i * 2.0 * 3.14159 / 128.0)+1);
  }

  load_wavetable(wavetable,0, wta_sel);
  load_wavetable(wavetable,1, wtb_sel);

  pwm_channel1.setBuffers(3, 64); // Give larger buffers since we're are 48khz sample rate
  pwm_channel1.onTransmit(channel1_cb);
  pwm_channel1.begin(sample_freq);
  channel1_cb();

  pwm_channel2.setBuffers(3, 64); // Give larger buffers since we're are 48khz sample rate
  pwm_channel2.onTransmit(channel2_cb);
  pwm_channel2.begin(sample_freq);
  channel2_cb();

  pwm_channel3.setBuffers(3, 64); // Give larger buffers since we're are 48khz sample rate
  pwm_channel3.onTransmit(channel3_cb);
  pwm_channel3.begin(sample_freq);
  channel3_cb();

  pwm_channel4.setBuffers(3, 64); // Give larger buffers since we're are 48khz sample rate
  pwm_channel4.onTransmit(channel4_cb);
  pwm_channel4.begin(sample_freq);
  channel4_cb();

  for (int i = 0; i < VOICE_COUNT; i++) {
      voice_state *voice = &voices[i];
      voice->id = i;
  }

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
  boot_complete = true;
}

uint32_t last_loop_time = 0;
uint32_t cnt = 0;
void loop() {   
  //Serial.println("working...");
  MIDI.read();
  if (millis() - last_loop_time > 5) {
    last_loop_time = millis();
    for (int i = 0; i < VOICE_COUNT; i++) {
      voice_state *voice = &voices[i];
      update_adsr(voice);
      if (!voice->gate && voice->amp > 0) {
        voice->amp--;
      }
    }
    last_loop_time = millis();
  }
}

void setup1(){
  screen_init();
  screen_welcome(); 
  //Wait for the other core to finish booting.
  while( !boot_complete ) delay(1);
}

uint32_t last_screen_update = millis();
void loop1(){
  if (millis() - last_screen_update > 25) {
    screen_update(screen_wavetable, lfo);
    last_screen_update = millis();
  }
}

