#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

#define SCREEN_MODE_IDLE 0
#define SCREEN_MODE_WELCOME 1
#define SCREEN_MODE_WAVETABLE 2
#define SCREEN_MODE_ADSR 3
#define SCREEN_MODE_LFO 4

#define SCREEN_MODE_FATAL 99

const uint16_t SCREEN_KEEP_TIME = 800;
const uint16_t SCREEN_LOW_POWER_TIME = 60000;
const uint16_t SCREEN_SLEEP_TIME = 0xFFFF;

uint32_t screen_mode_keep = millis();
uint8_t screen_mode = 0;

uint16_t screen_wave_offset=0;

void screen_init() {
  Wire.setSCL(21);
  Wire.setSDA(20);
  u8g2.begin();
  u8g2.setFont(u8g2_font_bitcasual_tr);	// choose a suitable font
  u8g2.setContrast(64);
}

void screen_welcome(){
  screen_mode = SCREEN_MODE_WELCOME;
  screen_mode_keep = millis();
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawStr(0,10,"PICOWAVE 2040");	// write something to the internal memory
  u8g2.sendBuffer();					// transfer internal memory to the display   
}

void screen_file_error() {
  screen_mode = SCREEN_MODE_FATAL;
  screen_mode_keep = millis();
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawStr(0,15,"UNABLE TO OPEN");	// write something to the internal memory
  u8g2.drawStr(12,30,"WAVETABLE");
  u8g2.sendBuffer();					// transfer internal memory to the display   
}

void screen_set_show_wavetable(uint8_t position) {
  screen_wave_offset = position*128;
  screen_mode = SCREEN_MODE_WAVETABLE;
  screen_mode_keep = millis();
}
void screen_draw_static_wavetable(uint8_t table[], uint16_t offset){
    u8g2.clearBuffer();					// clear the internal memory
    u8g2.drawStr(0,10,"WaveTable " );	// write something to the internal memory
    u8g2.drawGlyph(80,10,65+(offset/128));	// write something to the internal memory
    int j = 0;
    int y1 = table[0];
    int y2= 0;
    for (int i = 0; i < 128; i++) {
      y2 = table[offset+i];
      u8g2.drawLine(j, y1, i, y2);
      j=i;
      y1 = y2;
    }
    u8g2.sendBuffer(); // transfer internal memory to the display
}

void screen_draw_morphing_wave(uint8_t table[], uint8_t morph){
    u8g2.clearBuffer();					// clear the internal memory
    //u8g2.drawStr(0,10,"WaveTable");	// write something to the internal memory
    int inv_morph = 0xFF - morph;
    int j = 0;
    int y1;
    y1 =  table[0] * inv_morph;
    y1 += table[128] * morph;
    y1 >>= 8;
    int y2= 0;
    for (int i = 0; i < 128; i++) {
      y2 =  table[i] * inv_morph;
      y2 += table[i+128] * morph;
      y2 >>= 8;
      u8g2.drawLine(j, y1, i, y2);
      j=i;
      y1 = y2;
    }
    u8g2.sendBuffer();
}

void screen_update(uint8_t table[],uint8_t morph){
  if (screen_mode == SCREEN_MODE_FATAL) {
    return;
  }
  //Default back to idle screen mode if current screen mode expired
  if (screen_mode != SCREEN_MODE_IDLE && millis() - screen_mode_keep > SCREEN_KEEP_TIME) {
    screen_mode = SCREEN_MODE_IDLE;
  }
  if (screen_mode == SCREEN_MODE_WAVETABLE) {
    screen_draw_static_wavetable(table, screen_wave_offset);
  }

  if (screen_mode == SCREEN_MODE_IDLE) {
    screen_draw_morphing_wave(table, morph);
  }
}