/***********************************************************************
 *  (c) 2016  Frank DD4WH 2016_11_02 - MIT license 
 *  Teensy Bat & Ultrasound detector 
 */         
#define VERSION     " v1.2"
/*  (recording beta using the uSDFS lib)       
 *  https://forum.pjrc.com/threads/38988-Bat-detector
 *         
 *        made possible by the samplerate code by Frank Boesing, thanks Frank!
 * 
 *  tested on Teensy 3.5 + Teensy audio board 
 *  + standard tiny electret MIC soldered to the MIC Input of the audio board 
 *  
 *  needs the new version of the SD library by Paul Stoffregen (newer than 2016_11_01)
 *  
 *  User adjustments - with buttons
 *  
 *  MIC-GAIN                  33 + 34 -
 *  FREQUENCY                 35 + 36 -
 *  SAMPLE RATE               37 + 38 -
 *  WATERFALL/SPECTRUM        39  
 *  RECORD                    31
 *  STOP REC/PLAY             30
 *  PLAY                      29
 *   
 *  uses the SD card slot of the Teensy, NOT the SD card slot of the audio board  
 *    
 * Audio sample rate code - function setI2SFreq  
 * Copyright (c) 2016, Frank Bösing, f.boesing@gmx.de
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * NEW:
 * - uses the uSDFS lib for recording and SD lib for playback
 * - time expansion playback enabled (new sample rates 8k to 32k implemented): 
 *     Try recording a shaken keyring at 96k and playback at 16k, sounds like tons of heavy steel ;-)
 * - waterfall display by Frank B
 * 
 * TODO: 
 * - fix ticking noise issue with "newer" SD cards
 * - implement proper menu for time expansion playback (just choose the time expansion factor)
 * - file names with sample rate indicator
 * - display file names for playback
 * - menu to choose which file to playback
 * - realtime frequency translation of the whole spectrum (no idea how to do this efficiently: maybe just by downsampling???)
 * - activate realtime clock for accurate time/date stamps on recordings
 * - proper documentation/manual ?
 */

//#include <Time.h>
//#include <TimeLib.h>
#include <SD.h>
#include <Audio.h>
//#include <Wire.h>
#include <SPI.h>
#include <Bounce.h>
#include <Metro.h>
#include "ff.h"       // uSDFS lib
#include "ff_utils.h" // uSDFS lib

#include <ILI9341_t3.h>
#include "font_Arial.h"

/*time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
int helpmin; // definitions for time and date adjust - Menu
int helphour;
int helpday;
int helpmonth;
int helpyear;
int helpsec;
uint8_t hour10_old;
uint8_t hour1_old;
uint8_t minute10_old;
uint8_t minute1_old;
uint8_t second10_old;
uint8_t second1_old;
bool timeflag = 0;
*/


#define BACKLIGHT_PIN 0
#define TFT_DC      20
#define TFT_CS      21
#define TFT_RST     32  // 255 = unused. connect to 3.3V
#define TFT_MOSI     7
#define TFT_SCLK    14
#define TFT_MISO    12

// would be nice to use the fast DMA lib, but it does not work on my Teensy 3.5
//ILI9341_t3DMA tft = ILI9341_t3DMA(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

#define BUTTON_MIC_GAIN_P       33     
#define BUTTON_MIC_GAIN_M       34
#define BUTTON_FREQ_P           35     
#define BUTTON_FREQ_M           36     
#define BUTTON_SAMPLE_RATE_P    37     
#define BUTTON_SAMPLE_RATE_M    38     
#define BUTTON_TOGGLE_WATERFALL 39
#define BUTTON_RECORD           31
#define BUTTON_STOP             30
#define BUTTON_PLAY             29

Bounce mic_gain_P = Bounce(BUTTON_MIC_GAIN_P, 50); 
Bounce mic_gain_M = Bounce(BUTTON_MIC_GAIN_M, 50); 
Bounce freq_P = Bounce(BUTTON_FREQ_P, 50); 
Bounce freq_M = Bounce(BUTTON_FREQ_M, 50); 
Bounce sample_rate_P = Bounce(BUTTON_SAMPLE_RATE_P, 50);
Bounce sample_rate_M = Bounce(BUTTON_SAMPLE_RATE_M, 50);
Bounce toggle_waterfall = Bounce(BUTTON_TOGGLE_WATERFALL, 50);
Bounce button_Record = Bounce(BUTTON_RECORD, 50);
Bounce button_Stop = Bounce(BUTTON_STOP, 50);
Bounce button_Play = Bounce(BUTTON_PLAY, 50);
  
#define SAMPLE_RATE_MIN               0
#define SAMPLE_RATE_8K                0
#define SAMPLE_RATE_11K               1
#define SAMPLE_RATE_16K               2  
#define SAMPLE_RATE_22K               3
#define SAMPLE_RATE_32K               4
#define SAMPLE_RATE_44K               5
#define SAMPLE_RATE_48K               6
#define SAMPLE_RATE_88K               7
#define SAMPLE_RATE_96K               8
#define SAMPLE_RATE_176K              9
#define SAMPLE_RATE_192K              10
#define SAMPLE_RATE_MAX               10

// this audio comes from the codec by I2S2
AudioInputI2S               i2s_in; // MIC input
AudioRecordQueue            recorder; 
AudioSynthWaveformSine      sine1; // local oscillator
AudioEffectMultiply         mult1; // multiply = mix

//AudioAnalyzeFFT1024         fft1024_1; // for waterfall display
AudioAnalyzeFFT256          myFFT; // for spectrum display


AudioPlaySdRaw              player; 
AudioMixer4                 mix1;
AudioOutputI2S              i2s_out; // headphone output          
//AudioConnection patch2      (i2s_in, 0, fft1024_1, 0);
AudioConnection patch1      (i2s_in, 0,myFFT, 0);
AudioConnection patch3      (i2s_in, 0, recorder, 0);
AudioConnection patch4      (i2s_in, 0, mix1, 0);
AudioConnection patch5      (player, 0, mix1, 1);
AudioConnection patch6      (mix1, 0, mult1, 0);
AudioConnection patch7      (sine1, 0, mult1, 1);
AudioConnection patch8      (mult1, 0, i2s_out, 0);
AudioConnection patch9      (mult1, 0, i2s_out, 1);

AudioControlSGTL5000        sgtl5000_1;  

// Metro 1 second
Metro second = Metro(1000);
elapsedMillis since_bat_detection1;
elapsedMillis since_bat_detection2;

const int8_t    MODE_STOP = 0;
const int8_t    MODE_REC = 1;
const int8_t    MODE_PLAY = 2;

int mode = MODE_STOP; 
//File frec; // audio is recorded to this file first
int file_number = 0;
FRESULT rc;        /* Result code */
FATFS fatfs;      /* File system object */
FIL fil;        /* File object */

//#define MXFN 100 // maximal number of files 
#if defined(__MK20DX256__)
  #define BUFFSIZE (8*1024) // size of buffer to be written
#elif defined(__MK66FX1M0__)
  #define BUFFSIZE (64*1024) // size of buffer to be written
#endif

uint8_t buffern[BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
UINT wr;
uint32_t nj = 0;


int count_help = 0;
int8_t waterfall_flag = 0;
int idx_t = 0;
int idx = 0;
int64_t sum;
float32_t mean;
int16_t FFT_bin [128]; 
int16_t FFT_max1 = 0;
uint32_t FFT_max_bin1 = 0;
int16_t FFT_mean1 = 0;
int16_t FFT_max2 = 0;
uint32_t FFT_max_bin2 = 0;
int16_t FFT_mean2 = 0;
//int16_t FFT_threshold = 0;
int16_t FFT_bat [3]; // max of 3 frequencies are being displayed
int16_t index_FFT;
int l_limit;
int u_limit;
int index_l_limit;
int index_u_limit;
//const uint16_t FFT_points = 1024;
const uint16_t FFT_points = 256;

int barm [512];

int8_t mic_gain = 40; // start detecting with this MIC_GAIN in dB
int freq_real = 22000; // 22000; // start detecting at this frequency
int sample_rate = SAMPLE_RATE_96K;
int sample_rate_real = 96000;
String text="96k";
int freq_LO = 7000;

typedef struct SR_Descriptor
{
    const int SR_n;
    const char* const f1;
    const char* const f2;
    const char* const f3;
    const char* const f4;
    const float32_t x_factor;
} SR_Desc;

// Text and position for the FFT spectrum display scale
const SR_Descriptor SR [SAMPLE_RATE_MAX + 1] =
{
    //   SR_n ,  f1, f2, f3, f4, x_factor = pixels per f1 kHz in spectrum display
    {  SAMPLE_RATE_8K,  " 1", " 2", " 3", " 4", 64.0}, // which means 64 pixels per 1 kHz
    {  SAMPLE_RATE_11K,  " 1", " 2", " 3", " 4", 43.1}, 
    {  SAMPLE_RATE_16K,  " 2", " 4", " 6", " 8", 64.0}, 
    {  SAMPLE_RATE_22K,  " 2", " 4", " 6", " 8", 43.1}, 
    {  SAMPLE_RATE_32K,  "5", "10", "15", "20", 80.0}, 
    {  SAMPLE_RATE_44K,  "5", "10", "15", "20", 58.05}, 
    {  SAMPLE_RATE_48K,  "5", "10", "15", "20", 53.33},
    {  SAMPLE_RATE_88K,  "10", "20", "30", "40", 58.05},
    {  SAMPLE_RATE_96K,  "10", "20", "30", "40", 53.33},
    {  SAMPLE_RATE_176K,  "20", "40", "60", "80", 58.05},
    {  SAMPLE_RATE_192K,  "20", "40", "60", "80", 53.33} // which means 53.33 pixels per 20kHz
};    

//const int myInput = AUDIO_INPUT_LINEIN;
const int myInput = AUDIO_INPUT_MIC;

// Stop with dying message 
void die(char *str, FRESULT rc);
void setup();
void loop();

extern "C" uint32_t usd_getError(void);
struct tm seconds2tm(uint32_t tt);

void die(char *str, FRESULT rc) 
{ Serial.printf("%s: Failed with rc=%u.\n", str, rc); for (;;) delay(100); }

//=========================================================================
uint32_t count=0;
uint32_t ifn=0;
uint32_t isFileOpen=0;
char filename[80];
TCHAR wfilename[80];
uint32_t t0=0;
uint32_t t1=0;

void blink(uint16_t msec)
{
  digitalWriteFast(13,!digitalReadFast(13)); delay(msec);
}

void setup() {
  Serial.begin(115200);
  delay(200);

//setup pins with pullups
  pinMode(BUTTON_MIC_GAIN_P,INPUT_PULLUP);
  pinMode(BUTTON_MIC_GAIN_M,INPUT_PULLUP);
  pinMode(BUTTON_FREQ_P,INPUT_PULLUP);  
  pinMode(BUTTON_FREQ_M,INPUT_PULLUP);  
  pinMode(BUTTON_SAMPLE_RATE_P,INPUT_PULLUP);  
  pinMode(BUTTON_SAMPLE_RATE_M,INPUT_PULLUP);  
  pinMode(BUTTON_TOGGLE_WATERFALL,INPUT_PULLUP);  
  pinMode(BUTTON_RECORD,INPUT_PULLUP);  
  pinMode(BUTTON_STOP,INPUT_PULLUP);  
  pinMode(BUTTON_PLAY,INPUT_PULLUP);  

  // Audio connections require memory. 
  AudioMemory(60);
/*
  setSyncProvider(getTeensy3Time);
*/
// Enable the audio shield. select input. and enable output
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.8);
  sgtl5000_1.micGain (mic_gain);
  sgtl5000_1.adcHighPassFilterDisable(); // does not help too much!

// Init SD card use
// uses the SD card slot of the Teensy, NOT that of the audio board
// this init only for playback
  if(!(SD.begin(BUILTIN_SDCARD))) 
  {
      while (1) {
          Serial.println("Unable to access the SD card");
          delay(500);  
      }
  }

// Recording on SD card by uSDFS library
  f_mount (&fatfs, (TCHAR *)_T("0:/"), 0);      /* Mount/Unmount a logical drive */




// Init TFT display  
  pinMode( BACKLIGHT_PIN, OUTPUT );
  analogWrite( BACKLIGHT_PIN, 1023 );

  tft.begin();
  tft.setRotation( 3 );
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(14, 7);
  tft.setTextColor(ILI9341_ORANGE);
  tft.setFont(Arial_12);
  tft.print("Teensy Bat Detector  "); tft.print(VERSION);
  tft.setTextColor(ILI9341_WHITE);

  display_settings();

  // sorry, couldn´t resist ;-)
  logo_head();
  for (int v = 0; v < 6; v++) {
  logo(true);
  delay(200);
  logo(false);
  delay(200);
  }

  set_sample_rate (sample_rate);

  set_freq_LO (freq_real);

  mix1.gain(0,1); 
  mix1.gain(1,1); 


} // END SETUP


void loop() {
   controls();
   if (waterfall_flag == 1) 
   {
      waterfall();
   }
   else 
   {
      spectrum();
   }
   
   check_processor();
}

void controls() {

// first, check buttons
  mic_gain_P.update();
  mic_gain_M.update();
  freq_P.update();
  freq_M.update();
  sample_rate_P.update();
  sample_rate_M.update();
  toggle_waterfall.update();
  button_Record.update();
  button_Stop.update();
  button_Play.update();

  // Respond to button presses
  if (button_Record.fallingEdge()) {
    Serial.println("Record Button Press");
    if (mode == MODE_PLAY) stopPlaying();
    if (mode == MODE_STOP) startRecording();
  }
  if (button_Stop.fallingEdge()) {
    Serial.println("Stop Button Press");
    if (mode == MODE_REC) stopRecording();
    if (mode == MODE_PLAY) stopPlaying();
  }
  if (button_Play.fallingEdge()) {
    Serial.println("Play Button Press");
    if (mode == MODE_REC) stopRecording();
    if (mode == MODE_STOP) startPlaying();
  }

  // If we're playing or recording, carry on...
  if (mode == MODE_REC) {
    continueRecording();
  }
  if (mode == MODE_PLAY) {
    continuePlaying();
  }

  // change MIC GAIN  
  if ( mic_gain_P.fallingEdge()) { 
      mic_gain = mic_gain + 2;
      if (mic_gain > 63) {
        mic_gain = 63;
      }
      set_mic_gain(mic_gain);
  }
  if ( mic_gain_M.fallingEdge()) { 
      mic_gain = mic_gain - 2;
      if (mic_gain < 0) {
        mic_gain = 0;
      }
      set_mic_gain(mic_gain);
  }

  // change FREQUENCY of the local oscillator
  if ( freq_P.fallingEdge()) { 
      freq_real = freq_real + 1000;
      if (freq_real > 85000) {
        freq_real = 85000;
      }
      set_freq_LO(freq_real);
  }
  if ( freq_M.fallingEdge()) { 
      freq_real = freq_real - 1000;
            if (freq_real < 0) {
        freq_real = 0;
      }
      set_freq_LO(freq_real);
  }

  // change sample rate
  if ( sample_rate_P.fallingEdge()) { 
      sample_rate = sample_rate + 1;
            if (sample_rate > SAMPLE_RATE_MAX) {
        sample_rate = SAMPLE_RATE_MAX;
      }
        set_sample_rate (sample_rate);
  }
  if ( sample_rate_M.fallingEdge()) { 
      sample_rate = sample_rate - 1;
            if (sample_rate < SAMPLE_RATE_MIN) {
        sample_rate = SAMPLE_RATE_MIN;
      }
        set_sample_rate (sample_rate);
  }
  if ( toggle_waterfall.fallingEdge()) { 
        if (waterfall_flag == 1) 
        {
          waterfall_flag = 0;
          tft.setScroll(0);
          tft.setRotation( 3 );
          tft.fillScreen(ILI9341_BLACK);
          tft.setCursor(14, 7);
          tft.setTextColor(ILI9341_ORANGE);
          tft.setFont(Arial_12);
          tft.print("Teensy Bat Detector  "); tft.print(VERSION);
          tft.setTextColor(ILI9341_WHITE);
          display_settings();
          prepare_spectrum_display();
          logo_head();
          logo(true);
        }
        else 
        {
          waterfall_flag = 1;
        }
  }
} // END function "controls"

void       set_mic_gain(int8_t gain) {
    AudioNoInterrupts();
    sgtl5000_1.micGain (mic_gain);
    AudioInterrupts();
    display_settings();    
} // end function set_mic_gain

void       set_freq_LO(int freq) {
    // audio lib thinks we are still in 44118sps sample rate
    // therefore we have to scale the frequency of the local oscillator
    // in accordance with the REAL sample rate
    freq_LO = freq * (AUDIO_SAMPLE_RATE_EXACT / sample_rate_real); 
    // if we switch to LOWER samples rates, make sure the running LO 
    // frequency is allowed ( < 22k) ! If not, adjust consequently, so that
    // LO freq never goes up 22k, also adjust the variable freq_real  
    if(freq_LO > 22000) {
      freq_LO = 22000;
      freq_real = freq_LO * (sample_rate_real / AUDIO_SAMPLE_RATE_EXACT) + 9;
    }
    AudioNoInterrupts();
    sine1.frequency(freq_LO);
    AudioInterrupts();
    display_settings();
} // END of function set_freq_LO

void      display_settings() {
    tft.fillRect(14,32,200,17,ILI9341_BLACK);
    tft.setCursor(14, 32);
    tft.setFont(Arial_12); 
    tft.print("gain: "); tft.print (mic_gain);
    tft.print("     "); 
    tft.print("freq: "); tft.print (freq_real);
    tft.print("    "); 
    tft.fillRect(232,32,88,17,ILI9341_BLACK);
    tft.setCursor(232, 32);
    tft.print("       "); 
    tft.print (text);
       
 /*  // only for debugging  
    tft.fillRect(0,122,200,17,ILI9341_BLACK);
    tft.setCursor(0, 122);
    tft.print("LO: "); tft.print (freq_LO);
    tft.print("   "); 
    */
}

void      set_sample_rate (int sr) {
  switch (sr) {
    case SAMPLE_RATE_8K:
    sample_rate_real = 8000;
    text = " 8k";
    break;
    case SAMPLE_RATE_11K:
    sample_rate_real = 11025;
    text = "11k";
    break;
    case SAMPLE_RATE_16K:
    sample_rate_real = 16000;
    text = "16k";
    break;
    case SAMPLE_RATE_22K:
    sample_rate_real = 22050;
    text = "22k";
    break;
    case SAMPLE_RATE_32K:
    sample_rate_real = 32000;
    text = "32k";
    break;
    case SAMPLE_RATE_44K:
    sample_rate_real = 44100;
    text = "44.1k";
    break;
    case SAMPLE_RATE_48K:
    sample_rate_real = 48000;
    text = "48k";
    break;
    case SAMPLE_RATE_88K:
    sample_rate_real = 88200;
    text = "88.2k";
    break;
    case SAMPLE_RATE_96K:
    sample_rate_real = 96000;
    text = "96k";
    break;
    case SAMPLE_RATE_176K:
    sample_rate_real = 176400;
    text = "176k";
    break;
    case SAMPLE_RATE_192K:
    sample_rate_real = 192000;
    text = "192k";
    break;
  }
    AudioNoInterrupts();
    setI2SFreq (sample_rate_real); 
    delay(200); // this delay seems to be very essential !
    set_freq_LO (freq_real);
    AudioInterrupts();
    delay(20);
    display_settings();
    prepare_spectrum_display();
} // END function set_sample_rate

void prepare_spectrum_display() {
    int base_y = 211; 
    int b_x = 10;
    int x_f = SR[sample_rate].x_factor;
    tft.fillRect(0,base_y,320,240 - base_y,ILI9341_BLACK);
//    tft.drawFastHLine(b_x, base_y + 2, 256, ILI9341_PURPLE);  
//    tft.drawFastHLine(b_x, base_y + 3, 256, ILI9341_PURPLE);  
    tft.drawFastHLine(b_x, base_y + 2, 256, ILI9341_MAROON);  
    tft.drawFastHLine(b_x, base_y + 3, 256, ILI9341_MAROON);  
    // vertical lines
    tft.drawFastVLine(b_x - 4, base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine(b_x - 3, base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 2 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 2 + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    if(x_f * 3 + b_x < 256+b_x) {
    tft.drawFastVLine( x_f * 3 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 3 + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    }
    if(x_f * 4 + b_x < 256+b_x) {
    tft.drawFastVLine( x_f * 4 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 4 + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    }
    tft.drawFastVLine( x_f * 0.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 1.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 2.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    if(x_f * 3.5 + b_x < 256+b_x) {
    tft.drawFastVLine( x_f * 3.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    }
    if(x_f * 4.5 + b_x < 256+b_x) {
        tft.drawFastVLine( x_f * 4.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    }
    // text
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_9);
    int text_y_offset = 16;
    int text_x_offset = - 5;
    // zero
    tft.setCursor (b_x + text_x_offset, base_y + text_y_offset);
    tft.print("0");
    tft.setCursor (b_x + x_f + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f1);
    tft.setCursor (b_x + x_f * 2 + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f2);
    tft.setCursor (b_x + x_f *3 + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f3);
    tft.setCursor (b_x + x_f *4 + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f4);
//    tft.setCursor (b_x + text_x_offset + 256, base_y + text_y_offset);
    tft.print(" kHz");

    tft.setFont(Arial_14);
} // END prepare_spectrum_display

 void spectrum() { // spectrum analyser code by rheslip - modified
     if (myFFT.available()) {
//     if (fft1024_1.available()) {
    int scale;
    scale = 5;
  for (int16_t x = 2; x < 128; x++) {
//  for (uint16_t x = 8; x < 512; x+=4) {
     FFT_bin[x] = abs(myFFT.output[x]); 
//     FFT_bin[x/4] = abs(fft1024_1.output[x]); 
     int bar = (FFT_bin[x] * scale);
     if (bar >175) bar=175;
     // this is a very simple first order IIR filter to smooth the reaction of the bars
     bar = 0.05 * bar + 0.95 * barm[x]; 

     tft.drawPixel(x*2+10, 210-barm[x], ILI9341_BLACK);
     tft.drawPixel(x*2+10, 210-bar, ILI9341_WHITE);

     barm[x] = bar;
  }
//if (mode == MODE_STOP)  search_bats();     
  } //end if
} // end void spectrum

void  search_bats() {
    // the array FFT_bin contains the results of the 1024 point FFT --> 512 magnitude values
    // we look for bins that have a high amplitude compared to the mean noise, indicating the presence of ultrasound
    // 1. only search in those parts of the array > 14kHz and not around +-10kHz of the LO freq -->
    //    thus it is best, if I search in two parts --> 14kHz to freq_real-10k AND freq_real+10k to sample_rate/2
    // 2. determine mean and max in both parts of the array
    // 3. if we find a bin that is much larger than the mean (take care of situations where mean is zero!) --> identify the no. of the bin
    // 4. determine frequency of that bin (depends on sample_rate_real)
    //    a.) by simply multiplying bin# with bin width
    //    b.) by using an interpolator (not (yet) implemented)
    // 5. display frequency in bold and RED for 1-2 sec. (TODO: also display possible bat species ;-)) and then delete
    // goto 1.    

    // search array in two parts: 
    //  1.)  14k to (freq_real - 10k)
    // upper and lower limits for maximum search
     l_limit = 14000;
     u_limit = freq_real - 10000;
     index_l_limit =  (l_limit * FFT_points / sample_rate_real);  // 1024 !
     index_u_limit =  (u_limit * FFT_points / sample_rate_real);  // 1024 !
//     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);

     if (index_u_limit > index_l_limit) { 
//        arm_max_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_max1, &FFT_max_bin1);
            // this is the efficient CMSIS function to calculate the mean
//        arm_mean_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_mean1);
            // shift bin_max because we have not searched through ALL the 256 FFT bins
     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);
        FFT_max_bin1 = FFT_max_bin1 + index_l_limit;
     }
//     Serial.print(FFT_max1); Serial.print ("  "); Serial.println(FFT_mean1);

    //  2.)  (freq_real + 10k) to 256 
    // upper and lower limits for maximum search
     l_limit = freq_real + 10000;
     if (l_limit < 14000) {
      l_limit = 14000;
     }
     index_l_limit = (l_limit * FFT_points / sample_rate_real); 
     index_u_limit = (FFT_points / 2) - 1; 
     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);
     if (index_u_limit > index_l_limit) { 
//        arm_max_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_max2, &FFT_max_bin2);
            // this is the efficient CMSIS function to calculate the mean
//        arm_mean_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_mean2);
            // shift bin_max because we have not searched through ALL the 128 FFT bins
        FFT_max_bin2 = FFT_max_bin2 + index_l_limit;
     }
//         Serial.print(FFT_max2); Serial.print ("  "); Serial.println(FFT_mean2);

      display_found_bats();
    FFT_max1 = 0;
    FFT_mean1 = 0;
    FFT_max_bin1 = 0;
    FFT_max2 = 0;
    FFT_mean2 = 0;
    FFT_max_bin2 = 0;
}  // END function search_bats()

void display_found_bats() {
    int sign_x = 148; 
    int sign_y = 57; 
    if (since_bat_detection1 > 6000 && since_bat_detection2 > 6000) {
// DELETE red BAT sign
//      Serial.println("RED SIGN DELETED");      
      tft.fillRect(sign_x, sign_y, 64, 22, ILI9341_BLACK);
      tft.fillRect(sign_x + 86, sign_y + 4, 320 - 86 - sign_x, 40, ILI9341_BLACK);
//      since_bat_detection = 0;
    }

// DELETE FREQUENCY 1
    if (since_bat_detection1 > 6000) {
      tft.fillRect(sign_x + 86, sign_y + 4, 320 - 86 - sign_x, 14, ILI9341_BLACK);
//      since_bat_detection1 = 0;
    }

// DELETE FREQUENCY 2
    if (since_bat_detection2 > 6000) {
      tft.fillRect(sign_x + 86, sign_y + 30, 320 - 86 - sign_x, 14, ILI9341_BLACK);
//      since_bat_detection2 = 0;
    }
// PRINT RED BAT SIGN    
    if(FFT_max1 > (FFT_mean1 + 5) || FFT_max2 > (FFT_mean2 + 5)) {
//      Serial.println("BAT");
      tft.fillRect(sign_x, sign_y, 64, 22, ILI9341_RED);
      tft.setTextColor(ILI9341_WHITE);
      tft.setFont(Arial_14);
      tft.setCursor(sign_x + 2, sign_y + 4);
      tft.print("B A T !");      
//      since_bat_detection = 0;
    }
// PRINT frequency 1
    if(FFT_max1 > FFT_mean1 + 5) {
      tft.fillRect(sign_x + 86, sign_y + 4, 320 - 86 - sign_x, 14, ILI9341_BLACK);
      tft.setTextColor(ILI9341_ORANGE);
      tft.setFont(Arial_12);
      tft.setCursor(sign_x + 86, sign_y + 4);
      tft.print(FFT_max_bin1 * sample_rate_real / FFT_points);
      tft.print(" Hz");      
      since_bat_detection1 = 0;
    
//      Serial.print ("max Freq 1: ");
//      Serial.println(FFT_max_bin1 * sample_rate_real / 256);
    }
    
// PRINT frequency 2    
    if(FFT_max2 > FFT_mean2 + 5) { 
      tft.fillRect(sign_x + 86, sign_y + 30, 320 - 86 - sign_x, 14, ILI9341_BLACK);
      tft.setTextColor(ILI9341_ORANGE);
      tft.setFont(Arial_12);
      tft.setCursor(sign_x + 86, sign_y + 30);
      tft.print(FFT_max_bin2 * sample_rate_real / FFT_points);
      tft.print(" Hz");      
//      Serial.print ("max Freq 2: ");
//      Serial.println(FFT_max_bin2 * sample_rate_real / 256);
      since_bat_detection2 = 0;
    }
      tft.setTextColor(ILI9341_WHITE);
} // END function display_found_bats


// set samplerate code by Frank Boesing 
void setI2SFreq(int freq) {
  typedef struct {
    uint8_t mult;
    uint16_t div;
  } tmclk;

  const int numfreqs = 14;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 32000, 44100, (int)44117.64706 , 48000, 88200, (int)44117.64706 * 2, 96000, 176400, (int)44117.64706 * 4, 192000};

#if (F_PLL==16000000)
  const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
#elif (F_PLL==72000000)
  const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
#elif (F_PLL==96000000)
  const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
#elif (F_PLL==120000000)
  const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
#elif (F_PLL==144000000)
  const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
#elif (F_PLL==168000000)
  const tmclk clkArr[numfreqs] = {{32, 2625}, {21, 1250}, {64, 2625}, {21, 625}, {128, 2625}, {42, 625}, {8, 119}, {64, 875}, {84, 625}, {16, 119}, {128, 875}, {168, 625}, {32, 119}, {189, 646} };
#elif (F_PLL==180000000)
  const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255}, {128, 1875}, {107, 853}, {32, 255}, {219, 1604}, {214, 853}, {64, 255}, {219, 802} };
#elif (F_PLL==192000000)
  const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
#elif (F_PLL==216000000)
  const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
#elif (F_PLL==240000000)
  const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
#endif

  for (int f = 0; f < numfreqs; f++) {
    if ( freq == samplefreqs[f] ) {
      while (I2S0_MCR & I2S_MCR_DUF) ;
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return;
    }
  }
}

//  bat logo taken from Shezzy:  
//  http://sisterzpsptreasures.freeforums.org/easy-animated-bat-t77.html
//  copyright free graphics
//  "The image you create by following this tutorial belongs to you and you may do whatever you want with it."
void logo(bool wing1) {
      // Logo ;-)      
     int x = 265;
     int y = 10; 
//      grey background and white line rectangle around it 
     tft.fillRect(x + 23, y - 2,16,14,ILI9341_DARKGREY);
     tft.fillRect(x - 2, y - 2,16,14,ILI9341_DARKGREY);
//     tft.fillRect(x - 2, y - 2, 41, 14,ILI9341_DARKGREY);
    if (wing1) {
     //Wing1
     tft.drawFastHLine(x + 2, y + 0, 5, ILI9341_BLACK);
     tft.drawFastHLine(x + 31, y + 0, 5, ILI9341_BLACK);
     tft.drawPixel(x + 1, y + 1, ILI9341_BLACK);
     tft.drawPixel(x + 35, y + 1, ILI9341_BLACK);
     tft.drawPixel(x + 6, y + 1, ILI9341_BLACK);
     tft.drawPixel(x + 30, y + 1, ILI9341_BLACK);
//     tft.drawPixel(x + 6, y + 2, ILI9341_BLACK);
//     tft.drawPixel(x + 30, y + 2, ILI9341_BLACK);
     tft.drawFastVLine(x , y + 2, 7, ILI9341_BLACK);
     tft.drawFastVLine(x + 36 , y + 2, 7, ILI9341_BLACK);
     tft.drawFastHLine(x + 7, y + 2, 2, ILI9341_BLACK);
     tft.drawFastHLine(x + 28, y + 2, 2, ILI9341_BLACK);
     tft.drawPixel(x + 13, y + 2, ILI9341_BLACK);
     tft.drawPixel(x + 23, y + 2, ILI9341_BLACK);
     tft.drawFastHLine(x + 9, y + 3, 4, ILI9341_BLACK);
     tft.drawFastHLine(x + 24, y + 3, 4, ILI9341_BLACK);
     tft.drawFastHLine(x + 3, y + 5, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 31, y + 5, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 7, y + 5, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 27, y + 5, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 11, y + 5, 2, ILI9341_BLACK);
     tft.drawFastHLine(x + 24, y + 5, 2, ILI9341_BLACK);
     tft.drawPixel(x + 2, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 6, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 10, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 13, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 23, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 26, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 30, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 34, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 1, y + 7, ILI9341_BLACK);
     tft.drawPixel(x + 6, y + 7, ILI9341_BLACK);
     tft.drawPixel(x + 10, y + 7, ILI9341_BLACK);
     tft.drawPixel(x + 26, y + 7, ILI9341_BLACK);
     tft.drawPixel(x + 30, y + 7, ILI9341_BLACK);
     tft.drawPixel(x + 35, y + 7, ILI9341_BLACK);
    }
    else {
     //Wing2
     tft.drawFastHLine(x + 2, y + 2, 5, ILI9341_BLACK);
     tft.drawFastHLine(x + 31, y + 2, 5, ILI9341_BLACK);
     tft.drawPixel(x + 1, y + 3, ILI9341_BLACK);
     tft.drawPixel(x + 35, y + 3, ILI9341_BLACK);
     tft.drawPixel(x + 6, y + 3, ILI9341_BLACK);
     tft.drawPixel(x + 30, y + 3, ILI9341_BLACK);
//     tft.drawPixel(x + 6, y + 4, ILI9341_BLACK);
//     tft.drawPixel(x + 30, y + 4, ILI9341_BLACK);
     tft.drawFastVLine(x , y + 4, 7, ILI9341_BLACK);
     tft.drawFastVLine(x + 36 , y + 4, 7, ILI9341_BLACK);
     tft.drawFastHLine(x + 7, y + 4, 2, ILI9341_BLACK);
     tft.drawFastHLine(x + 28, y + 4, 2, ILI9341_BLACK);
//     tft.drawPixel(x + 13, y + 4, ILI9341_BLACK);
//     tft.drawPixel(x + 23, y + 4, ILI9341_BLACK);
     tft.drawFastHLine(x + 9, y + 5, 5, ILI9341_BLACK);
     tft.drawFastHLine(x + 24, y + 5, 5, ILI9341_BLACK);
     tft.drawFastHLine(x + 3, y + 7, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 31, y + 7, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 7, y + 7, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 27, y + 7, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 11, y + 7, 3, ILI9341_BLACK);
     tft.drawFastHLine(x + 24, y + 7, 3, ILI9341_BLACK);
     tft.drawPixel(x + 2, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 6, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 10, y + 8, ILI9341_BLACK);
//     tft.drawPixel(x + 13, y + 8, ILI9341_BLACK);
//     tft.drawPixel(x + 23, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 26, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 30, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 34, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 1, y + 9, ILI9341_BLACK);
     tft.drawPixel(x + 6, y + 9, ILI9341_BLACK);
     tft.drawPixel(x + 10, y + 9, ILI9341_BLACK);
     tft.drawPixel(x + 26, y + 9, ILI9341_BLACK);
     tft.drawPixel(x + 30, y + 9, ILI9341_BLACK);
     tft.drawPixel(x + 35, y + 9, ILI9341_BLACK);
    }
}

void logo_head() {
     int x = 265;
     int y = 10; 
     tft.fillRect(x + 14, y - 2,9,14,ILI9341_DARKGREY);
     // Head 
     tft.drawPixel(x + 15, y + 2, ILI9341_BLACK);
     tft.drawPixel(x + 21, y + 2, ILI9341_BLACK);
     tft.drawPixel(x + 14, y + 3, ILI9341_BLACK);
     tft.drawPixel(x + 16, y + 3, ILI9341_BLACK);
     tft.drawPixel(x + 20, y + 3, ILI9341_BLACK);
     tft.drawPixel(x + 22, y + 3, ILI9341_BLACK);
     tft.drawPixel(x + 14, y + 4, ILI9341_BLACK);
     tft.drawPixel(x + 22, y + 4, ILI9341_BLACK);
     tft.drawFastHLine(x + 17, y + 4, 3, ILI9341_BLACK);
     tft.drawPixel(x + 14, y + 5, ILI9341_BLACK);
     tft.drawPixel(x + 22, y + 5, ILI9341_BLACK);
     tft.drawPixel(x + 16, y + 5, ILI9341_RED);
     tft.drawPixel(x + 20, y + 5, ILI9341_RED);
     tft.drawPixel(x + 17, y + 6, ILI9341_RED);
     tft.drawPixel(x + 19, y + 6, ILI9341_RED);
     tft.drawPixel(x + 14, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 22, y + 6, ILI9341_BLACK);
     tft.drawPixel(x + 15, y + 7, ILI9341_BLACK);
     tft.drawPixel(x + 21, y + 7, ILI9341_BLACK);
     tft.drawPixel(x + 16, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 20, y + 8, ILI9341_BLACK);
     tft.drawPixel(x + 17, y + 8, ILI9341_WHITE);
     tft.drawPixel(x + 19, y + 8, ILI9341_WHITE);
     tft.drawFastHLine(x + 17, y + 9, 3, ILI9341_BLACK);
}

void check_processor() {
      if (second.check() == 1) {
      Serial.print("Proc = ");
      Serial.print(AudioProcessorUsage());
      Serial.print(" (");    
      Serial.print(AudioProcessorUsageMax());
      Serial.print("),  Mem = ");
      Serial.print(AudioMemoryUsage());
      Serial.print(" (");    
      Serial.print(AudioMemoryUsageMax());
      Serial.println(")");
/*      tft.fillRect(100,120,200,80,ILI9341_BLACK);
      tft.setCursor(10, 120);
      tft.setTextSize(2);
      tft.setTextColor(ILI9341_WHITE);
      tft.setFont(Arial_14);
      tft.print ("Proc = ");
      tft.setCursor(100, 120);
      tft.print (AudioProcessorUsage());
      tft.setCursor(180, 120);
      tft.print (AudioProcessorUsageMax());
      tft.setCursor(10, 150);
      tft.print ("Mem  = ");
      tft.setCursor(100, 150);
      tft.print (AudioMemoryUsage());
      tft.setCursor(180, 150);
      tft.print (AudioMemoryUsageMax());
     */ 
      AudioProcessorUsageMaxReset();
      AudioMemoryUsageMaxReset();
    }
} // END function check_processor


void waterfall(void) // thanks to Frank B !
{ 
    const uint16_t Y_OFFSET = 0;
// code for 256 point FFT 
  static int count = 0;
  uint16_t lbuf[320]; // 320 vertical stripes, each one is the result of one FFT 
  if (myFFT.available()) {
//    for (int i = 0; i < 240; i++) {
    for (int i = 0; i < 240; i++) { // we have 128 FFT bins, we take 120 of them
//      int val = fft1024_1.read(i) * 65536.0; //v1
      int val = myFFT.read(i/2) * 65536.0 * 5; //v1
      //int val = fft1024_1.read(i*2, i*2+1 ) * 65536.0; //v2      
      lbuf[239 - i] = tft.color565(
              min(255, val), //r
              (val/8>255)? 255 : val/8, //g
              ((255-val)>>1) <0? 0: (255-val)>>1 //b
             ); 
    }
//    tft.writeRect(count, 16, 1, 239-16, (uint16_t*) &lbuf);
    // print one FFT result (120 bins) with 240 pixels
    tft.writeRect(count, Y_OFFSET, 1, 239 - Y_OFFSET, (uint16_t*) &lbuf);
    tft.setScroll(319 - count);
    count++;
    if (count >= 320) count = 0;
    count_help = count;
  }

/* // code for 1024 point FFT 
    static int count = 0;
  uint16_t lbuf[320];  
  if (fft1024_1.available()) {
//    for (int i = 0; i < 240; i++) {
    for (int i = 0; i < 480; i+=2) {
//      int val = fft1024_1.read(i) * 65536.0; //v1
      int val = fft1024_1.read(i) * 65536.0 * 5; //v1
      //int val = fft1024_1.read(i*2, i*2+1 ) * 65536.0; //v2      
      lbuf[240-i/2-1] = tft.color565(
              min(255, val), //r
              (val/8>255)? 255 : val/8, //g
              ((255-val)>>1) <0? 0: (255-val)>>1 //b
             ); 
    }
//    tft.writeRect(count, 16, 1, 239-16, (uint16_t*) &lbuf);
    tft.writeRect(count, 32, 1, 239-32, (uint16_t*) &lbuf);
    tft.setScroll(319 - count);
    count++;
    if (count >= 320) count = 0;
    count_help = count;
  }
*/
}

void startRecording() {
  recorder.begin();
  mode = MODE_REC;
  Serial.print("startRecording");
    // close file
    if(isFileOpen)
    {
      //close file
      rc = f_close(&fil);
      if (rc) die("close", rc);
      //
      isFileOpen=0;
    }
  
  if(!isFileOpen)
  {
      file_number++;
  sprintf(filename, "Bat_%u.raw", file_number);
    Serial.println(filename);
  char2tchar(filename, 80, wfilename);
  rc = f_stat (wfilename, 0);
    Serial.printf("stat %d %x\n",rc,fil.obj.sclust);
  rc = f_open (&fil, wfilename, FA_WRITE | FA_CREATE_ALWAYS);
    Serial.printf(" opened %d %x\n\r",rc,fil.obj.sclust);
 
    // check if file is Good
    if(rc == FR_INT_ERR)
    { // only option is to close file
        rc = f_close(&fil);
        if(rc == FR_INVALID_OBJECT)
        { Serial.println("unlinking file");
          rc = f_unlink(wfilename);
          if (rc) {
            die("unlink", rc);
          }
        }
        else
        {
          die("close", rc);
        }
    }
    // retry open file
    rc = f_open(&fil, wfilename, FA_WRITE | FA_CREATE_ALWAYS);
    if(rc) { 
      die("open", rc);
    }
    isFileOpen=1;
  }
}

void continueRecording() {
  const uint32_t N_BUFFER = 1;
  const uint32_t N_LOOPS = 256;
  // buffer size total = 256 * n_buffer * n_loops
  // new plan: use 32*1024 = 32768 byte buffer size
  // queue: write n_buffer blocks * 256 bytes to buffer at a time; free queue buffer;
  // repeat n_loops times ( * n_buffer * 256 = total amount to write at one time)
  // then write to SD card

  if (recorder.available() >= N_BUFFER  ) {// one buffer = 256 (8bit)-bytes = block of 128 16-bit samples
    for (int i = 0; i < N_BUFFER; i++) {
    memcpy(buffern + i*256 + nj * 256 * N_BUFFER, recorder.readBuffer(), 256);
    recorder.freeBuffer();
//    Serial.println(en);
//    en++;
    } 
    if (nj >=  (N_LOOPS - 1)) {
      nj = 0;
    rc = f_write (&fil, buffern, N_BUFFER * 256 * N_LOOPS, &wr);
//    Serial.println ("Writing");
       if (rc== FR_DISK_ERR) // IO error
     {  uint32_t usd_error = usd_getError();
        Serial.printf(" write FR_DISK_ERR : %x\n\r",usd_error);
        // only option is to close file
        // force closing file
     }
     else if(rc) die("write",rc);
    }
  nj ++;
  }

}

void stopRecording() {
  Serial.print("stopRecording");
  recorder.end();
  if (mode == MODE_REC) {
    while (recorder.available() > 0) {
    rc = f_write (&fil, (byte*)recorder.readBuffer(), 256, &wr);
//      frec.write((byte*)recorder.readBuffer(), 256);
      recorder.freeBuffer();
    }
      //close file
      rc = f_close(&fil);
      if (rc) die("close", rc);
      //
      isFileOpen=0;
//    frec.close();
//    playfile = recfile;
  }
  mode = MODE_STOP;
//  clearname();
  Serial.println (" Recording stopped!");
}

void startPlaying() {
//      String NAME = "Bat_"+String(file_number)+".raw";
//      char fi[15];
//      NAME.toCharArray(fi, sizeof(NAME));
      mix1.gain(1,1);
      mix1.gain(0,0);       
  Serial.println("startPlaying ");
  Serial.println ("Playfile: "); Serial.print (filename);
  Serial.println ("Name: "); Serial.print (filename);
  delay(100);
    player.play(filename);
//    player.play("Bat_1.raw");
//    player.play("REC1.RAW");

  mode = MODE_PLAY;
}
  
void continuePlaying() {
  if (!player.isPlaying()) {
    player.stop();
    mode = MODE_STOP;
      mix1.gain(1,0);
      mix1.gain(0,1);       
    Serial.println("End of recording");
  }
}

void stopPlaying() {
      mix1.gain(1,0);
      mix1.gain(0,1);       
  Serial.print("stopPlaying");
  if (mode == MODE_PLAY) player.stop();
  mode = MODE_STOP;
  Serial.println (" Playing stopped");
}

