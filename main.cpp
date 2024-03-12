/* This version uses MBED Tasks for each of the primary radio firmware functions:

This version uses a Waveshare RP2040 Pico Zero Controller
and a Si47xx receiver, and a Class E transmitter.
The transmitter power can be 12V, or 5V from the USB
The RP2040 can only be powered from USB as it has no diode protection on 5V

Notes:
 The Si47XX can only tune in 1 KHz steps. To get a finer frequency you must use the BFO frequncy.

Setup:

 Setup Serial
 Setup ADC with DMA
 Setup USB
 Setup Si5351 Clocks
 Setup Si47XX receiver

Tasks:

1. ADC sampling and USB Audio Write to PC
2. CAT serial protocol
3. USB Audio read from PC - determines transmit freq
4. Receiver control 

Main loop:

Control


Pico Connections:

 P3  Si4732 RST
 P26 (A0) ADC in
 P4/5  Si5351 I2C
 PA6  RX_ENABLE (H for RX, L for TX)

I2C Addresses:

 Si5351
 Si4732 0x62 

MPM Sep 2023
    updated Jan 2024
    updated Mar 12 2024 for V2 PCB version

*/


#include <Arduino.h>
#include <Wire.h>
#include <rtos.h>
#include <pico/multicore.h>
#include "dma_adc.h"

// My tuning parameters
#include "simple_digital.h"

using namespace std;
using namespace rtos;

#define MS(x) chrono::milliseconds(x)


byte lights = 0;

uint32_t RF_freq = 14074000UL; // freq in Hz

//#define CALIBRATE 1
#define DEBUGGING 1
// for testing - control CAT
//#define DOCAT 1


// The RP2040 Zero module only has one LED, a WS2812
#include <Adafruit_NeoPixel.h>  
#define PIN_NEOPIXEL 16
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
uint8_t color = 0;
#define BRIGHTNESS 5  //max 255
// R, G, B, RG, RB, GB, W, BLK
uint32_t colors[] = {
  pixels.Color(BRIGHTNESS, 0, 0), pixels.Color(0, BRIGHTNESS, 0), pixels.Color(0, 0, BRIGHTNESS),
  pixels.Color(BRIGHTNESS, BRIGHTNESS, 0), pixels.Color(BRIGHTNESS, 0, 2),
  pixels.Color(0, BRIGHTNESS, BRIGHTNESS), pixels.Color(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS),
  pixels.Color( 0, 0, 0)};

uint16_t adc_val = 0;

// Si5351
// Clk0 - Transmit clock
// Clk1 - 32768 KHz as RCLK to Si47XX
// Clk2 - Calibrate clock
#include <si5351.h>
Si5351 si5351;
bool i2c_found = false;


// Si4732
#include <SI4735.h>
SI4735 rx;
#include <patch_init.h> // SSB patch for whole SSBRX initialization string

const uint16_t size_content = sizeof ssb_patch_content; // see patch_init.h
bool ssbLoaded = false;
#define SSB_LSB 1
#define SSB_USB 2

#define RX_RST 3

#define RX_ENABLE 6
void loadSSB();

// USB Audio IO
#include "PluggableUSBAudio.h"
#define AUDIOSAMPLING 48000  // USB Audio sampling frequency
USBAudio audio(true, AUDIOSAMPLING, 2, AUDIOSAMPLING, 2);
static uint8_t readBuffer[192];  //48 samples (=  0.5 ms at 48000 Hz sampling) data sent from PC are received (16bit stero; 48*2*2).
int16_t writeBuffer16[96];       //48 samples of data are written to PC in one packet (stereo).
uint16_t writeCounter=0;
uint16_t nBytes=0;
int16_t monodata[48];

// For determination of Audio signal frequency 
int16_t mono_prev=0;  
int16_t mono_preprev=0;  
float delta_prev=0;
int16_t sampling=0;
int16_t cycle=0;
uint32_t cycle_frequency[136];

// TX/RX control
int Tx_Status = 0; // 0=RX, 1=TX
int Tx_Start = 0;  // 0=RX, 1=TX
int not_TX_first = 0;
uint32_t Tx_last_mod_time;
uint32_t Tx_last_time;
uint32_t push_last_time;  // to detect the long puch for frequency change by push switch
  
volatile bool USBAudio_read_in;
//volatile bool receiving;

void USBAudioWrite(int16_t left, int16_t right);
void USBAudioRead();
void change_freq(uint32_t f);

// Setups

// ADC configuration
#define ADC_BUFFER_SIZE 128

// configuration
const struct dma_adc_config config = {
    // GPIO to use for input, must be ADC compatible (GPIO 26 - 28)
    .gpio = 26,
    // sample rate in Hz
    .sample_rate = 16000,
    // number of samples to buffer
    .sample_buffer_size = ADC_BUFFER_SIZE,
};

// variables
int16_t sample_buffer[ADC_BUFFER_SIZE];

volatile int samples_read = 0;

void on_analog_samples_ready()
{
    // callback from library when all the samples in the library
    // internal sample buffer are ready for reading 
    samples_read = dma_adc_read(sample_buffer, ADC_BUFFER_SIZE);
}

void adc_setup() {
 
  // initialize the dma adc
  if (dma_adc_init(&config) < 0) {
      Serial.println("dma adc failed!");
      while (1) { Serial.println("init failed"); delay(1000); }
  }

   // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    dma_adc_set_samples_ready_handler(on_analog_samples_ready);

      // start capturing data from adc
    if (dma_adc_start() < 0) {
        Serial.println("dma adc start failed!");
        while (1) { Serial.println("start failed"); delay(1000);  }
    }
            
}


void receiver_setup() {
  byte rx_i2c;

  // Check RX I2C
  rx_i2c = rx.getDeviceI2CAddress(RX_RST); // must align predicted and stored I2C addresses
  i2c_found = (rx_i2c > 0);
  if (!i2c_found) return; // cant find the radio receiver
  // Setup Si4732 chip
  rx.setRefClock(32768);      // Ref = 32768Hz
  rx.setRefClockPrescaler(1); // 32768 x 1 = 32768Hz
  rx.setup(RX_RST, -1, AM_CURRENT_MODE, SI473X_ANALOG_AUDIO, XOSCEN_RCLK); // Analog and external RCLK
  delay(100);
  #if 1
  loadSSB();
  rx.setSSB(14000,14350,RF_freq/1000,1,SSB_USB); //14074 USB default
  rx.setSSBSidebandCutoffFilter(1);
  rx.setSSBAutomaticVolumeControl(1);
  rx.setSSBAudioBandwidth(2);
  //rx.setSSBBfo(500);
  #else
  rx.setAM(570,1500,1080,10);
  //rx.setAGC(0,0);
  #endif
 // rx.setAvcAmMaxGain(90);
  delay(200);
  rx.setVolume(56);

}  


void clocks_setup() {
  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_6PF,0, 0);
  if (i2c_found) {
 
  si5351.set_correction(-300, SI5351_PLL_INPUT_XO);

 // RX clock
  si5351.set_freq(32768UL*100UL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_4MA);
// TX clock
  si5351.set_freq(14000000UL*100UL, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);

#ifdef CALIBRATION
// Calibration
  si5351.set_freq(10000000UL*100UL, SI5351_CLK2);
  si5351.output_enable(SI5351_CLK2, 1);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
#endif
  }
}

//
// main functions
//
int i = 0;

int16_t bias = 800; // should correct this dc bias

// Switch from transmit to receive
void receive() {
  si5351.output_enable(SI5351_CLK0, 0);  // TX Off
  digitalWrite(RX_ENABLE, 1); // we are done, turn RX back on
  Tx_Status = 0;
  pixels.setPixelColor(0, colors[7]); // Black
  pixels.show();
  for (int i = 0; i <48; i++) { // clean usb input data
    monodata[i] = 0;
  } 
}


// Keep reading from receiver and sending to USB
void receiving() {
  int32_t x;
  int sample_count;

  USBAudioRead(); // check for transmit audio
    if (USBAudio_read_in) {
      not_TX_first = 0;
      Tx_Start = 1;
      return; // change to transmit path
    }


  if (samples_read > 0) {
  sample_count = samples_read;
  i = 0;
 
  while (sample_count > 0) {
    x = (sample_buffer[i] + sample_buffer[i+1])/2;
    x =  x - bias;
    for (int j = 0; j<6; j++) USBAudioWrite(x, x);
    i += 2;
    sample_count -=2;
   } // sample while loop
  
  }
  
 }


void cat() {
  String receivedPart1;
  String receivedPart2;    
  String command;
  String command2;  
  String parameter;
  String parameter2; 
  String sent;
  String sent2;
  String data = "";
 
  uint32_t freq = RF_freq;
  int bufferIndex = 0;
  
  char received;
  while(Serial.available() > 0){    
    received = Serial.read();  
    if (received != ';') {
      if('a' <= received && received <= 'z'){
        received = received - ('a' - 'A');
      }
      if(('A' <= received && received <= 'Z') || ('0' <= received && received <= '9')){
        data += received;
      }
    }
    else { 
      if (bufferIndex == 0) {        
        data += '\0';
        receivedPart1 = data;
        data = "";
        bufferIndex ++;
      }
      else {
        data += '\0';
        receivedPart2 = data;
        data = "";
        bufferIndex ++;
      }
    }
  }     

  command = receivedPart1.substring(0,2);
  command2 = receivedPart2.substring(0,2);    
  parameter = receivedPart1.substring(2,receivedPart1.length());
  parameter2 = receivedPart2.substring(2,receivedPart2.length());


  if (command == "FA")  {          
    if (parameter != "")  {          
      long int freqset = parameter.toInt();
      if (freqset >= 1000000 && freqset <= 54000000) freq = freqset;
        change_freq(freq);
        delay(50);
    }          
    sent = "FA" // Return 11 digit frequency in Hz.  
    + String("00000000000").substring(0,11-(String(freq).length()))   
    + String(freq) + ";";     
  }
  else if (command == "FB")  {          
    if (parameter != "")  {          
      long int freqset = parameter.toInt();
      if (freqset >= 1000000 && freqset <= 54000000) freq = freqset;
        change_freq(freq);
        delay(50);
    }          
    sent = "FB" // Return 11 digit frequency in Hz.  
    + String("00000000000").substring(0,11-(String(freq).length()))   
    + String(freq) + ";";     
  }  
  else if (command == "IF")  {          
    sent = "IF" // Return 11 digit frequency in Hz.  
    + String("00000000000").substring(0,11-(String((long int)freq).length()))   
    + String((long int)freq) + "0001+00000" + "00000" + String(Tx_Status).substring(0,1) + "20000000;";     //USB  
  }
  else if (command == "MD")  {          
    sent = "MD2;";                     //USB  
  }
  else  if (command == "ID")  {  
    sent = "ID019;";
  }
  else  if (command == "PS")  {  
    sent = "PS1;";
  }
  else  if (command == "AI")  {  
    sent = "AI0;";
  }
  else  if (command == "RX")  {  
    sent = "RX0;";
  }
  else  if (command == "TX")  {  
    sent = "TX0;";
  }
  else  if (command == "AG")  {  
    sent = "AG0000;";
  }
  else  if (command == "XT")  {  
    sent = "XT0;";
  }
  else  if (command == "RT")  {  
    sent = "RT0;";
  }
  else  if (command == "RC")  {  
    sent = ";";
  }
  else  if (command == "RS")  {  
    sent = "RS0;";
  }
  else  if (command == "VX")  {  
    sent = "VX0;";
  }
  else  if (command == "SA")  {  
    sent = "SA000000000000000;";
  }
  //else  {
  //  sent = String("?;");
  //}
//------------------------------------------------------------------------------ 
  if (command2 == "ID")   {  
    sent2 = "ID019;";
  }
  //else  {
  //  sent2 = String("?;");
  //}               
  Serial.print(sent);
  if (bufferIndex == 2)  {
    Serial.print(sent2);
  }
  Serial.flush();   
}


// Helpers

void flash_led(uint32_t col, uint32_t ontime, uint32_t offtime) {
    pixels.setPixelColor(0, col);
    pixels.show();
    delay(ontime);
    pixels.setPixelColor(0, colors[7]);
    pixels.show();
    delay(offtime);
}

// Change Si4735 rx/tx frequency
// Use BFO if not on a 1 KHz boundary
// f is in Hz
void change_freq(uint32_t f) {
    int h,l;
    if (f == RF_freq) return; // no change
    h = f / 1000;
    l = f - 1000*h;
    rx.setFrequency(h);
    if (l > 0) rx.setSSBBfo(-l);
    RF_freq = f;
}

// Send usb audio data to the PC
void USBAudioWrite(int16_t left,int16_t right) {
  if(nBytes>191){
    uint8_t *writeBuffer =  (uint8_t *)writeBuffer16;
    audio.write(writeBuffer, 192);
    writeCounter =0;
    nBytes =0;
  }
  writeBuffer16[writeCounter]=left;
  writeCounter++;
  nBytes+=2;
  writeBuffer16[writeCounter]=right;
  writeCounter++;
  nBytes+=2;
}

// Fetch usb audio from PC into the radio (for control of transmittion)
void USBAudioRead() {
  int32_t monosum=0;
 
  USBAudio_read_in = audio.read(readBuffer, sizeof(readBuffer));
 
  if (USBAudio_read_in) {
    for (int i = 0; i < 48 ; i++) {
      int16_t outL = (readBuffer[4*i]) + (readBuffer[4*i+1] << 8);
      int16_t outR = (readBuffer[4*i+2]) + (readBuffer[4*i+3] << 8);
      int16_t mono = (outL+outR)/2;
      monosum += mono;
      monodata[i] = mono;
    }
  }
  if (monosum == 0) USBAudio_read_in = false;

}

void loadSSB()
{
  rx.setI2CFastModeCustom(400000); // You can try rx.setI2CFastModeCustom(700000); or greater value
  rx.loadPatch(ssb_patch_content, size_content, 2);
  rx.setI2CFastModeCustom(100000);
  ssbLoaded = true;
}

// Setup

void setup() {
    Serial.begin(115200);
    delay(500);

    adc_setup();

    clocks_setup();

    if (!i2c_found) {
      // Just flash red forever -  clock not found
      while (1) {
        flash_led(colors[0], 200,400);
      }
    }
#if 1
    receiver_setup();
        if (!i2c_found) {
      // Just flash purple forever -  receiver not found
        while (1) {
          flash_led(colors[4], 200, 400);
        }
    }
#endif
   flash_led(colors[6], 200, 0); //white flash - setup OK

     // initialization of monodata[] 
  for (int i = 0; i < 48; i++) {
    monodata[i] = 0;
  } 


//USBAudioRead();

#ifdef DEBUGGING
#define DBG_PIN 8
pinMode(DBG_PIN, OUTPUT);

#endif  
}


// Transmit code - samples input audio and adjusts tx (CLK0) frequency
// The samples arrive in a buffer, we need to look for +/- transitions and measure the
// period to determine the delta in transmit frequency.

void transmit(int64_t tx_freq) {
  if (Tx_Status == 0){
    digitalWrite(RX_ENABLE, 0); // RX->TX
    si5351.output_enable(SI5351_CLK0, 1); 
    Tx_Status = 1;
    pixels.setPixelColor(0, colors[0]); // RED = TX ON
    pixels.show();
  }
  si5351.set_freq(RF_freq*100UL + tx_freq, SI5351_CLK0);
 // si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
 // si5351.output_enable(SI5351_CLK0, 1);  // TEST
}


void transmitting() {
  int i;
  int16_t mono, difference;
  float delta, period;
  uint64_t audio_freq;
  

  //#ifdef DEBUGGING
  //digitalWrite(DBG_PIN, USBAudio_read_in);
  //#endif

  if (USBAudio_read_in) {
  #ifdef DEBUGGING
 //   digitalWrite(DBG_PIN, USBAudio_read_in);
  #endif
    for (i = 0; i < 48; i++) {
      mono = monodata[i];

      if ((mono_prev < 0) && (mono >= 0)) { // zero crossing
        if ((mono == 0) && (((double)mono_prev * 1.8 - (double)mono_preprev < 0.0) || ((double)mono_prev * 2.02 - (double)mono_preprev > 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
          Tx_Start = 0;
          break;
       }
      difference = mono - mono_prev;
      delta = (float) mono_prev / (float) difference;
      period = ((float) 1.0 + delta_prev) + (float) sampling - delta;
      audio_freq = (uint64_t)(AUDIOSAMPLING*100.0/(double) period); // in 0.01 Hz 

      if ((audio_freq > FSK_MIN) && (audio_freq < FSK_MAX)){
          cycle_frequency[cycle]=(uint32_t)audio_freq;
          cycle++;
        }
        // Store previous values
        delta_prev = delta;
        sampling = 0;
        mono_preprev = mono_prev;
        mono_prev = mono;     
      } 
      else if ((not_TX_first == 1) && (mono_prev == 0) && (mono == 0)) {        //Detect non-transmission
        Tx_Start = 0;
        break;
       } 
       else {
    // no crossing, just move on
      sampling++;
      mono_preprev = mono_prev;
      mono_prev = mono;
      }
      
    } // end of usb read input loop for this pass
    if (Tx_Start == 0){
      cycle = 0;
  //    sampling = 0;
  //    mono_preprev = mono_prev = 0;
      receive();
      return;
    }

    /*------------
      This is a cycle throttle meassurement where even if a frequency change has been detected no
      change will be propagated to the Si5351 unless a minimum of FSK_THRESHOLD (msec) has been elapsed
      avoiding cluttering the I2C bus with noise
    */  
    if ((cycle > 0) && ((millis() - Tx_last_mod_time) > FSK_THRESHOLD)) { 
      audio_freq = 0;
      for (int i=0; i<cycle; i++){
        audio_freq += cycle_frequency[i];
      }
      audio_freq = audio_freq / (uint64_t) cycle; // average the audio freqs

   //   long unsigned freqdiff=abs((long int)audio_freq-(long int)last_audio_freq);
      
    //  if (freqdiff > FSK_MIN_CHANGE) {
         transmit(audio_freq);
   //   }   
      cycle = 0;
      not_TX_first = 1;
      Tx_last_mod_time = millis();
   //   last_audio_freq = audio_freq;
    }
    
  not_TX_first = 1;
  Tx_last_time = millis();
   // USBAudioRead(); // get more audio from PC
  } // end of main USB read in
  else if  ((millis() - Tx_last_time) > 100) {     // If USBaudio data is not received for more than 50 ms during transmission, the system moves to receiving. 
    Tx_Start = 0;
    cycle = 0;
    sampling = 0;
    mono_preprev = 0;
    mono_prev = 0;
    receive();
    return;
  }
 
   #ifdef DEBUGGING
 // digitalWrite(DBG_PIN, USBAudio_read_in);
  #endif
  // Start another usb read
  USBAudioRead();
}

// Main Loop

void loop() {
  
  if (Serial.available()) cat();
  if (Tx_Start == 0) receiving();
  else transmitting();
  digitalWrite(DBG_PIN, Tx_Start);

}
