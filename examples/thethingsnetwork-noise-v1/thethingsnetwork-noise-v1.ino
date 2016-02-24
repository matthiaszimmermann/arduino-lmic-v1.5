/*******************************************************************************
 * noise measuring node for thethingsnetwork (ttn) [1]
 * developed in 2016 as a showcase with a real time noise map for zurich
 * 
 * author: matthias zimmermann
 * 
 * change DEVADDR to a unique address for every new node [2]
 * 
 * written for the hardware setup stated below:
 * - teensy lc [3]
 * - rfm95 [4]
 * - adafruit mems micro [5]
 * 
 * important: this script needs the lmic library [6] to work.
 * 
 * [1] http://thethingsnetwork.org/
 * [2] http://thethingsnetwork.org/wiki/AddressSpace
 * [3] https://www.pjrc.com/teensy/teensyLC.html
 * [4] http://www.hoperf.nl/RFM95W
 * [5] https://www.adafruit.com/products/2716
 * [6] https://github.com/tftelkamp/arduino-lmic-v1.5
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LORA_ENABLE true: connect & send via lora
// LORA_ENABLE false: switch lora communication off
#define LORA_ENABLE true

// PRINT_DEBUG true: print debug info on serial
// PRINT_DEBUG false: don't print debug info to serial
#define PRINT_DEBUG true

// number of bytes for a lora message
#define LORA_MESSAGE_SIZE 12

// analog input noise pin
#define PIN_NOISE A0

// size of inner loop to get microphone min max levels
#define CNT_MAX 3000

// number of samples used to calibrate min and max
#define CALIBRATION_SAMPLES 50

// size of diff histo for calibration
#define CALIBRATION_HISTO_SIZE 10

#define LINE "--------------------------"

// lora end-device address (DevAddr)
// for ttn zurich, please use the address space 5A4801xx
static const u4_t DEVADDR = 0x5A480104; // <-- change this for every node!

// number of seconds to accumulate until tranmitting data
// transmissions start every 20 seconds. over time the interval between transmissions is increasing to 120 seconds
static const float SAMPLE_INTERVAL[] = {20.0, 20.0, 20.0, 30.0, 30.0, 30.0, 30.0, 60.0, 60.0, 120.0};
static const u1_t SAMPLE_INTERVAL_MAX_INDEX = 9;

// lora application identifier (AppEUI)
// not used in this example
static const u1_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// lora DevEUI, unique device ID (LSBF)
// not used in this example
static const u1_t DEVEUI[8]  = { 0x42, 0x42, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// lora network session key (NwkSKey)
// use this key for ttn
static const u1_t DEVKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// lora application session key (AppSKey)
// use this key to get your data decrypted by ttn
static const u1_t ARTKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// gobal vars for sound level
int cnt, val_min, val_max, diff;

// global vars for accumulated noise measures
int acc_max, acc_sum, acc_cnt, samples_cnt;
long acc_start;

// global vars for calibration
int calibration_diff[CALIBRATION_SAMPLES];
int calibration_histo[CALIBRATION_HISTO_SIZE];

// minimum difference for noise reading (updated during calibration)
int diff_min;

// variable to check if node is in transmission mode
boolean nodeIsTransmitting = false;
boolean dataReadyToSend = false;

// char buffer for lora payload data
uint8_t loraData[] = "Hello, world!";

// lora send job
static osjob_t sendJob;

// pin mapping for communication between the teensy lc and the rfm95
lmic_pinmap pins = {
  .nss = 10,
  .rxtx = 7, // not connected on rfm92/rfm95
  .rst = 9,  // needed on rfm92/rfm95
  .dio = {2, 5, 6},
};

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  if(PRINT_DEBUG) {
    Serial.println(LINE);
    Serial.println("Starting setup ...");
  }

  initLora();
  initSensor();
  resetRawSensorValues();
  resetNoiseLevelValues();
  
  if(PRINT_DEBUG) {
    Serial.println(LINE);
  }
}

void loop() {
  sendLora(&sendJob);

  while(1) {
    if(!nodeIsTransmitting) {
      updateRawSensorValues(analogRead(PIN_NOISE));
  
      // update/print current noise level
      if(cnt == CNT_MAX) {
        updateNoiseLevelCalibration();        
        diff = smoothDiff(val_max - val_min, diff_min);
        printDiff(diff, acc_cnt);
        updateNoiseLevelValues(diff);
        resetRawSensorValues();
      }
    }

    os_runloop_once();    
  }
}

// configure lora module
void initLora() {
  if(!LORA_ENABLE) {
    if(PRINT_DEBUG) {
      Serial.println("WARNING: LoRA disabled, not connecting");
    }
    
    return;
  }
  
  if(PRINT_DEBUG) {
    Serial.println("Configuring LoRa module...");
  }
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, (uint8_t*)DEVKEY, (uint8_t*)ARTKEY);
  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  
  if(PRINT_DEBUG) {
    Serial.println("LoRa module ready");
  }
}

// init sensor values and noise related variables
void initSensor() {
  // set unused pins to output
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  for(int i = 15; i <=23; i++) {
    pinMode(i, OUTPUT);
  }

  // set pin for microphone
  pinMode(PIN_NOISE, INPUT);

  // init min noise reading differences
  diff_min = 0;
  
  // reset number of samples sent
  samples_cnt = 0;
}

// reset variables for raw noise measurments
void resetRawSensorValues() {
  cnt = 0;
  val_min = 2000;
  val_max = 0;
  diff = 0;
}

// reset variables for accumulated noise measurments
void resetNoiseLevelValues() {
  acc_cnt = 0;
  acc_sum = 0;
  acc_max = 0;
}

// update variables for raw noise measurments
void updateRawSensorValues(int val) {
  if (val < val_min) { val_min = val; }
  if (val > val_max) { val_max = val; }
  
  cnt++;
}

// update calibration of noise level differences
void updateNoiseLevelCalibration() {
  
  // init calibration histo
  if(acc_cnt == 0) {
    for(int i = 0; i < CALIBRATION_HISTO_SIZE; i++) {
      calibration_histo[i] = 0;
    }
  }
  
  // collecting samples for calibration
  if(acc_cnt < CALIBRATION_SAMPLES) {
    calibration_diff[acc_cnt] = val_max - val_min;
  }
  // computing current calibration
  else if(acc_cnt == CALIBRATION_SAMPLES) {
    // update histogram for calibration
    for(int i = 0; i < CALIBRATION_SAMPLES; i++) {
      if(calibration_diff[i] < CALIBRATION_HISTO_SIZE) {
        calibration_histo[calibration_diff[i]]++;
      }
    }

    // reset diff_min
    diff_min = 0;

    // find best diff_min. 
    // assumpion: most frequent noise diff value represents silence
    for(int i = 1; i < CALIBRATION_HISTO_SIZE; i++) {
      if(calibration_histo[i] > calibration_histo[diff_min]) {
        diff_min = i;
      }
    }

    if(PRINT_DEBUG) {
      Serial.println(LINE);
      Serial.print("new calibration difference: ");
      Serial.println(diff_min);
      Serial.println(LINE);
    }
  }
}

// update variables for accumulated noise measurments
void updateNoiseLevelValues(int d) {
  if(acc_cnt == 0) {
    acc_start = millis();
  }
  
  if (diff > acc_max) { 
    acc_max = d; 
  }

  acc_sum += d;
  acc_cnt++;

  dataReadyToSend = true;
}

void sendLora(osjob_t* job){
  printNoiseLevel();
  prepareDataForTransmission();
  resetNoiseLevelValues();
  
  if(PRINT_DEBUG) {
    Serial.println(LINE);
    if(!LORA_ENABLE) {
      Serial.println("WARNING: Not sending data any data, LoRA is disabled");
    }
  }

  if(LORA_ENABLE && dataReadyToSend) { 
    if(PRINT_DEBUG) {
      Serial.println("Sending data via LoRA");
      Serial.println("aaaabbbbcccc (int values encoded in hex format: a=max, b=sum, c=cnt)");
      Serial.println((char*)loraData);
      Serial.println(LINE);
    
      // Show TX channel (channel numbers are local to LMIC)
      Serial.print("LMIC Time: ");
      Serial.println(millis() / 1000);
      Serial.print("LMIC Send, txCnhl: ");
      Serial.println(LMIC.txChnl);
      Serial.print("LMIC Opmode check: ");
    }
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      if(PRINT_DEBUG) {
        Serial.println(" OP_TXRXPEND, not sending");
        Serial.println(LINE);
      }
    }
    // We are clear to send data
    else {
      if(PRINT_DEBUG) {
        Serial.println("ok");
      }

      nodeIsTransmitting = true;
      
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, loraData, LORA_MESSAGE_SIZE, 0);
    }
  }
    
  // schedule next job to run at the given timestamp (absolute system time)
  os_setTimedCallback(job, os_getTime()+sec2osticks(SAMPLE_INTERVAL[samples_cnt++]), sendLora);

  if(samples_cnt >= SAMPLE_INTERVAL_MAX_INDEX) {
    samples_cnt = SAMPLE_INTERVAL_MAX_INDEX;
  }
}

// lora callback: transmission completed, ...
void onEvent (ev_t ev) {
  switch(ev) {
    // scheduled data sent (optionally data received)
    // note: this includes the receive window!
    case EV_TXCOMPLETE:
      // use this event to keep track of actual transmissions
      if(PRINT_DEBUG) {
        Serial.print("LMIC EV_TXCOMPLETE, time: ");
        Serial.println(millis() / 1000);
      }
      
      if(LMIC.dataLen) { // data received in rx slot after tx
        //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
        if(PRINT_DEBUG) {
          Serial.println("LMIC Data Received!");
        }
      }

      if(PRINT_DEBUG) {
        Serial.println(LINE);
      }
      
      nodeIsTransmitting = false;    
      break;
      
    default:
      break;
  }
}

// encode acc_max, acc_sum, acc_cnt to loraData array in hex format
// loraData then contains the 0 terminated string aaaabbbbcccc (int values encoded in hex format: a=max, b=sum, c=cnt)
void prepareDataForTransmission() {
  byte high, low;
  high = highByte(acc_max);
  low = lowByte(acc_max);
  loraData[0] = highNibble(high);
  loraData[1] = lowNibble(high);
  loraData[2] = highNibble(low);
  loraData[3] = lowNibble(low);

  high = highByte(acc_sum);
  low = lowByte(acc_sum);
  loraData[4] = highNibble(high);
  loraData[5] = lowNibble(high);
  loraData[6] = highNibble(low);
  loraData[7] = lowNibble(low);

  high = highByte(acc_cnt);
  low = lowByte(acc_cnt);
  loraData[8] = highNibble(high);
  loraData[9] = lowNibble(high);
  loraData[10] = highNibble(low);
  loraData[11] = lowNibble(low);

  loraData[12] = 0;
}

byte highNibble(byte value) {
  byte b = value >> 4;
  return b > 9 ? b + 0x37 : b + 0x30;
}

byte lowNibble(byte value) {
  byte b = value & 0xF;
  return b > 9 ? b + 0x37 : b + 0x30;
}

// get normalizes accumulated noise
double getNoiseLevelNormalized() {
  return acc_sum / ((millis() - acc_start) / 1000.0);
}

double sampleTime() {
  return (millis() - acc_start) / 1000.0;
}

// diff values below the calibrated minimum difference are not meaningful
int smoothDiff(int d, int dMin) {
  if(d < dMin) {
    return 0;
  }
  else {
    return d - dMin;
  }
}

// print current noise level
void printDiff(int d, int cnt) {
  if(PRINT_DEBUG) {
    Serial.print(getDiffString(d));   
    Serial.print(" ");   
    Serial.print(d);   
    Serial.print(" (min ");   
    Serial.print(val_min);   
    Serial.print(" max ");   
    Serial.print(val_max);   
    Serial.print(") cnt=");
    Serial.print(cnt);
    Serial.print(" time=");
    Serial.println(sampleTime());
  }
}

// converts diff value into a string for printing on serial
String getDiffString(int d) {
  int noise = d / 2;
  int pos = 1;
  String bar;

  if(diff == 0) { bar = "."; }
  else          { bar = "#"; }
  
  for(;pos < 40; pos++) {    
    if(pos <= noise) { bar += "#"; }
    else             { bar += " "; }
  }

  return bar;
}

// print accumulated noise info
void printNoiseLevel() {
  if(PRINT_DEBUG) {
    Serial.println(LINE);
    Serial.print("Noise ");   
    Serial.print(getNoiseLevelNormalized());
    Serial.print(" (max ");   
    Serial.print(acc_max);   
    Serial.print(" sum ");   
    Serial.print(acc_sum);  
    Serial.print(" cnt ");   
    Serial.print(acc_cnt);       
    Serial.print(")");
    Serial.println();
  }
}

// provide application router id (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

// provide device id (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, DEVKEY, 16);
}

