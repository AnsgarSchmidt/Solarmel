#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>

////////////////// Hardware Settings ///////////////////////////////////////////////////////////////////////
// Digital
#define COMRX_PIN                  0 // Serial Receive PREDEFINED
#define COMTX_PIN                  1 // Serial Transmit PREDEFINED
#define MOUNT_PIN                  2 // Button to mount and unmount SD
#define SAMPLE_NOW_PIN             3 // Button to initiate imidiately sampling
#define TEMP_DATA_PIN              4 // DataPin for tempsensors
#define LOW_MEM_LED_PIN            5 // Low on internal memory InfoLED. Red LED onboard
#define SAMPLING_LED_PIN           6 // Sampling in progress.         Green LED onboard
#define SD_MOUNTED_LED_PIN         7 // Indicates SD is mounted or not
#define SD_WRITE_ERROR_LED_PIN     8 // Indicates an error writing to SD card
#define SD_WRITE_DATA_LED_PIN      9 // Indicates SD access
#define SD_DATA_PIN               10 // Adafruit SD shield
#define MOSI_PIN                  11 // MOSI
#define MISO_PIN                  12 // MISO
#define TEMP_SENSOR_ERROR_LED_PIN 13 // Error with Temp Sensor

// Analog
#define PUMP_WATER_PIN         A0 // Solar water pump
#define BURNER_PIN             A1 // Burner
#define SOLAR_CELL_PIN         A2 // Solar cell voltag sample pin
#define A3_PIN                 A3 // 
#define RTCSDA_PIN             A4 // RTC I2C Interface. Hardwired
#define RTCSCL_PIN             A5 // RTC I2C Interface. Hardwired

// Temperature Sensors
DeviceAddress TEMP_OUTDOOR       = {0x28, 0xB9, 0x08, 0xC3, 0x03, 0x00, 0x00, 0xE4};
DeviceAddress TEMP_TOWARD_FLOR   = {0x28, 0x7B, 0x36, 0xC3, 0x03, 0x00, 0x00, 0x33};
DeviceAddress TEMP_RETURN_FLOW   = {0x28, 0x0D, 0xF9, 0xC2, 0x03, 0x00, 0x00, 0xDB};
DeviceAddress TEMP_BOILER_TOP    = {0x28, 0x8B, 0x1D, 0xC3, 0x03, 0x00, 0x00, 0xC9};
DeviceAddress TEMP_BOILER_MIDDLE = {0x28, 0xF5, 0x06, 0xC3, 0x03, 0x00, 0x00, 0x4E};
DeviceAddress TEMP_BOILER_BOTTOM = {0x28, 0xD9, 0x0A, 0xC3, 0x03, 0x00, 0x00, 0xA4};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// Constants //////////////////////////////////////////////////////////////////////////////
#define DEBOUNCE_DELAY         50 // debounceDelay for Button in millis
#define MOUNT_DELAY         10000 // delay between mount and unmount in millis
#define WRITE_INTERVAL  5*60*1000 // delay for flushing data to SD
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// Vars ///////////////////////////////////////////////////////////////////////////////
// Buttons
long     lastMountDebounceTime = 0;                   // Timestamp for debouncing only neccessary for Mounting. Imidiately sampling is a state
int      lastMountButtonState  = LOW;                 // the previous reading from the input pin
int      mountButtonState      = LOW;                 // the debounced status for input pin
long     lastMountTime         = 0;                   // Timestamp for (un)mounting. 
// Status
int      ismounted             = 0;                   // SD card mounted (in Use)  0=no,1=yes,2=write error
int      newData               = LOW;                 // New Data available to store
// Data
long     sampleDelay           = 10000;               // Time between samples, initial 10 secs will be chanced depending on data
long     lastSampleTime        = 0;                   // Timestamp of last sampling
File     logfile;                                     // Logfile itself
char     logfilename[]         = "1970-01-01-00.csv"; // CurrentFileNameForLogging
DateTime logdate;                                     // Object to store the logfilename as datetime
long     lastsynctime          = 0;                   // Last time the data where written to the SD card
// Temperature
OneWire           oneWire(TEMP_DATA_PIN);
DallasTemperature sensors(&oneWire);

// RAW Data
#define MAX_RAW_DATA 500
struct Data{
  DateTime date;
  float temperature[6];
  float solar;
  boolean pump;
  boolean burn;
  boolean isStored;
};
Data rawdata[MAX_RAW_DATA];

// RTC
RTC_DS1307 RTC;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////// Setup routine /////////////////////////////////////////////////////
void setup(){
  //Pins input
  pinMode(MOUNT_PIN,        INPUT);
  pinMode(SAMPLE_NOW_PIN,   INPUT);
  pinMode(SOLAR_CELL_PIN,   INPUT);

  //Enable Pull up resistors
  digitalWrite(MOUNT_PIN,        HIGH);
  digitalWrite(SAMPLE_NOW_PIN,   HIGH);
  digitalWrite(SOLAR_CELL_PIN,   HIGH);

  //Output Pins
  pinMode(LOW_MEM_LED_PIN,           OUTPUT);
  pinMode(SAMPLING_LED_PIN,          OUTPUT);
  pinMode(SD_MOUNTED_LED_PIN,        OUTPUT);
  pinMode(SD_WRITE_ERROR_LED_PIN,    OUTPUT);
  pinMode(SD_DATA_PIN,               OUTPUT);
  pinMode(TEMP_DATA_PIN,             OUTPUT);
  pinMode(TEMP_SENSOR_ERROR_LED_PIN, OUTPUT);

  // Timers
  lastMountDebounceTime = millis();
  lastMountTime         = millis();
  lastSampleTime        = millis();

  // Initial
  digitalWrite(LOW_MEM_LED_PIN,           LOW);
  digitalWrite(SAMPLING_LED_PIN,          LOW);
  digitalWrite(SD_MOUNTED_LED_PIN,        LOW);
  digitalWrite(SD_WRITE_ERROR_LED_PIN,    LOW);
  digitalWrite(TEMP_SENSOR_ERROR_LED_PIN, LOW);

  // Serial Communication
  Serial.begin(9600); // Slow to make sure the connection is stable

  // RTC
  Wire.begin();
  RTC.begin();
  if (! RTC.isrunning()) {
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  // Temperature 
  sensors.begin();
  sensors.setResolution(TEMP_12_BIT);
  if(checkSensor(TEMP_OUTDOOR)){
    sensors.setResolution(TEMP_OUTDOOR,TEMP_12_BIT);
  }
  if(checkSensor(TEMP_TOWARD_FLOR)){
    sensors.setResolution(TEMP_TOWARD_FLOR,TEMP_12_BIT);
  }
  if(checkSensor(TEMP_RETURN_FLOW)){
    sensors.setResolution(TEMP_RETURN_FLOW,TEMP_12_BIT);
  }
  if(checkSensor(TEMP_BOILER_TOP)){
    sensors.setResolution(TEMP_BOILER_TOP,TEMP_12_BIT);
  }
  if(checkSensor(TEMP_BOILER_MIDDLE)){
    sensors.setResolution(TEMP_BOILER_MIDDLE,TEMP_12_BIT);
  }
  if(checkSensor(TEMP_BOILER_BOTTOM)){
    sensors.setResolution(TEMP_BOILER_BOTTOM,TEMP_12_BIT);
  }

  // Filename
  logdate = DateTime(RTC.now().unixtime() - 7*86400L); // Set last week to force mountHandling to generate new Filename
  mountHandling();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Sensor Handling ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean checkSensor(DeviceAddress insensor){
  if(!sensors.validAddress(insensor)){
    Serial.print("Sensor address not valid:");
    printSensorAddress(insensor);
    Serial.println();
    digitalWrite(TEMP_SENSOR_ERROR_LED_PIN, HIGH);
    return false;
  }
  if(!sensors.isConnected(insensor)){
    Serial.print("Sensor not connected:");
    printSensorAddress(insensor);
    Serial.println();
    digitalWrite(TEMP_SENSOR_ERROR_LED_PIN, HIGH);
    return false;
  }  
  return true;
}

float getTemperature(DeviceAddress insensor){
  if(checkSensor(insensor)){
    float temp = sensors.getTempC(insensor);
    if(temp == DEVICE_DISCONNECTED){
      Serial.print("Sensor error:");
      printSensorAddress(insensor);
      Serial.println();      
      digitalWrite(TEMP_SENSOR_ERROR_LED_PIN, HIGH);
    }else{
      printSensorAddress(insensor);
      Serial.print(" Temperature:");
      Serial.println(temp);
    }
    return temp;
  }
  else{
    return DEVICE_DISCONNECTED;
  }
}

void printSensorAddress(DeviceAddress insensor){
 for(int i=0;i<8;i++){
   Serial.print(insensor[i],HEX);
   if(i<7){
     Serial.print(":");
   }
 } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Button Handling ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void buttonHandling(){
  // Mount
  int reading = digitalRead(MOUNT_PIN);
  if (reading != lastMountButtonState) {
    lastMountDebounceTime = millis();
  } 
  if ((millis() - lastMountDebounceTime) > DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    mountButtonState = reading;
  }
  lastMountButtonState = reading;

  // Sample now
  if(digitalRead(SAMPLE_NOW_PIN) == LOW){
    lastSampleTime = 0; // Reset Sampling Time in order to force a sample;
  }  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// LED Handling ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ledHandling(){
  if(ismounted == 0){
    digitalWrite(SD_MOUNTED_LED_PIN,     LOW);
    digitalWrite(SD_WRITE_ERROR_LED_PIN, LOW);
  }
  else if(ismounted == 1){
    digitalWrite(SD_MOUNTED_LED_PIN,     HIGH);
    digitalWrite(SD_WRITE_ERROR_LED_PIN, LOW );
  }
  else{
    digitalWrite(SD_MOUNTED_LED_PIN,     LOW );
    digitalWrite(SD_WRITE_ERROR_LED_PIN, HIGH);    
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Mount Handling /////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mount(){
  if (SD.begin(SD_DATA_PIN)) {
    if(SD.exists(logfilename)){
      logfile = SD.open(logfilename, FILE_WRITE);
    }else{
      logfile = SD.open(logfilename, FILE_WRITE);
    }        
    if(logfile){
      ismounted = 1;
    }else{
      ismounted = 2;
    }
  }else{
    ismounted = 2;
  }
}
void umount(){
  if(logfile){
    logfile.flush();
    logfile.close();
  }
  ismounted = 0;
}
void mountHandling(){
  if(mountButtonState == LOW && ((millis()-lastMountTime)>MOUNT_DELAY)){ // LOW because of internal pull up resistor
    Serial.println("Toggle mount");
    lastMountTime = millis(); // Reset counter
    // if mounted umount otherwise try to mount
    if(ismounted == 0){     
      mount();
    }else if(ismounted == 1){
      umount();
    }else{
      ismounted = 0;            
    }
  }
  // File name Handling
  DateTime now = RTC.now();
  if((now.year()  != logdate.year() ) || 
     (now.month() != logdate.month()) ||
     (now.day()   != logdate.day()  )
    ){
    logdate = now;
    // day
    logfilename[9] =             now.day()%10      + '0';
    logfilename[8] = (now.day()-(now.day()%10))/10 + '0';
    // month    
    logfilename[6] =               now.month()%10      + '0';
    logfilename[5] = (now.month()-(now.month()%10))/10 + '0';
    // year
    int y1 =  now.year()%10;
    int y2 = (now.year()                 -y1)/10;
    int y3 = (now.year()         -(y2*10)-y1)/10;
    int y4 = (now.year()-(y3*100)-(y2*10)-y1)/10;
    logfilename[3] = y1 + '0';
    logfilename[2] = y2 + '0';
    logfilename[1] = y3 + '0';    
    logfilename[0] = y4 + '0';

    if(ismounted == 1){
      umount();
      mount();
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Sampling Handling //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void samplingHandling(){
  if( (millis()-lastSampleTime)>sampleDelay){
    digitalWrite(SAMPLING_LED_PIN,HIGH);
    Serial.println("Start Sampling");
    lastSampleTime = millis(); // Reset the counter
    sensors.requestTemperatures(); //Request for all Sensors
    Serial.println("Sampling requested now reading");
    rawdata[0].date = RTC.now();
    rawdata[0].temperature[0] = getTemperature(TEMP_OUTDOOR);
    rawdata[0].temperature[1] = getTemperature(TEMP_TOWARD_FLOR);
    rawdata[0].temperature[2] = getTemperature(TEMP_RETURN_FLOW);
    rawdata[0].temperature[3] = getTemperature(TEMP_BOILER_TOP);
    rawdata[0].temperature[4] = getTemperature(TEMP_BOILER_MIDDLE);
    rawdata[0].temperature[5] = getTemperature(TEMP_BOILER_BOTTOM);
    rawdata[0].solar          = analogRead(SOLAR_CELL_PIN);
    rawdata[0].pump           = true;
    rawdata[0].burn           = true;
    rawdata[0].isStored       = false;
    newData = 1;
    Serial.println("End Sampling");
    digitalWrite(SAMPLING_LED_PIN,LOW);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// Write Handling //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void writeHandling(){
  if(newData == 1){
    newData = 0;
    if(ismounted == 1){
      if(logfile){
        // Check for all entries if they are stored alreay
        for(int i=MAX_RAW_DATA-1;i>=0;i--){
          if(rawdata[i].isStored == false){
            // Write entry
            // Time
            logfile.print(rawdata[i].date.year(),DEC);
            logfile.print("-");
            logfile.print(rawdata[i].date.month(),DEC);
            logfile.print("-");
            logfile.print(rawdata[i].date.day(),DEC);
            logfile.print(" ");
            logfile.print(rawdata[i].date.hour(),DEC);
            logfile.print(":");
            logfile.print(rawdata[i].date.minute(),DEC);
            logfile.print(":");
            logfile.print(rawdata[i].date.second(),DEC);
            logfile.print(",");
            logfile.print(rawdata[i].date.unixtime(),DEC);
            for(int t=0;t<6;t++){
              // Temp            
              logfile.print(",");
              logfile.print(rawdata[i].temperature[t],DEC);
            }
            logfile.print(",");
            logfile.print(rawdata[i].solar,DEC);
            logfile.print(",");
            if(rawdata[i].pump == 1){
              logfile.print(1,DEC);
            }else{
              logfile.print(0,DEC);
            }
            logfile.print(",");
            if(rawdata[i].burn == 1){
              logfile.print(1,DEC);
            }else{
              logfile.print(0,DEC);
            }
            logfile.println();
            // Mark as stored on SD
            rawdata[i].isStored = 1;
        
            // Flush data
            if((millis()-lastsynctime)>WRITE_INTERVAL){
              digitalWrite(SD_WRITE_DATA_LED_PIN,HIGH);
              lastsynctime = millis();
              logfile.flush();
              digitalWrite(SD_WRITE_DATA_LED_PIN,LOW);
            }
          }
        }        
      }
      else{
        ismounted = 2;
      }
    }
    // Shift Data
    for(int i=MAX_RAW_DATA-1;i>0;i--){
      rawdata[i] = rawdata[i-1]; 
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// FreeSpace Handling /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void freeSpaceHandling(){
  if(ismounted == 1){
    if(1){
      digitalWrite(LOW_MEM_LED_PIN,HIGH);
    }else{
      digitalWrite(LOW_MEM_LED_PIN,LOW);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Main Loop //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  buttonHandling();
  ledHandling();
  mountHandling();
  samplingHandling();
  writeHandling();
  freeSpaceHandling();
}

