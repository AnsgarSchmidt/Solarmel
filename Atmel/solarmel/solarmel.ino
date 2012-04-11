#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

////////////////// Hardware Settings ///////////////////////////////////////////////////////////////////////
// Input Buttons
#define MOUNT_PIN               0 // Button to mount and unmount SD
#define SAMPLE_NOW_PIN          1 // Button to initiate imidiately sampling

// Input Sensor
#define TEMP_DATA_PIN          A1 // DataPin for tempsensors
#define SOLAR_CELL_PIN         A0 // Solar cell voltag sample pin

// Output LED
#define LOW_MEM_LED_PIN         3 // Low on internal memory InfoLED. Red LED onboard
#define SAMPLING_LED_PIN        4 // Sampling in progress.         Green LED onboard
#define SD_MOUNTED_LED_PIN      5 // Indicates SD is mounted or not
#define SD_WRITE_ERROR_LED_PIN  6 // Indicates an error writing to SD card
#define SD_WRITE_DATA_LED_PIN   7 // Indicates SD access

#define RTCSDA                 A4 // RTC I2C Interface. Hardwired
#define RTCSCL                 A5 // RTC I2C Interface. Hardwired
#define SD_DATA_PIN            10 // Adafruit SD shield
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////// Constants //////////////////////////////////////////////////////////////////////////////
#define DEBOUNCE_DELAY         50 // debounceDelay for Button in millis
#define MOUNT_DELAY         10000 // delay between mount and unmount in millis
#define WRITE_INTERVAL  5*60*1000 // delay for flushing data to SD
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// Vars ///////////////////////////////////////////////////////////////////////////////
// Buttons
long     lastMountDebounceTime = 0;                // Timestamp for debouncing only neccessary for Mounting. Imidiately sampling is a state
int      lastMountButtonState  = LOW;              // the previous reading from the input pin
int      mountButtonState      = LOW;              // the debounced status for input pin
long     lastMountTime         = 0;                // Timestamp for (un)mounting. 
// Status
int      ismounted             = 0;                // SD card mounted (in Use)  0=no,1=yes,2=write error
int      newData               = LOW;              // New Data available to store
// Data
long     sampleDelay           = 10000;            // Time between samples, initial 10 secs will be chanced depending on data
long     lastSampleTime        = 0;                // Timestamp of last sampling
File     logfile;                                  // Logfile itself
char     logfilename[]         = "1970-01-01.csv"; // CurrentFileNameForLogging
DateTime logdate;
long     lastsynctime          = 0;                // Last time the data where written to the SD card
// RAW Data
struct Data{
  DateTime date;
  float temperature[6];
  float solar;
  int pump;
  int burn;
  int oil;  
};
Data rawdata[2];

// RTC
RTC_DS1307 RTC;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////// Setup routine /////////////////////////////////////////////////////
void setup(){
  // Pins
  pinMode(MOUNT_PIN,        INPUT);
  pinMode(SAMPLE_NOW_PIN,   INPUT);
  pinMode(SOLAR_CELL_PIN,   INPUT);

  pinMode(LOW_MEM_LED_PIN,        OUTPUT);
  pinMode(SAMPLING_LED_PIN,       OUTPUT);
  pinMode(SD_MOUNTED_LED_PIN,     OUTPUT);
  pinMode(SD_WRITE_ERROR_LED_PIN, OUTPUT);
  pinMode(SD_DATA_PIN,            OUTPUT);
  pinMode(TEMP_DATA_PIN,          OUTPUT);

  // Timers
  lastMountDebounceTime = millis();
  lastMountTime         = millis();
  lastSampleTime        = millis();

  // Initial
  digitalWrite(LOW_MEM_LED_PIN,        LOW);
  digitalWrite(SAMPLING_LED_PIN,       LOW);
  digitalWrite(SD_MOUNTED_LED_PIN,     LOW);
  digitalWrite(SD_WRITE_ERROR_LED_PIN, LOW);

  // RTC
  Wire.begin();
  RTC.begin();
  if (! RTC.isrunning()) {
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  Serial.begin(9600); // Slow to make sure the connection is stable

  // Filename
  logdate = DateTime(RTC.now().unixtime() - 7*86400L); // Set yesterday to force mountHandling to generate new Filename
  mountHandling();

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
  if(digitalRead(SAMPLE_NOW_PIN)){
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
    digitalWrite(LOW_MEM_LED_PIN,        LOW);
  }else if(ismounted == 1){
    digitalWrite(SD_MOUNTED_LED_PIN,     HIGH);
    digitalWrite(SD_WRITE_ERROR_LED_PIN, LOW );
    //digitalWrite(LOW_MEM_LED_PIN,      LOW);
  }else{
    digitalWrite(SD_MOUNTED_LED_PIN,     LOW );
    digitalWrite(SD_WRITE_ERROR_LED_PIN, HIGH);    
    //digitalWrite(LOW_MEM_LED_PIN,      LOW);
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
}
void mountHandling(){
  if(mountButtonState == HIGH && ((millis()-lastMountTime)>MOUNT_DELAY)){
    lastMountTime = millis(); // Reset counter
    // if mounted umount otherwise try to mount
    if(ismounted == 0){     
      mount();
    }else if(ismounted == 1){
      umount();
      ismounted = 0;
    }else{
      ismounted = 0;            
    }
  }
  // File name Handling
  DateTime now = RTC.now();
  if(
     (now.year()  != logdate.year() ) || 
     (now.month() != logdate.month()) ||
     (now.day()   != logdate.day()  )
     ){
       logdate = now;
       logfilename[0] = 2 + '0';
       logfilename[1] = 0 + '0';
       logfilename[2] = 1 + '0';
       logfilename[3] = 2 + '0';
       logfilename[5] = 0 + '0';
       logfilename[6] = 4 + '0';
       logfilename[8] = 0 + '0';
       logfilename[9] = 3 + '0';
       if(ismounted == 1){
         umount();
         mount();
       }
  }
  // TODO: file filename != current date umount change filename and remount
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Sampling Handling //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void samplingHandling(){
  if( (millis()-lastSampleTime)>sampleDelay){
    digitalWrite(SAMPLING_LED_PIN,HIGH);
    lastSampleTime = millis(); // Reset the counter
    delay(500); // simulate sampling
    rawdata[0].date = RTC.now();
    rawdata[0].temperature[0] = 12.2;
    rawdata[0].temperature[1] = 22.2;
    rawdata[0].temperature[2] = 32.2;
    rawdata[0].temperature[3] = 42.2;
    rawdata[0].temperature[4] = 52.2;
    rawdata[0].temperature[5] = 62.2;
    rawdata[0].solar = 123.5;
    rawdata[0].pump = 1;
    rawdata[0].burn = 1;
    rawdata[0].oil  = 1;
    newData = 1;
    digitalWrite(SAMPLING_LED_PIN,LOW);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// Write Handling //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void writeHandling(){
  if(newData == 1){
    if(ismounted == 1){
      if(logfile){
        logfile.print("ENTRY");
        DateTime now = RTC.now();
        logfile.print(now.year(),DEC);
        logfile.println(now.unixtime());
        newData=0;
        if((millis()-lastsynctime)>WRITE_INTERVAL){
          digitalWrite(SD_WRITE_DATA_LED_PIN,HIGH);
          lastsynctime = millis();
          logfile.flush();
          digitalWrite(SD_WRITE_DATA_LED_PIN,LOW);
        }
      }else{
        ismounted = 2;
      }
    }
    else{
      // Store data internal
      newData = 0;
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

