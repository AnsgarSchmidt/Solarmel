#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>

////////////////// Hardware Settings ///////////////////////////////////////////////////////////////////////
// Com
#define COMRX_PIN                  0 // Serial Receive PREDEFINED
#define COMTX_PIN                  1 // Serial Transmit PREDEFINED

// Buttons
#define D2_PIN                     2 //
#define SAMPLE_NOW_PIN             3 // Button to initiate imidiately sampling

// Temperature
#define TEMP_DATA_PIN              4 // DataPin for tempsensors

// LEDs
#define TEMP_SENSOR_ERROR_LED_PIN  5 // Error with Temp Sensor
#define SAMPLING_LED_PIN           6 // Sampling in progress.         Green LED onboard
#define D7_PIN                     7 //
#define SD_WRITE_ERROR_LED_PIN     8 // Indicates an error writing to SD card
#define SD_WRITE_DATA_LED_PIN      9 // Indicates SD access

// SD
#define SD_DATA_PIN               10 // Adafruit SD shield
#define MOSI_PIN                  11 // MOSI for SD shield
#define MISO_PIN                  12 // MISO for SD shield
#define CLK                       13 // CLK  for SD shield

// Analog
#define PUMP_WATER_PIN            A0 // Solar water pump
#define BURNER_PIN                A1 // Burner
#define SOLAR_CELL_PIN            A2 // Solar cell voltag sample pin
#define A3_PIN                    A3 // 
#define RTCSDA_PIN                A4 // RTC I2C Interface. Hardwired
#define RTCSCL_PIN                A5 // RTC I2C Interface. Hardwired

// Temperature Sensors
DeviceAddress TEMP_OUTDOOR         = {0x28, 0xB9, 0x08, 0xC3, 0x03, 0x00, 0x00, 0xE4};
DeviceAddress TEMP_TOWARD_FLOR     = {0x28, 0x7B, 0x36, 0xC3, 0x03, 0x00, 0x00, 0x33};
DeviceAddress TEMP_RETURN_FLOW     = {0x28, 0x0D, 0xF9, 0xC2, 0x03, 0x00, 0x00, 0xDB};
DeviceAddress TEMP_BOILER_TOP      = {0x28, 0x8B, 0x1D, 0xC3, 0x03, 0x00, 0x00, 0xC9};
DeviceAddress TEMP_BOILER_MIDDLE   = {0x28, 0xF5, 0x06, 0xC3, 0x03, 0x00, 0x00, 0x4E};
DeviceAddress TEMP_BOILER_BOTTOM   = {0x28, 0xD9, 0x0A, 0xC3, 0x03, 0x00, 0x00, 0xA4};

////////////////////// Constants //////////////////////////////////////////////////////////////////////////////
#define DEBOUNCE_DELAY            50 // debounceDelay for Button in millis
#define MOUNT_DELAY            10000 // delay between mount and unmount in millis
#define WRITE_INTERVAL     5*60*1000 // delay for flushing data to SD

#define MAX_RAW_DATA               2 // Amount of Raw data stored localy
////////////////////////// Vars ///////////////////////////////////////////////////////////////////////////////
// Data
uint16_t          sampleDelay             = 10000;          // Time between samples, initial 10 secs. Will be chanced depending on data
uint32_t          lastSampleTime          = 0L;             // Timestamp of last sampling
uint8_t           newData                 = false;
// Filehandling
File              logfile;                                  // Logfile itself
char              logfilename[]           = "01-01-00.csv"; // CurrentFileNameForLogging MM-DD-VV.csv VV=Logfile type version number
DateTime          logdate;                                  // Object to store the logfilename as datetime
// Temperature
OneWire           oneWire(TEMP_DATA_PIN);
DallasTemperature sensors(&oneWire);
// RTC
RTC_DS1307        RTC;
// SD
Sd2Card   card;
SdVolume  volume;
SdFile    root;
const int chipSelect = 10; // ?????????????????
// RAW Data
struct Data{
  DateTime date;
  float    temperature[6];
  uint16_t solar;
  uint8_t  pump;
  uint8_t  burn;
  uint8_t  isStored;
};
Data rawdata[MAX_RAW_DATA];

/////////////////////////////////////////// Setup routine /////////////////////////////////////////////////////
void setup(){
  // Serial Communication
  Serial.begin(9600); // Slow to make sure the connection is stable
  
  Serial.println("Serial communication established now setting up");

  //Pins input
  pinMode     (SAMPLE_NOW_PIN,            INPUT );
  pinMode     (SOLAR_CELL_PIN,            INPUT );

  //Enable Pull up resistors
  digitalWrite(SAMPLE_NOW_PIN,            HIGH  );

  //Output Pins
  pinMode     (SAMPLING_LED_PIN,          OUTPUT);
  pinMode     (SD_WRITE_ERROR_LED_PIN,    OUTPUT);
  pinMode     (SD_DATA_PIN,               OUTPUT);
  pinMode     (TEMP_DATA_PIN,             OUTPUT);
  pinMode     (TEMP_SENSOR_ERROR_LED_PIN, OUTPUT);

  // Initial
  digitalWrite(SAMPLING_LED_PIN,          LOW   );
  digitalWrite(SD_WRITE_ERROR_LED_PIN,    LOW   );
  digitalWrite(TEMP_SENSOR_ERROR_LED_PIN, LOW   );

  // Timers
  lastSampleTime        = millis();

  // RTC
  Wire.begin();
  RTC.begin();
  if (! RTC.isrunning()) {
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
    
  // Temperature 
  sensors.begin();
  sensors.setResolution(TEMP_12_BIT);  // Global
  if(checkSensor(TEMP_OUTDOOR      )){sensors.setResolution(TEMP_OUTDOOR      ,TEMP_12_BIT);}
  if(checkSensor(TEMP_TOWARD_FLOR  )){sensors.setResolution(TEMP_TOWARD_FLOR  ,TEMP_12_BIT);}
  if(checkSensor(TEMP_RETURN_FLOW  )){sensors.setResolution(TEMP_RETURN_FLOW  ,TEMP_12_BIT);}
  if(checkSensor(TEMP_BOILER_TOP   )){sensors.setResolution(TEMP_BOILER_TOP   ,TEMP_12_BIT);}
  if(checkSensor(TEMP_BOILER_MIDDLE)){sensors.setResolution(TEMP_BOILER_MIDDLE,TEMP_12_BIT);}
  if(checkSensor(TEMP_BOILER_BOTTOM)){sensors.setResolution(TEMP_BOILER_BOTTOM,TEMP_12_BIT);}

  // Reset Data storage, I know this is paranoia
  for(uint8_t i=0;i<MAX_RAW_DATA;i++){
    rawdata[i].date             = 0;
    for(uint8_t t=0;t<6;t++){
      rawdata[i].temperature[t] = 0.0;
    }
    rawdata[i].solar            = 0.0;
    rawdata[i].pump             = false;
    rawdata[i].burn             = false;
    rawdata[i].isStored         = true; // No need to write empty data on SD card
  }
  
  // SD
  if (SD.begin(SD_DATA_PIN)){
    Serial.println("SD mount successfull");
  }else{
    Serial.println("SD mount error");
  }
  logdate = DateTime(RTC.now().unixtime() - (100000L)); // Set past date to force mountHandling to generate new Filename
  filenameHandling();                                   // Calculate new filename

  Serial.println("Setup ready starting mainloop");
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
//      printSensorAddress(insensor);
//      Serial.print(" Temperature:");
//      Serial.println(temp);
    }
    return temp;
  }else{
    return DEVICE_DISCONNECTED;
  }
}

void printSensorAddress(DeviceAddress insensor){
  for(uint8_t i=0;i<8;i++){
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
  // Sample now
  if(digitalRead(SAMPLE_NOW_PIN) == LOW){ //Button inverted for pull up resistor
    Serial.println("SAMPLE NOW");
    lastSampleTime = 0; // Reset Sampling Time in order to force a sample;
  }  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Filenamehandling Handling /////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filenameHandling(){
  DateTime now = RTC.now();
  if((now.year()  != logdate.year()  ) || 
     (now.month() != logdate.month() ) ||
     (now.day()   != logdate.day()   )){
    logdate = now;
    // day
    logfilename[4] =               now.day()  %10      + '0';
    logfilename[3] = (now.day()  -(now.day()  %10))/10 + '0';
    // month    
    logfilename[1] =               now.month()%10      + '0';
    logfilename[0] = (now.month()-(now.month()%10))/10 + '0';
    Serial.print("Setting new Filename:");
    Serial.println(logfilename);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Sampling Handling //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void samplingHandling(){
  if( (millis() - lastSampleTime) > sampleDelay){
    digitalWrite(SAMPLING_LED_PIN,HIGH);
    Serial.println("Start Sampling");
    lastSampleTime = millis(); // Reset the counter
    sensors.requestTemperatures(); //Request for all Sensors
    Serial.println("Sampling requested now reading");
    // Write data always into 0 record, write Handling takes care of shifting data
    rawdata[0].date           = RTC.now();
    rawdata[0].temperature[0] = getTemperature(TEMP_OUTDOOR      );
    rawdata[0].temperature[1] = getTemperature(TEMP_TOWARD_FLOR  );
    rawdata[0].temperature[2] = getTemperature(TEMP_RETURN_FLOW  );
    rawdata[0].temperature[3] = getTemperature(TEMP_BOILER_TOP   );
    rawdata[0].temperature[4] = getTemperature(TEMP_BOILER_MIDDLE);
    rawdata[0].temperature[5] = getTemperature(TEMP_BOILER_BOTTOM);
    rawdata[0].solar          = analogRead    (SOLAR_CELL_PIN    );
    rawdata[0].pump           = false;  //dummy
    rawdata[0].burn           = false;  //dummy
    rawdata[0].isStored       = false; //new unstored data, will be handeled by writeHandling
    newData = true; // semaphore for writeHandling
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
    logfile = SD.open(logfilename, FILE_WRITE);
    if(logfile){
      Serial.println("Writing data to disk");
      // Check for all entries if they are stored alreay backward
      for(int16_t i=(MAX_RAW_DATA-1); i>=0; i--){
          Serial.print("In Loop ");
          Serial.println(i);
          if(rawdata[i].isStored == false){
            Serial.println("Storing");
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
            // Temp            
            for(uint8_t t=0; t<6; t++){
              logfile.print(",");
              logfile.print(rawdata[i].temperature[t],DEC);
            }
            logfile.print(",");
            // Solar
            Serial.println(rawdata[i].solar);
            logfile.print(rawdata[i].solar,DEC);
            logfile.print(",");
            // Pump
            if(rawdata[i].pump == 1){logfile.print(1,DEC);}else{logfile.print(0,DEC);}
            logfile.print(",");
            // Burner
            if(rawdata[i].burn == 1){logfile.print(1,DEC);}else{logfile.print(0,DEC);}
            logfile.println();
            // Mark as stored on SD
            rawdata[i].isStored = 1;
          }
      }//for
      logfile.close();
      Serial.println("logfile closed");
    }else{
      Serial.print("can not open file:");
      Serial.println(logfilename);
    }
    // Shift Data
    for(uint8_t i=MAX_RAW_DATA-1; i>0; i--){
      rawdata[i] = rawdata[i-1]; 
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Sampling Rate Handling /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculateSamplingRate(){
  if(rawdata[0].burn == true){
    sampleDelay = 1000; // Every Second
    return;
  }
  if(rawdata[0].pump == true){
    sampleDelay = 2000; // On Solarpower every 2 seconds
    return;
  }

  // TODO: More impressing cool math here
  if(true){
   sampleDelay=1000;
   return;
  }

  // Default
  sampleDelay = 10*1000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Main Loop //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  buttonHandling();
  filenameHandling();
  samplingHandling();
  writeHandling();
  calculateSamplingRate();
}

