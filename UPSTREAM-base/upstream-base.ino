// This #include statement was automatically added by the Particle IDE.
#include "SI114X.h"

// This #include statement was automatically added by the Particle IDE.
//#include <Adafruit_SI1145.h>

// This #include statement was automatically added by the Particle IDE.
#include <RTClibrary.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_BME680.h>





// This #include statement was automatically added by the Particle IDE.


/************************************
 * µPSTREAM Base v.1 
 * Designed for Particle Xenon Mesh'd to Argon 
 * Maintained by: Je'aime Powell
 * Contact: jpowell@tacc.utexas.edu or jeaimehp@gmail.com
 * Initally Coded: 02/20/20
 ***********************************/

/**********************************
  **** Sensors and Connections *****
  * ********************************
  * SD Datalogger (3.3v & GND) 
  * -- Adalogger FeatherWing 
  *     - SCK  - D13 [SCK]
  *     - MOSI - D12 [MOSI]
  *     - MISO - D11 [MISO]
  *     - SDCS - D5
  * 
  * RTC (3.3v & GND) 
  * -- Adalogger FeatherWing
  *     - SCL -  1 [SCL] *Not Used
  *     - SDA -  0 [SDA] *Not USed
  * 
  * 
  * Temperature/Humidity/Pressure/VOC (3.3v & GND
  * -- BME680 - 0x??
  *     - SCL -  1 [SCL] 
  *     - SDA -  0 [SDA]
  * 
  * TSL2561 0x39
  * 
  * Soil Moisture - A0
  * 
  * NOTE: MQTT will not work from a
  * mesh node, only the hub 
  * (i.e. Argon, Boron)
  * ******************************/


///////////////////////////
// Libraries
///////////////////////////

//Allows to take strings from the json document
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1 
// This #include statement was automatically added by the Particle IDE.
#include <ArduinoJson.h>

// This #include statement was automatically added by the Particle IDE.
//#include <Adafruit_TSL2561_U.h>



// This #include statement was automatically added by the Particle IDE.
#include <SdFat.h>
#include <SPI.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_GPS.h>

// This #include statement was automatically added by the Particle IDE.
#include "MQTT/MQTT.h"

//-------------------------------------------------------//

///////////////////////////
// Sensor and Pin Definitions
///////////////////////////

//Photoresistor
  #define PHOTORESISTOR_PIN 18
  
// Adalogger SDCArd CS 
    // SD chip select pin
    const uint8_t chipSelect = D5;
    
// SD Card File Object
    // file system
    SdFat sd;

    // test file
    SdFile myFile;
    
// GPS
  // what's the name of the hardware serial port?
  #define GPSSerial Serial1

  // Connect to the GPS on the hardware port
  Adafruit_GPS GPS(&GPSSerial);
     
  // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
  // Set to 'true' if you want to debug and listen to the raw GPS sentences
  #define GPSECHO false

// BME680 
  //Adafruit_BME680 bme; // I2C
  #define SEALEVELPRESSURE_HPA (1013.25) // Average
  Adafruit_BME680 bme; // I2C (Grove = 0x76 Adafruit = 0x77)  

// TSL2561 Lux Sensor
  //Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// Soil Moisture
  #define SOIL_MOISTURE A0
  float soilMoistureValue = 0.0;
  
// Sunlight Sensor SI1145 
//  Adafruit_SI1145 SI1145 = Adafruit_SI1145();
SI114X SI1145 = SI114X(); // Grove

  
// RTC
  RTC_PCF8523 rtc;

////////////////////////////////
// Data Collection and Timing
////////////////////////////////

// Variables for Daily Time Sync 
// Ref: https://docs.particle.io/reference/device-os/firmware/xenon/#particle-synctime-
    #define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
    unsigned long lastSync = millis();

// How often to publish to CHORDS 
// Note: Do not set this lower that 10 seconds if using Particle.publish
    int pub_time = 60; //<---- Number of seconds between data collections
    
    
    unsigned long lastmillis_pub = 0;
    // Convert pub_time to milliseconds
    int pub_time_milli = pub_time * 1000;
    
    String mydevID = System.deviceID();
    
    



// MQTT Setup
 void callback(char* topic, byte* payload, unsigned int length) {}
 MQTT client("pt2050dashboard-dev.tacc.utexas.edu", 1883, callback);
  

// Device System Information 
String myID = System.deviceID();



//-------------------------------------------------------//


///////////////////////////
// SETUP
///////////////////////////

void setup() {
  // Entering Setup console message
    Particle.publish("Entering Setup", "OK");

 //Initialize SD Card
    Particle.publish("SDCard Logger", "Initializing");
    delay(1000);
     if (!sd.begin(chipSelect)){
         Particle.publish("SDCard Logger", "Failed");
         //jsonsensor["SDCard"] = "FAILED";
         delay(1000);
         Particle.publish("ALERT", "SDCard_Failure!");
     }
     else {
            Particle.publish("SDCard Logger", "OK");
            //jsonsensor["SDCard"] = "OK";
            delay(1000);
    }
  // GPS Setup
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  /*
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  */
  
  // BME680 
  //if (!bme.begin()) { // Adafruit BME680
  if (!bme.begin(0x76)) { // Grove Seeed bme680 0x76
    Particle.publish("BME680", "Could not find a valid BME680 sensor, check wiring!");
  }
  
  
  //TSL2561 
  /*
  if(!tsl.begin()) {
    Particle.publish("TSL2561","Init Fault");  
  }
  else{
    // Ref: https://github.com/adafruit/Adafruit_TSL2561
    //You can also manually set the gain or enable auto-gain support
    // tsl.setGain(TSL2561_GAIN_1X);      //No gain ... use in bright light to avoid sensor saturation 
    // tsl.setGain(TSL2561_GAIN_16X);     // 16x gain ... use in low light to boost sensitivity 
    tsl.enableAutoRange(true);            // Auto-gain ... switches automatically between 1x and 16x 
  
    //Changing the integration time gives you better sensor resolution (402ms = 16-bit data) 
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution 
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions 
  }
  */

  //Grove Sunlight Sensor SI1145
  if (!SI1145.Begin()) {
  
        Particle.publish ("Si1145" , "Si1145 is not ready!");
    }

  Time.zone(0);

  // RTC on Adalogger
  if (!rtc.initialized()) {
    // following line sets the RTC to the date & time this sketch was compiled
    Particle.syncTime();
    delay(500);
    Particle.publish("RTC", "Syncing RTC Time");
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //Ref: https://codebender.cc/sketch:46065#RTC_SKETCH_FOR_SETING_TIME_COMPILE.ino
    //Ref: https://docs.particle.io/reference/device-os/firmware/photon/#time
    rtc.adjust(DateTime(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second()));
  }

  
  /////////////////////////////
  // Custom Particle Functions
  /////////////////////////////
  //Particle.function("GPS Info",gps_func);
  Particle.function("Status LED", ledfunc);
  Particle.function("Device_Stat", dev_status);
  
  // MQTT 
  // connect to the server
  client.connect("upstream1");
  if (client.isConnected()) { 
  client.publish("/upstream/status","I’m Alive…");
  Particle.publish ("MQTT", "OK");
  }
  else {
    Particle.publish ("MQTT", "Unavailable");  
  }
  
// SD Card Setup
  Particle.publish("Exiting Setup", "OK");

//Set SPI to correct mode to write to the SD Card
    SPI.setDataMode(SPI_MODE0);
    // open the file for write at end like the "Native SD library"
    if (!myFile.open("startup.txt", O_RDWR | O_CREAT | O_AT_END)) {
        //sd.errorHalt("opening startup.log for write failed");
        Particle.publish("SDCard_Setup", "SD Card startup.txt init failed");
        Particle.publish("ALERT", "SD Card startup.txt init failed");
    }
    else {
        // if the file opened okay, write to it:
        
        time_t time = Time.now();
        String timeStamp = Time.format(time, TIME_FORMAT_ISO8601_FULL);
        myFile.println("////////////////SD Card initialized///////////////////");
        myFile.printf(timeStamp,"\n");
        myFile.printf("fileSize: %d\n", myFile.fileSize());
        myFile.printf("Device ID ",mydevID,"\n");
        myFile.printf("[--------------------------------------------------]\n");
  
         // close the file:
        myFile.close();
        }  
        
        
 //Sitrep
 // Send Sitrep
    delay(500);
    dev_status("phone home");
    

 //Turn off Status LED 
  RGB.control(true);
   // Red
    RGB.color(255, 0, 0);
    delay(500);
    RGB.color(0, 0, 0);
    delay(100);
    RGB.color(255, 0, 0);
    delay(500);
   // Yellow
    RGB.color(255, 255, 0);
    delay(500);
    RGB.color(0, 0, 0);
    delay(100);
    RGB.color(255, 255, 0);
    delay(500);
    // Green
    RGB.color(0, 255, 0);
    delay(1000);
  // OFF
  RGB.color(0, 0, 0);




}

int ledfunc (String onoff){
    int ledstate;
    if (onoff == "on"){
        Particle.publish("LED-status", "On");
        RGB.control(false);
        ledstate = 1;
    }
    else{
        RGB.control(true);
        Particle.publish("LED-status", "Use: on");
        ledstate = 0;
    }
    return ledstate;
}


int gps_func (String blah){
    Particle.publish("GPS Status", "Reading...");
    char c = GPSSerial.read();
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) {
       Particle.publish("GPS Status", "Not found");
       return -1; 
       }
    }
   Particle.publish("GPS Fix", String(GPS.fix));
   Particle.publish("GPS Lat", String(GPS.lat));
   Particle.publish("GPS Long", String(GPS.lon));
   //Particle.publish("GPS all", String(c));
    
    return 0;

}


///////////////////////////
// Loop
///////////////////////////



void loop() {
    // Daily Time Sync
    if (millis() - lastSync > ONE_DAY_MILLIS) {
        // Request time synchronization from the Particle Device Cloud
        Particle.syncTime();
        lastSync = millis();
        Particle.publish("Syncing Time", "OK");
        dev_status("phone home");
    }
    
    char c = GPSSerial.read();
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) 
       //Particle.publish("GPS Status", "Not found");
       return; 
       }
    
    String gps_fix = String(GPS.fix);
    Particle.variable ("GPS",gps_fix);
    String gps_loc = String (GPS.lat + GPS.lon);
    Particle.variable ("GPS Location",gps_loc);
    
    //JSON Data Object Setup
    // set a Json array. Creates a Json object named jsondata
    const size_t capacity = JSON_ARRAY_SIZE(8) + JSON_OBJECT_SIZE(16) + 30;
    DynamicJsonDocument jsondata(capacity);
    
    // Timestamp
        // Time.zone(-6);  // Central Time
        Time.zone(0);  // UTC - Needed for Chords
        time_t time = Time.now();
        String timeStamp = Time.format(time, TIME_FORMAT_ISO8601_FULL);
    
    // Device Information
    jsondata["timestamp"] = timeStamp;
    jsondata["deviceID"] = myID;
    
    // BME680
    
    jsondata["temperature"] = bme.temperature;
    //float tempmqtt = bme.temperature;
    jsondata["relativeHumidity"] =  bme.humidity;
    jsondata["pressureHpa"] =  bme.pressure / 1000.0;
    jsondata["gasResistanceKOhms"] = bme.gas_resistance / 1000.0;
    jsondata["approxAltitudeInM"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    
    // Lux
    /*
    sensors_event_t event;
    tsl.getEvent(&event);
    jsondata["lux"] = event.light;
    */
    
    //Sunlight Sensor SI1145
    jsondata["visibleLight"] = SI1145.ReadVisible();
    jsondata["IRLight"] = SI1145.ReadIR();
    jsondata["UVLight"] = SI1145.ReadUV();
    float UVindex = SI1145.ReadUV();
    // the index is multiplied by 100 so to get the
    // integer index, divide by 100!
    UVindex /= 100.0;  
    jsondata["UVindex"] = UVindex;
    
    //Analog Average Reads
    int soilsumRead = 0;
    //int photosumRead = 0;
    int trial_reads = 5;
    for (int i = 0; i < trial_reads; i++ ){
        soilsumRead = soilsumRead + analogRead(SOIL_MOISTURE);
        //photosumRead = photosumRead + analogRead(PHOTORESISTOR_PIN);
    }
    
    //Photoresistor
    // jsondata["photoresistor"] = photosumRead / (float)trial_reads;
    
    //Soil Moisture
    soilMoistureValue = soilsumRead / (float)trial_reads;
    jsondata["SoilValue"] = soilMoistureValue;
    
    // Particle Publish Data Set
     if (millis() - lastmillis_pub >= pub_time_milli) {
        String data;
        serializeJson(jsondata, data);
        Particle.publish("data", data);
        lastmillis_pub = millis();
        
        // SD Card Log data
        //Set SPI to correct mode to write to the SD Card
        SPI.setDataMode(SPI_MODE0);
        String data_file_name = mydevID + Time.format(Time.now(), "%F") + ".log";
        if (!myFile.open(data_file_name, O_RDWR | O_CREAT | O_AT_END)) {
            Particle.publish("SDCard_Write", "SD Card data_file_name failed");
            Particle.publish("ALERT", "SD Card data_file_name init failed");
        }
        else{
            myFile.println(data);
            myFile.close();
        }
       
     }
}
     
int dev_status(String blah) {
    delay(100);
    //Argon
    /*
    byte mac[6];
    WiFi.macAddress(mac);
    String stringmac;
    for (byte i = 0; i < 6; ++i)
        {
        char buf[3];
        sprintf(buf, "%2X", mac[i]);
        stringmac += buf;
        if (i < 5) stringmac += ':';
    }
    */
    
    
    int uptime = System.uptime();
    // Create JSON Object
    const size_t capacity = JSON_ARRAY_SIZE(8) + JSON_OBJECT_SIZE(16) + 30;
    DynamicJsonDocument jsonstatus(capacity);
    
    
    // Create Wifi Object
    //WiFiSignal sig = WiFi.RSSI();
    //float strength = sig.getStrength();
    
    jsonstatus["Device_ID"] = mydevID;
    jsonstatus["Device_Ver"] = System.version();
    jsonstatus["Device_Ut"] = uptime;
    
    //Boron
     // Boron Cell signal strength
    CellularSignal sig = Cellular.RSSI();
    jsonstatus["Cell_Sig"] = int(sig.getStrength());
    jsonstatus["Battery_gauge"] = System.batteryCharge();

    //Argon
    /*
    jsonstatus["Device_IP"] = String(WiFi.localIP());
    jsonstatus["Device_MAC"] = stringmac;
    jsonstatus["Wifi_SSID"] = WiFi.SSID();
    jsonstatus["Wifi_Sig"] = strength;
    */
    
    //Create JSON String
    String sitrep;
    serializeJson(jsonstatus, sitrep);
    Particle.publish("SitRep",sitrep);
    return 1;
}
 
