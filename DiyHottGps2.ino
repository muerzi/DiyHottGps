/**
 * DIY-HOTT-GPS is a standalone Arduino based Application that acts 
 * as a HoTTv4 capable device to transmit GPS/Vario information. 
 * Code contains parts by Oliver Bayer, Carsten Giesen, Jochen Boenig and Stefan Muerzl 11/2013
 * tested by Robert aka Skyfighter THANKS!
 */

#include "SoftwareSerial.h"
#include <avr/io.h> 
#include <TinyGPS++.h>
#include <avr/wdt.h> 

#define LED 13
//#define Vario
//#define open360Tracker

TinyGPSPlus gps; 

TinyGPSCustom exactLat(gps, "GPRMC", 3);
TinyGPSCustom exactLon(gps, "GPRMC", 5);

double HOME_LAT = 0, HOME_LON = 0;
float start_height = 0;
bool is_set_home = 0;
uint32_t last = 0;
int p_alt[4]={0,0,0,0};

//Variables for GPS-Functions
int16_t LatDegMin, LatDecMin, LonDegMin, LonDecMin;
bool newdata = false;
uint32_t now = millis(); 
  
struct {
  
  uint8_t  GPS_fix;
  uint8_t  GPS_numSat;
  double GPS_latitude_HOME;
  double GPS_longitude_HOME;
  //double GPS_latitude;
  //double GPS_longitude;
  uint16_t GPS_LatDegMin;
  uint16_t GPS_LatDecMin;
  uint16_t GPS_LonDegMin;
  uint16_t GPS_LonDecMin;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_Vario1;
  uint8_t  GPS_Vario3;
  uint8_t  GPS_flightDirection;
  uint16_t GPS_distanceToHome;
  uint8_t  GPS_directionToHome;
  uint8_t  GPS_alarmTone;
  int16_t altitude;
  
} MultiHoTTModule;

void setup() {

  wdt_enable(WDTO_2S);
  pinMode(LED, OUTPUT);
  Serial.begin(38400);
  hottV4Setup();
  
  #ifdef Vario
	setupAltitude();
  #endif
     
  MultiHoTTModule.GPS_fix       = 0;  
  
}

void loop() {

   smartdelay(200);
   wdt_reset();
   
   #ifdef Vario
	  MultiHoTTModule.GPS_altitude = readAltitude();
   #else
	  MultiHoTTModule.GPS_altitude =  gps.altitude.meters();
   #endif
   
   updateVario(); 

   //set homeposition
   if (is_set_home == 0)  // we need more than 6 sats
   {
       if (gps.satellites.value() >=6 && gps.altitude.isValid() && gps.location.isValid())
	   {
		   MultiHoTTModule.GPS_latitude_HOME = gps.location.lat();
		   MultiHoTTModule.GPS_longitude_HOME = gps.location.lng();
		   //start_height = alt;	   //in future height will be set with BMP085
		   is_set_home = 1;
		   MultiHoTTModule.GPS_fix       = 1;
	   }
   }  
   
       
   
   sscanf(exactLat.value(), "%4d.%4d", MultiHoTTModule.GPS_LatDegMin, MultiHoTTModule.GPS_LatDecMin);
   sscanf(exactLon.value(), "%5d.%4d", MultiHoTTModule.GPS_LonDegMin, MultiHoTTModule.GPS_LonDecMin);
   
   MultiHoTTModule.GPS_numSat = gps.satellites.value();  

   MultiHoTTModule.GPS_speed     = MultiHoTTModule.GPS_speed; 
   
   MultiHoTTModule.GPS_altitude = MultiHoTTModule.GPS_altitude+500;//-start_height;  // from GPS in cm, +500m Offset for Hott   
   
   MultiHoTTModule.GPS_distanceToHome = gps.distanceBetween(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON); //calculation of distance to home
   
   MultiHoTTModule.GPS_directionToHome = gps.courseTo(MultiHoTTModule.GPS_latitude_HOME, MultiHoTTModule.GPS_longitude_HOME, MultiHoTTModule.GPS_LatDegMin, MultiHoTTModule.GPS_LonDegMin); //calculation of bearing from home to plane
   
   MultiHoTTModule.GPS_flightDirection = ((int)gps.course.deg())/2;   //flightcourse of the plane
   
   hottV4SendTelemetry();// send data 

}

void smartdelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}

void updateVario(){
   if ((now - last) > 1000) //measure every second for Vario function
   {  
     last = now;
     p_alt[3] = p_alt[2];
     p_alt[2] = p_alt[1];
     p_alt[1] = p_alt[0];
     p_alt[0] = MultiHoTTModule.GPS_altitude+500;     
   }
}

void toggle_LED(){
 digitalWrite(LED, !digitalRead(LED)); 
}

