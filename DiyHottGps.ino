/**
 * DIY-HOTT-GPS is a standalone Arduino based Application that acts 
 * as a HoTTv4 capable device to transmit GPS/Vario information. 
 * Code contains party by Oliver Bayer, Carsten Giesen, Jochen Boenig and Stefan Muerzl 04/2013
 */

#include "SoftwareSerial.h"
#include <avr/io.h> 
#include <TinyGPS.h> 
TinyGPS gps; 

float HOME_LAT = 0, HOME_LON = 0;
float start_height = 0;

bool feedgps();  
bool is_set_home = 0;
uint32_t last = 0;
int p_alt[4]={0,0,0,0};

struct {
  
  uint8_t  GPS_fix;
  uint8_t  GPS_numSat;
  uint32_t GPS_latitude;
  uint32_t GPS_longitude;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_Vario1;
  uint8_t  GPS_Vario3;
  uint8_t  GPS_flightDirection;
  uint16_t GPS_distanceToHome;
  uint8_t  GPS_directionToHome;
  uint8_t  GPS_alarmTone;
  
} MultiHoTTModule;

#define LED 13

void setup() {
  
  pinMode(LED, OUTPUT);
  delay(200);

  Serial.begin(57600);
  delay(200);
  //Serial.println("$PMTK300,250,0,0,0,0*2A");  //   init GPS
  delay(100);
  
  is_set_home = 0;

  hottV4Setup();
  
}

bool feedgps()
{
  while (Serial.available())
  {
    if (gps.encode(Serial.read()))
      return true;
  }
  return false;
}

void toggle_LED(){
 digitalWrite(LED, !digitalRead(LED)); 
}

void loop() {

  //Variables for GPS-Functions
  unsigned long speed_k; // Knots * 100
  long lat, lon;
  float flat, flon, alt;
  unsigned long age, dat, tim, ui_course;
  //uint16_t alt;
  unsigned int numsat; 
  bool newdata = false;
  uint32_t now = millis();  
  
 if (feedgps()) newdata = true;
 
 if ( newdata == true)
 {

   gps.get_position(&lat, &lon, &age);
   gps.f_get_position(&flat, &flon);
   gps.get_datetime(&dat, &tim, &age);
   //alt = (int) gps.f_altitude();
   alt =  gps.altitude()/100;
   numsat = gps.satellites(); 
   ui_course = gps.course()/100;
   speed_k = gps.speed(); 
   
   if ((now - last) > 1000) //measure every second for Vario function
   {  
     last = now;
     p_alt[3] = p_alt[2];
     p_alt[2] = p_alt[1];
     p_alt[1] = p_alt[0];
     p_alt[0] = alt+500;     
   }
   
   //set homeposition
   if (is_set_home == 0 && numsat >= 5)  // we need more than 5 sats
   {
       
       HOME_LAT = flat;
       HOME_LON = flon;
       start_height = alt;	   //in future height will be set with BMP085

	   if ((gps.altitude()/100) != 9999999)	
	   {
		is_set_home = 1;
	   }	
   }  
   
   MultiHoTTModule.GPS_fix       = 1;       
   MultiHoTTModule.GPS_numSat    = numsat;  //Satellites in view
   MultiHoTTModule.GPS_latitude  = lat;     //Geograph. Latitude
   MultiHoTTModule.GPS_longitude = lon;     //Geograph. Longitude
   MultiHoTTModule.GPS_speed     = (speed_k * 1852) / 100000; // from GPS in Knots*100 -> km/h
   MultiHoTTModule.GPS_altitude = alt+500-start_height;  // from GPS in cm, +500m Offset for Hott   
   MultiHoTTModule.GPS_distanceToHome = gps.distance_between(flat, flon, HOME_LAT, HOME_LON); //calculation of distance to home
   MultiHoTTModule.GPS_directionToHome = gps.course_to(HOME_LAT, HOME_LON, lat, lon) / 2; //calculation of bearing from home to plane
   MultiHoTTModule.GPS_flightDirection = ui_course/2;   //flightcourse of the plane
   
 }

  // send data
  hottV4SendTelemetry();
}
