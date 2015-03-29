/**
 * DIY-HOTT-GPS is a standalone Arduino based Application that acts 
 * as a HoTTv4 capable device to transmit GPS/Vario information. 
 * Code contains parts by Oliver Bayer, Carsten Giesen, Jochen Boenig, Mikal Hart, ZiegeOne and Stefan Muerzl 03/2015
 * tested by Robert aka Skyfighter THANKS!
 * 
 * Before compiling check you Baudrate in void setup()!
 * Vor dem Compilieren bitte in der void setup() die Baudrate an euer GPS anpassen!
 *
 * This Project needs TinyGPS v13 from http://arduiniana.org/libraries/tinygps/ otherwise the sensor will not work correct!
 * Download here: https://github.com/mikalhart/TinyGPS/archive/v13.zip
 *
 * Diese Projekt benötigt TinyGPS v13 von http://arduiniana.org/libraries/tinygps/ ansonsten funktioniert der Sensor nicht!
 * Download hier: https://github.com/mikalhart/TinyGPS/archive/v13.zip
 *
 * Version 1
 *
 */

#include "SoftwareSerial.h"
#include <avr/io.h> 
#include <TinyGPS.h> 

//#define Vario

TinyGPS gps; 

float HOME_LAT = 0, HOME_LON = 0;
float start_height = 0;

bool is_set_home = 0;
uint32_t last = 0;
int p_alt[4]={0,0,0,0};

//Variables for GPS-Functions
unsigned long speed_k; // Knots * 100
long lat, lon;
float flat, flon, alt;
unsigned long age, dat, tim, ui_course;
//uint16_t alt;
unsigned int numsat; 
bool newdata = false;
uint32_t now = millis(); 
  
struct {
  
  uint8_t  GPS_fix;
  uint8_t  GPS_numSat;
  float GPS_latitude;
  float GPS_longitude;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_Vario1;
  uint8_t  GPS_Vario3;
  uint8_t  GPS_flightDirection;
  uint16_t GPS_distanceToHome;
  uint8_t  GPS_directionToHome;
  uint8_t  GPS_alarmTone;
  int32_t altitude;
  
} MultiHoTTModule;

#define LED 13 

void setup() {
  
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  is_set_home = 0;

  #ifdef Vario
	setupAltitude();
  #endif
  
  hottV4Setup();
  
}

void toggle_LED(){
 digitalWrite(LED, !digitalRead(LED)); 
}

void loop() {

   smartdelay(200);
	
   gps.get_position(&lat, &lon, &age);
   gps.f_get_position(&flat, &flon);
   gps.get_datetime(&dat, &tim, &age);
   #ifdef Vario
	  alt = readAltitude();
   #else
	  alt =  gps.altitude()/100;
   #endif
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
   if (is_set_home == 0 && numsat >= 6)  // we need more than 6 sats
   {
       
       HOME_LAT = flat;
       HOME_LON = flon;
       //start_height = alt;	   //in future height will be set with BMP085

	   if ((gps.altitude()/100) != 9999999)	
	   {
		is_set_home = 1;
	   }	
   }  
   
   MultiHoTTModule.GPS_fix       = 1;       
   MultiHoTTModule.GPS_numSat    = numsat;  //Satellites in view
   MultiHoTTModule.GPS_latitude  = flat;     //Geograph. Latitude
   MultiHoTTModule.GPS_longitude = flon;     //Geograph. Longitude
   MultiHoTTModule.GPS_speed     = (speed_k * 1852) / 100000; // from GPS in Knots*100 -> km/h
   MultiHoTTModule.GPS_altitude = alt+500;//-start_height;  // from GPS in cm, +500m Offset for Hott   
   MultiHoTTModule.GPS_distanceToHome = gps.distance_between(flat, flon, HOME_LAT, HOME_LON); //calculation of distance to home
   MultiHoTTModule.GPS_directionToHome = gps.course_to(HOME_LAT, HOME_LON, lat, lon) / 2; //calculation of bearing from home to plane
   MultiHoTTModule.GPS_flightDirection = ui_course/2;   //flightcourse of the plane
   // send data
   hottV4SendTelemetry();
 

   #ifdef Vario
	readAltitude();
  #endif
 

}

void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}
