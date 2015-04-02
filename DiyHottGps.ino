/**
 * DIY-HOTT-GPS is a standalone Arduino based Application that acts 
 * as a HoTTv4 capable device to transmit GPS/Vario information. 
 * Code contains parts by Oliver Bayer, Carsten Giesen, Jochen Boenig, Mikal Hart, ZiegeOne and Stefan Muerzl 03/2015
 * tested by Robert aka Skyfighter THANKS!
 * 
 * Before compiling check you Baudrate in void setup()!
 * Vor dem Compilieren bitte in der void setup() die Baudrate an euer GPS anpassen!
 */

#include "SoftwareSerial.h"
#include <avr/io.h> 
#include "TinyGPS.h"

//#define Vario

TinyGPS gps; 

float f_HOME_LAT = 0, f_HOME_LON = 0;
float start_height = 0;

bool is_set_home = 0;
uint32_t last = 0;
int p_alt[4]={0,0,0,0};

//Variables for GPS-Functions
unsigned long speed_k=0; // Knots * 100
long lat=0, lon=0, lati=0, loni=0;
float flat=0, flon=0;
unsigned long age=0, dat=0, tim=0, ui_course=0;
int alt=0;
unsigned int numsat=0; 
float alt_offset = 500;
bool newdata = false;
uint32_t now = millis(); 

uint16_t GPS_distanceToHome;

struct {
  
  //uint8_t  GPS_fix;
  //uint8_t  GPS_numSat;
  float GPS_latitude;
  float GPS_longitude;
  //uint16_t GPS_altitude;
  //uint16_t GPS_speed;
  //uint16_t GPS_Vario1;
  //uint8_t  GPS_Vario3;
  //uint8_t  GPS_flightDirection;
  
  //uint8_t  GPS_directionToHome;
  //uint8_t  GPS_alarmTone;
  int32_t altitude;
  
} MultiHoTTModule;

#define LED 13 

void setup() {
  
  pinMode(LED, OUTPUT);
  Serial.begin(9600);

  #ifdef Vario
	setupAltitude();
  #endif
  
  hottV4Setup();
  is_set_home = 0;
  p_alt[0]=0;
  p_alt[1]=0;
  p_alt[2]=0;
  p_alt[3]=0;  
}

void loop() {

   smartdelay(200);
	
   gps.get_position(&lat, &lon, &age);
   gps.get_position2(&lati, &loni, &age);
   gps.f_get_position(&flat, &flon);
   
   #ifdef Vario
	  alt = readAltitude();
   #else
	  alt =  gps.altitude()/100;
          
   #endif
   
   
   now = millis();
   
   if ((now - last) > 1000) //measure every second for Vario function
   {  
     last = now;
     p_alt[3] = p_alt[2];
     p_alt[2] = p_alt[1];
     p_alt[1] = p_alt[0];
     p_alt[0] = alt;     
   }
   
   sethome();
   hottV4SendTelemetry();
}

void toggle_LED()
{
 digitalWrite(LED, !digitalRead(LED)); 
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

void sethome()
{
   if (is_set_home == 0 && gps.satellites() >= 6)  // we need more than 6 sats
   {
       
       f_HOME_LAT = flat;
       f_HOME_LON = flon;
       start_height = alt;	   //in future height will be set with BMP085
       p_alt[0]=alt;
       p_alt[1]=alt;
       p_alt[2]=alt;
       p_alt[3]=alt; 

	   if ((gps.altitude()/100) != 9999999)	
	   {
		is_set_home = 1;
	   }	
   }  
}
