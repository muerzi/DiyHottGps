#include <TinyGPS.h>
#include <avr\pgmspace.h>

/* This sample code demonstrates the basic use of a TinyGPS object.
   Typically, you would feed it characters from a serial GPS device, but 
   this example uses static strings for simplicity.
*/
prog_char str1[] PROGMEM = "$GPRMC,201547.000,A,3014.5527,N,09749.5808,W,0.24,163.05,040109,,*1A";
prog_char str2[] PROGMEM = "$GPGGA,201548.000,3014.5529,S,09749.5808,E,1,07,1.5,225.6,M,-22.5,M,18.8,0000*77";
prog_char str3[] PROGMEM = "$GPRMC,201548.000,A,3014.5529,N,17749.5808,W,0.17,53.25,040109,,*24";
prog_char str4[] PROGMEM = "$GPGGA,201549.000,3014.5533,S,17749.5812,E,1,07,1.5,223.5,M,-22.5,M,18.8,0000*7C";
prog_char *teststrs[4] = {str1, str2, str3, str4};

static void sendstring(TinyGPS &gps, const PROGMEM char *str);
static void gpsdump(TinyGPS &gps);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);

void setup()
{
  TinyGPS test_gps;
  Serial.begin(115200);
  
  Serial.println("Sats HDOP Latitude Longitude  Latitude Longitude");
  Serial.println("          (dd.dddddd)           (DDMM.mmmm)     ");
  Serial.println("------------------------------------------------");
  gpsdump(test_gps);
  for (int i=0; i<4; ++i)
  {
    sendstring(test_gps, teststrs[i]);
    gpsdump(test_gps);
  }
}

void loop()
{
}

static void sendstring(TinyGPS &gps, const PROGMEM char *str)
{
  while (true)
  {
    char c = pgm_read_byte_near(str++);
    if (!c) break;
    gps.encode(c);
  }
  gps.encode('\r');
  gps.encode('\n');
}

static void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  long lati, loni;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);
  gps.get_position2(&lati, &loni, &age);
  
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5);
  //Serial.print(lati);
  print_int(lati, TinyGPS::GPS_INVALID_F_ANGLE, 10);
  print_int(loni, TinyGPS::GPS_INVALID_F_ANGLE, 11);

  Serial.println();
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    Serial.print(sz);
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(" ");
  }
}

