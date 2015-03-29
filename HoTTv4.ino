#include "HoTTv4.h"

#define HOTTV4_RXTX 3
#define HOTTV4_TX_DELAY 1000
#define OFFSET_ALTITUDE 500
#define OFFSET_M2S 72
#define OFFSET_M3S 120

static uint8_t outBuffer[173];

/*static int32_t m1s = 0;
static int32_t m3s = 0;
static int32_t m10s = 0;
*/

int16_t altitude = 0;

SoftwareSerial hottV4Serial(HOTTV4_RXTX , HOTTV4_RXTX);

/**
 * Common setup method for HoTTv4
 */
void hottV4Setup() {
  hottV4Serial.begin(19200);
  hottV4EnableReceiverMode();
}

/**
 * Enables RX and disables TX
 */
static inline void hottV4EnableReceiverMode() {
  DDRD &= ~(1 << HOTTV4_RXTX);
  PORTD |= (1 << HOTTV4_RXTX);
}

/**
 * Enabels TX and disables RX
 */
static inline void hottV4EnableTransmitterMode() {
  DDRD |= (1 << HOTTV4_RXTX);
}

/**
 * Writes out given byte to HoTT serial interface.
 */
static void hottV4SerialWrite(uint8_t c) {
  hottV4Serial.write(c);
}

  static void hottV4GPSUpdate() {
    //number of Satelites
    HoTTV4GPSModule.GPSNumSat=MultiHoTTModule.GPS_numSat;
    if (MultiHoTTModule.GPS_fix > 0) {
      /** GPS fix */
      HoTTV4GPSModule.GPS_fix = 0x33; // Dgps: '0x44' 2D = '0x32' 3D = '0x33' nofix = '0x2d'
      
      uint16_t lat_D=0;
      uint16_t lat_M=0;      
      uint16_t lat_S=0;       
      uint16_t lat_SS=0;      
      uint8_t lat_home_D=0;   
      uint16_t lat_home_M=0;  
      uint16_t lat_home_S=0;  
      uint16_t lat_home_SS=0; 
      uint16_t lat_hott_M=0;  
      uint32_t lat_hott_S=0;  

      uint8_t lon_D=0;
      uint16_t lon_M=0;
      uint16_t lon_S=0;
      uint16_t lon_SS=0;
      uint8_t lon_home_D=0;
      uint16_t lon_home_M=0;
      uint16_t lon_home_S=0;
      uint16_t lon_home_SS=0;
      uint16_t lon_hott_M=0;
      uint32_t lon_hott_S=0; 
	  
      //latitude
      lat_D =      (int) MultiHoTTModule.GPS_latitude;                                            
      lat_M =      (int) ((MultiHoTTModule.GPS_latitude-lat_D)*60);                               
      lat_S  =     (int) ((((MultiHoTTModule.GPS_latitude-lat_D)*60)-lat_M)*60);                  
      lat_SS  =    (int) ((((((MultiHoTTModule.GPS_latitude-lat_D)*60)-lat_M)*60)- lat_S)*10000); 
      lat_hott_M = (int) (lat_D*100)+ lat_M;                             
      lat_hott_S = ((lat * 100000) - (lat_D * 100000)) * 6;              
      lat_hott_S = lat_hott_S % 10000;                                   
        
      HoTTV4GPSModule.LatitudeNS=(MultiHoTTModule.GPS_latitude<0);  
      HoTTV4GPSModule.LatitudeMinLow = lat_hott_M;
      HoTTV4GPSModule.LatitudeMinHigh = lat_hott_M >> 8;
      HoTTV4GPSModule.LatitudeSecLow = lat_hott_S;
      HoTTV4GPSModule.LatitudeSecHigh = lat_hott_S >> 8;
	  
	  
      //longitude
      lon_D =      (int) MultiHoTTModule.GPS_longitude;                                            
      lon_M =      (int) ((MultiHoTTModule.GPS_longitude-lon_D)*60);                               
      lon_S  =     (int) ((((MultiHoTTModule.GPS_longitude-lon_D)*60)-lon_M)*60);                  
      lon_SS  =    (int) (((((MultiHoTTModule.GPS_longitude-lon_D)*60)-lon_M)*60-lon_S)*10000);    
      lon_hott_M = (int) (lon_D*100) + lon_M;                            
      lon_hott_S = ((lon * 100000) - (lon_D * 100000)) * 6;              
      lon_hott_S = lon_hott_S % 10000;                                   
      
      HoTTV4GPSModule.longitudeEW=(MultiHoTTModule.GPS_longitude<0);
      HoTTV4GPSModule.longitudeMinLow = lon_hott_M;
      HoTTV4GPSModule.longitudeMinHigh = lon_hott_M >> 8;
      HoTTV4GPSModule.longitudeSecLow = lon_hott_S;
      HoTTV4GPSModule.longitudeSecHigh = lon_hott_S >> 8;
	  
	  
      /** GPS Speed in km/h */
      uint16_t speed = MultiHoTTModule.GPS_speed;  // in km/h
      HoTTV4GPSModule.GPSSpeedLow = speed & 0x00FF;
      HoTTV4GPSModule.GPSSpeedHigh = speed >> 8;
      /** Distance to home */      
      HoTTV4GPSModule.distanceLow = MultiHoTTModule.GPS_distanceToHome & 0x00FF;
      HoTTV4GPSModule.distanceHigh = MultiHoTTModule.GPS_distanceToHome >> 8;
      /** Altitude */
      HoTTV4GPSModule.altitudeLow = MultiHoTTModule.GPS_altitude & 0x00FF;
      HoTTV4GPSModule.altitudeHigh = MultiHoTTModule.GPS_altitude >> 8;
      /** Home Direction */
      HoTTV4GPSModule.HomeDirection = MultiHoTTModule.GPS_directionToHome;
      /**Flightdirection */
      HoTTV4GPSModule.flightDirection = MultiHoTTModule.GPS_flightDirection;
      
      //VARIO  not implemented yet, should be a BMP085
      //m/s
      HoTTV4GPSModule.resolutionLow = MultiHoTTModule.GPS_distanceToHome & 0x00FF;
      HoTTV4GPSModule.resolutionHigh = MultiHoTTModule.GPS_distanceToHome >> 8;
      //m/3s
      HoTTV4GPSModule.unknow1 = MultiHoTTModule.GPS_flightDirection;
      
      HoTTV4GPSModule.alarmTone = MultiHoTTModule.GPS_alarmTone;
      
    } else {
      HoTTV4GPSModule.GPS_fix = 0x2d; // Displays a ' ' to show nothing or clear the old value
    }
  }

/**
 * Sends HoTTv4 capable GPS telemetry frame.
 */
  static void hottV4SendGPS() {
    /** Minimum data set for EAM */    
    HoTTV4GPSModule.startByte = 0x7C;
    HoTTV4GPSModule.sensorID = HOTTV4_GPS_SENSOR_ID;
    HoTTV4GPSModule.sensorTextID = HOTTV4_GPS_SENSOR_TEXT_ID;
    HoTTV4GPSModule.endByte = 0x7D;
    /** ### */
    
    hottV4GPSUpdate();

    if (is_set_home == 0)
    {
      HoTTV4GPSModule.alarmTone = 0x08; //alarm tone if no fix
	  toggle_LED(); 					//Let the led blink
    }else
    {
      HoTTV4GPSModule.alarmTone = 0x0;
    }

    // Clear output buffer
    memset(&outBuffer, 0, sizeof(outBuffer));

    // Copy GPS data to output buffer
    memcpy(&outBuffer, &HoTTV4GPSModule, kHoTTv4BinaryPacketSize);

    // Send data from output buffer
    hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
  }

  /**
 * Updates m1s, m3s, and m10s inclination for VARIO.

static void updateVarioInclination() {
  static uint32_t now_1 = 0;
  static uint32_t now_3 = 0;
  static uint32_t now_10 = 0;

  static int32_t reference_m1s = 0;
  static int32_t reference_m3s = 0;  
  static int32_t reference_m10s = 0;

  static int16_t m1s = 0;
  static int16_t m3s = 0;
  static int16_t m10s = 0;

  uint32_t now = seconds();

  if ((0 == reference_m1s) && (0 != MultiHoTTModule.altitude)) {
    reference_m1s = MultiHoTTModule.altitude;
    reference_m3s = MultiHoTTModule.altitude;    
    reference_m10s = MultiHoTTModule.altitude;
  } else {
    if ((now - now_1) >= 1) {
      m1s = (int16_t)(MultiHoTTModule.altitude - reference_m1s);
      reference_m1s = MultiHoTTModule.altitude;
      now_1 = now;      
    }

    if ((now - now_3) >= 3) {
      m3s = (int16_t)(MultiHoTTModule.altitude - reference_m3s);
      reference_m3s = MultiHoTTModule.altitude;
      now_3 = now;      
    }
    
    if ((now - now_10) >= 10) {
      m10s = (int16_t)(MultiHoTTModule.altitude - reference_m10s);
      reference_m10s = MultiHoTTModule.altitude;
      now_10 = now;      
    }
  }

#if defined DEBUG
  Serial.print("m1/s: ");
  Serial.println(m1s);
  Serial.print("m3/s: ");
  Serial.println(m3s);
  Serial.print("m10/s: ");
  Serial.println(m10s);
#endif
  
  if (MultiHoTTModuleSettings.varioBeep) {
    if (m1s >= 250) {
      HoTTV4VarioModule.alarmTone = 'D';
    } else if (m1s >= 100) {
      HoTTV4VarioModule.alarmTone = 'E';
    } else if (m1s <= -250) {
      HoTTV4VarioModule.alarmTone = 'N';
    } else if (m1s <= -100) {
      HoTTV4VarioModule.alarmTone = 'R';
    }
  }

  HoTTV4VarioModule.m1s = 30000 + m1s;
  HoTTV4VarioModule.m3s = 30000 + m3s;
  HoTTV4VarioModule.m10s = 30000 + m10s;
}
*/

/**
 * Seconds since start
 */
static uint32_t seconds() {
  return millis() / 1000;
}

/**
 * Updates height over ground, max. height over ground, and min. height over ground for VARIO.
static void updateVarioAltitude() {
  static int16_t maxAltitude = OFFSET_ALTITUDE;
  static int16_t minAltitude = OFFSET_ALTITUDE;
  
  static int32_t referenceRawAltitude = 0;

  if ((0 == referenceRawAltitude) && (0 != MultiHoTTModule.altitude)) {
    referenceRawAltitude = MultiHoTTModule.altitude;    
  } else {
    altitude = (int16_t)((MultiHoTTModule.altitude - referenceRawAltitude) / 100);
    maxAltitude = max(maxAltitude, OFFSET_ALTITUDE + altitude);
    minAltitude = min(minAltitude, OFFSET_ALTITUDE + altitude);
    
    if (altitude >= (100 * MultiHoTTModuleSettings.maxAltitude)) {
      HoTTV4VarioModule.alarmTone = HoTTv4NotificationMaxAltitude;  
      HoTTV4VarioModule.alarmInverse |= 0x2; // Invert max altitude 
    }
  }
  HoTTV4VarioModule.altitude = OFFSET_ALTITUDE + altitude; 
  HoTTV4VarioModule.maxAltitude = maxAltitude;
  HoTTV4VarioModule.minAltitude = minAltitude;  
}
 */ 
 
/**
 Sends HoTTv4 capable VARIO telemetry frame.
static void hottV4SendVARIO() {
   Minimum data set for Vario 
  HoTTV4VarioModule.startByte = 0x7C;
  HoTTV4VarioModule.sensorID = HOTTV4_VARIO_SENSOR_ID;
  HoTTV4VarioModule.sensorTextID = HOTTV4_VARIO_SENSOR_TEXT_ID;
  HoTTV4VarioModule.endByte = 0x7D;

  
  updateVarioAltitude();
  updateVarioInclination();

  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4VarioModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}*/


 /* Expects an array of at least size bytes. All bytes till size will be transmitted
 * to the HoTT capable receiver. Last byte will always be treated as checksum and is
 * calculated on the fly.
 */
static void hottV4SendData(uint8_t *data, uint8_t size) {
  hottV4Serial.flush();

  // Protocoll specific waiting time
  // to avoid collisions
  delay(5);

  if (hottV4Serial.available() == 0) {
    hottV4EnableTransmitterMode();

    uint16_t crc = 0;

    for (uint8_t i = 0; i < (size - 1); i++) {
      crc += data[i];
      hottV4SerialWrite(data[i]);

      // Protocoll specific delay between each transmitted byte
      delayMicroseconds(HOTTV4_TX_DELAY);
    }

    // Write package checksum
    hottV4SerialWrite(crc & 0xFF);

    hottV4EnableReceiverMode();
  }
}

/**
 * Entry point to send HoTTv4 capable data frames according to the
 * requested module.
 */
void hottV4SendTelemetry() {
  static enum _hottV4_state {
    IDLE,
    BINARY,
    TEXT,
  } hottV4_state = IDLE;

  if (hottV4Serial.available() > 1) {
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t c = hottV4Serial.read();

      if (IDLE == hottV4_state) {
        switch (c) {
          case 0x80:
            hottV4_state = BINARY;
            break;
          case 0x7F:
            hottV4_state = TEXT;
            break;
          default:
            hottV4_state = IDLE;
        }
      } else if (BINARY == hottV4_state) {
        switch (c) {
            
	    case HOTTV4_GPS_SENSOR_ID:
            hottV4SendGPS();
            hottV4_state = IDLE;
        break;
		
	    case HOTTV4_VARIO_SENSOR_ID:
            //hottV4SendVARIO();
            //hottV4_state = IDLE;
        break;
            
           default:
            hottV4_state = IDLE;
        }
      } 
    }
  }
}
