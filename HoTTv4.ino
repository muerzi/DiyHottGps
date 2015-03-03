#include "HoTTv4.h"

#define HOTTV4_RXTX 3
#define HOTTV4_TX_DELAY 1000

static uint8_t outBuffer[173];

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
      //latitude
      HoTTV4GPSModule.LatitudeNS=(MultiHoTTModule.GPS_latitude<0);
      uint8_t deg = MultiHoTTModule.GPS_latitude / 100000;
      uint32_t sec = (MultiHoTTModule.GPS_latitude - (deg * 100000)) * 6;
      uint8_t min = sec / 10000;
      sec = sec % 10000;
      uint16_t degMin = (deg * 100) + min;
      HoTTV4GPSModule.LatitudeMinLow = degMin;
      HoTTV4GPSModule.LatitudeMinHigh = degMin >> 8;
      HoTTV4GPSModule.LatitudeSecLow = sec;
      HoTTV4GPSModule.LatitudeSecHigh = sec >> 8;
      //latitude
      HoTTV4GPSModule.longitudeEW=(MultiHoTTModule.GPS_longitude<0);
      deg = MultiHoTTModule.GPS_longitude / 100000;
      sec = (MultiHoTTModule.GPS_longitude - (deg * 100000)) * 6;
      min = sec / 10000;
      sec = sec % 10000;
      degMin = (deg * 100) + min;
      HoTTV4GPSModule.longitudeMinLow = degMin;
      HoTTV4GPSModule.longitudeMinHigh = degMin >> 8;
      HoTTV4GPSModule.longitudeSecLow = sec;
      HoTTV4GPSModule.longitudeSecHigh = sec >> 8;
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
            
           default:
            hottV4_state = IDLE;
        }
      } 
    }
  }
}
