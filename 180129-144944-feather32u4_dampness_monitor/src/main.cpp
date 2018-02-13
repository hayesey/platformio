#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <Wire.h>
#include <Arduino.h>
#include <TinyGPS.h>

int sleepcycles = 75;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
uint8_t data[4];
#define LedPin 13     // pin 13 LED is not used, because it is connected to the SPI port
#define VBATPIN A9
#define HihAddr 0x27
#define DEVID 3
//#define DEVID 2
//#define DEVID 3
//#define DEVID 4

// deveui & appeui need to be LSB
// appkey msb
#if DEVID==1
  // dampmon1
  static const u1_t DEVEUI[8]  = { 0x0A, 0x2A, 0x86, 0xCB, 0x8D, 0x0E, 0x46, 0x00 };
  static const u1_t APPKEY[16] = { 0xDA, 0xEF, 0xE2, 0x77, 0xEF, 0x58, 0x0F, 0x3F, 0x9D, 0x77, 0x24, 0xCC, 0xE1, 0xB4, 0x4F, 0x92 };
  // 01 is main unit with gps
  static const uint8_t DEVTYPE = 0x01;
#elif DEVID==2
  // dampmon2
  static const u1_t DEVEUI[8]  = { 0x93, 0x0F, 0x45, 0x20, 0xBD, 0x9E, 0xBD, 0x00 };
  static const u1_t APPKEY[16] = { 0xEB, 0x07, 0xC7, 0xE2, 0x84, 0x9E, 0x36, 0x2D, 0xC4, 0x9E, 0x7A, 0xCD, 0xA5, 0x84, 0x65, 0x9C };
  // 02 is device without gps
  static const uint8_t DEVTYPE = 0x02;
#elif DEVID==3
  // dampmon3
  static const u1_t DEVEUI[8]  = { 0x6B, 0x4E, 0xA0, 0x1C, 0x3B, 0x8D, 0xE7, 0x00 };
  static const u1_t APPKEY[16] = { 0x96, 0x09, 0x94, 0x89, 0x97, 0x27, 0xF8, 0x6F, 0x56, 0xD3, 0xCE, 0x9B, 0xCB, 0x63, 0x7C, 0x2F };
  // 02 is device without gps
  static const uint8_t DEVTYPE = 0x02;
#elif DEVID==4
  // dampmon4
  static const u1_t DEVEUI[8]  = { 0x16, 0xB8, 0xAC, 0x75, 0x5D, 0x5D, 0xC3, 0x00 };
  static const u1_t APPKEY[16] = { 0xC0, 0x7D, 0x60, 0xD4, 0x47, 0x10, 0x8B, 0x65, 0x9A, 0x67, 0xC8, 0x97, 0x0F, 0xBE, 0xED, 0xCE };
  // 02 is device without gps
  static const uint8_t DEVTYPE = 0x02;
#endif

// this is always the same for this application
static const u1_t APPEUI[8] = { 0x7A, 0x9A, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

static void initfunc (osjob_t*);

// provide APPEUI (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

// provide DEVEUI (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// provide APPKEY key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
}

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4, // Needed on RFM92/RFM95? (probably not) D0/GPIO16
  .dio = {7, 6, LMIC_UNUSED_PIN}, // Specify pin numbers for DIO0, 1, 2
// connected to D7, D6, -
};

float getBatVoltage() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // Voltage is devided by two by bridge resistor so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  //Serial.print("VBat: " ); Serial.println(measuredvbat);
  return measuredvbat;
}

float getHumidity() {
  float humidity = ((((data[0] & 0x3F) * 256) + data[1]) * 100.0) / 16383.0;
  return humidity;
}

float getTemperature() {
  int temp = ((data[2] * 256) + (data[3] & 0xFC)) / 4;
  float cTemp = (temp / 16384.0) * 165.0 - 40.0;
  return cTemp;
}

void requestData() {

  // Start I2C Transmission
  Wire.beginTransmission(HihAddr);
  // Select data register
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 4 bytes of data
  Wire.requestFrom(HihAddr, 4);

  // Read 4 bytes of data
  // humidity msb, humidity lsb, temp msb, temp lsb
  if (Wire.available() == 4)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
  }
}

void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      //Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      //Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      //Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      //Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      //Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin,LOW);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      //Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      //Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      //Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        //Serial.print(F("Data Received: "));
        //Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i>10){
          i=10;     // maximum number of BLINKs
        }
          for(j=0;j<i;j++)
          {
            digitalWrite(LedPin,HIGH);
            delay(200);
            digitalWrite(LedPin,LOW);
            delay(400);
          }
      }
      //Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      //Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      //Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      //Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      //Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      //Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      //Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    //Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    float vbat = getBatVoltage();
    int vbatint = (int)(vbat*100);
    requestData();
    delay(125);
    int humidity = round(getHumidity() * 100);
    int temperature = round(getTemperature() * 100);

    if (DEVTYPE == 0x02) {
      uint8_t message[7];
      message[0] = DEVTYPE;
      message[1] = (uint8_t)highByte(vbatint);
      message[2] = (uint8_t)lowByte(vbatint);
      message[3] = (uint8_t)highByte(humidity);
      message[4] = (uint8_t)lowByte(humidity);
      message[5] = (uint8_t)highByte(temperature);
      message[6] = (uint8_t)lowByte(temperature);
      LMIC_setTxData2(1, message, sizeof(message), 0);
    } else if (DEVTYPE == 0x01) {
      TinyGPS gps;
      bool newData = false;
      float flat, flon;
      for (unsigned long start = millis(); millis() - start < 1000;)
      {
        while (Serial1.available())
        {
          char c = Serial1.read();
          if (gps.encode(c)) // Did a new valid sentence come in?
            newData = true;
        }
      }
      if (newData)
      {
        unsigned long age;
        gps.f_get_position(&flat, &flon, &age);
        flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
        flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
      }
      uint32_t lat = flat * 1000000;
      uint32_t lon = flon * 1000000;
      uint8_t message[15];
      message[0] = DEVTYPE;
      message[1] = (uint8_t)highByte(vbatint);
      message[2] = (uint8_t)lowByte(vbatint);
      message[3] = (uint8_t)highByte(humidity);
      message[4] = (uint8_t)lowByte(humidity);
      message[5] = (uint8_t)highByte(temperature);
      message[6] = (uint8_t)lowByte(temperature);
      message[7] = lat >> 24;
      message[8] = lat >> 16;
      message[9] = lat >> 8;
      message[10] = lat;
      message[11] = lon >> 24;
      message[12] = lon >> 16;
      message[13] = lon >> 8;
      message[14] = lon;
      LMIC_setTxData2(1, message, sizeof(message), 0);
    }
    //Serial.println(F("Sending: "));
  }
}

// initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}

void setup()
  {
  delay(10000);
  Wire.begin();
  //Serial.begin(9600);
  //Serial.println(F("Starting"));
  Serial1.begin(9600);
  delay(1000);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  os_setCallback(&initjob, initfunc);
  LMIC_reset();
}

void loop() {
    // start OTAA JOIN
    if (joined==false) {
        os_runloop_once();
    }
    else {
      do_send(&sendjob);    // Sent sensor values
      while(sleeping == false)
      {
        os_runloop_once();
      }
      sleeping = false;
      //delay(120000);
      for (int i=0;i<sleepcycles;i++)
      {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
      }
    }

      digitalWrite(LedPin,((millis()/100) % 2) && (joined==false)); // only blinking when joining and not sleeping

}
