#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Arduino.h>

int sleepcycles = 75;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
#define LedPin 13     // pin 13 LED is not used, because it is connected to the SPI port
#define VBATPIN A9
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t DEVEUI[8]  = { 0x0A, 0x2A, 0x86, 0xCB, 0x8D, 0x0E, 0x46, 0x00 };
static const u1_t APPEUI[8] = { 0x7A, 0x9A, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = { 0xDA, 0xEF, 0xE2, 0x77, 0xEF, 0x58, 0x0F, 0x3F, 0x9D, 0x77, 0x24, 0xCC, 0xE1, 0xB4, 0x4F, 0x92 };
Adafruit_BMP085 bmp;

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
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  return measuredvbat;
}

void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin,LOW);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
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
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    float vbat = getBatVoltage();
    int vbatint = (int)(vbat*100);
    Serial.print("vbatint: " ); Serial.println(vbatint);
    uint8_t message[12];
    message[0] = 0x01;
    message[1] = 0x02;
    message[2] = (uint8_t)highByte(vbatint);
    message[3] = (uint8_t)lowByte(vbatint);
    int pressure = (int)(bmp.readPressure() / 10);

    message[4] = 0x02;
    message[5] = 0x73;
    message[6] = (uint8_t)highByte(pressure);
    message[7] = (uint8_t)lowByte(pressure);

    int temperature = (int)((bmp.readTemperature() * 100)/10);

    message[8] = 0x03;
    message[9] = 0x67;
    message[10] = (uint8_t)highByte(temperature);
    message[11] = (uint8_t)lowByte(temperature);

    LMIC_setTxData2(1, message, sizeof(message), 0);
    Serial.println(F("Sending: "));
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
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  delay(10000);
  Serial.begin(9600);
  Serial.println(F("Starting"));
  delay(10000);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  os_setCallback(&initjob, initfunc);
  LMIC_reset();
}


void loop()
{
    // start OTAA JOIN
    if (joined==false)
    {

      os_runloop_once();

    }
    else
    {
      do_send(&sendjob);    // Sent sensor values
      while(sleeping == false)
      {
        os_runloop_once();
      }
      sleeping = false;
      for (int i=0;i<sleepcycles;i++)
      {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
      }
    }

      digitalWrite(LedPin,((millis()/100) % 2) && (joined==false)); // only blinking when joining and not sleeping

}
