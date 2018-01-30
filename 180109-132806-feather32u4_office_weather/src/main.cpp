#define BSFRANCEBOARD 1
#define LOWPOWER 1

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
// https://github.com/thesolarnomad/lora-serialization
#include <LoraMessage.h>  // Use the encoder to format and transmit data.
#include <Adafruit_SleepyDog.h>

#define LEDPIN 13
#if BSFRANCEBOARD
#define VBATPIN A9
#endif

/*************************************
 * TODO: Change the following keys
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 *************************************/
static u1_t NWKSKEY[16] = { 0xB2, 0xF0, 0xBB, 0x6A, 0xF4, 0x36, 0x4C, 0x2F, 0x0B, 0x3E, 0x0C, 0x1D, 0xC1, 0x87, 0x6D, 0x27 };
static u1_t APPSKEY[16] = { 0xDE, 0xE9, 0x83, 0x1A, 0x31, 0x3F, 0x16, 0x62, 0x41, 0x0E, 0x78, 0xDE, 0x87, 0x68, 0x64, 0x85 };
static u4_t DEVADDR = 0x26011816;   // Put here the device id in hexadecimal form.
Adafruit_BMP085 bmp;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 600;

// Pin mapping
// These settings are for the user made board with the Atmega32u4 and the Wemos RFM95 shield.
/*
const lmic_pinmap lmic_pins = {
    .nss = 9,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {8, 8, LMIC_UNUSED_PIN} // Since the Wemos Lora Shield merges the pin with diodes, just use the same pin number };
*/

// These settings are for the BSFrance LORA32U4 board. Remember: pin 6 must be connected to DIO6 through a jumper wire!
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6 , LMIC_UNUSED_PIN}
};

// The BSFrance Lora32U4 board has a voltage divider from the LiPo connector that allows
// to measure battery voltage.
#if BSFRANCEBOARD
float getBatVoltage() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // Voltage is devided by two by bridge resistor so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  return measuredvbat;
}
#endif

void print_pressure() {
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  float pressure = (float)bmp.readPressure() / 100;
  Serial.print(pressure);
  Serial.println(" hPa");

  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  float seaLevelPressure = (float)bmp.readSealevelPressure() / 100;
  Serial.print(seaLevelPressure);
  Serial.println(" hPa");
  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
 Serial.print("Real altitude = ");
 Serial.print(bmp.readAltitude(101500));
 Serial.println(" meters");

 Serial.println();
}


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
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
        Serial.println(F("Sending uplink packet..."));
        // Lit up the led to signal transmission . Should not be used if aiming to save power...
        digitalWrite(LEDPIN,HIGH);
        //print_pressure();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        digitalWrite(LEDPIN,LOW);
        //radio.sleep();
        //Watchdog.sleep(600);
        //os_setCallback(&sendjob, do_send);
    }
}

void setup() {
    //while (!Serial);	// Wait for the serial port to wake up, otherwise Linux has trouble to connect.
    pinMode(LEDPIN,OUTPUT);
    for ( int i = 0 ; i < 3 ; i++) {
      digitalWrite(LEDPIN,HIGH);
      delay(500);
      digitalWrite(LEDPIN,LOW);
      delay(500);
    }

    Serial.begin(115200);
    Serial.println(F("Starting..."));

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Set static session parameters.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(DR_SF7,14);
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while (1) {}
    }
    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
