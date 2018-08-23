#include <Arduino.h>
#include <TheThingsNetwork.h>
#include "LowPower.h"

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868

const char *appEui = "70B3D57ED0011A42";
const char *appKey = "382EB32314636116CD910EEC8AF80C5F";
int sleepcycles = 37;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec

void setup();
void loop();
void sendState();

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

const byte sensorPin = 2;
byte windowState = LOW;
byte prevState = LOW;

void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Initialize LED output pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000);

  debugSerial.println("-- STATUS");
  ttn.showStatus();
  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);

  delay(1000);
  pinMode(sensorPin, INPUT_PULLUP);
}

void loop() {
  //debugSerial.println("-- LOOP");
  sendState();
  for (int i=0;i<sleepcycles;i++)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
    windowState = digitalRead(sensorPin);
    if (windowState != prevState) {
      sendState();
      prevState = windowState;
      delay(1000);
    }
  }
}


void sendState() {
  byte data[1];
  data[0] = (windowState == HIGH) ? 1 : 0;
  ttn.sendBytes(data, sizeof(data));
}
