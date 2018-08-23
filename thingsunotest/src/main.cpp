#include <Arduino.h>
#include <TheThingsNetwork.h>

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868

const char *appEui = "70B3D57ED0011A42";
const char *appKey = "382EB32314636116CD910EEC8AF80C5F";

void message(const byte* payload, int length, int port);
void sendState();
void stateChange();

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

const byte interruptPin = 2;
volatile byte windowState = LOW;

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

  // Set callback for incoming messages
  ttn.onMessage(message);
  delay(1000);
  pinMode(interruptPin, INPUT_PULLUP);
  //windowState = (digitalRead(interruptPin) == HIGH) ? 1 : 0;
  //debugSerial.print("windowState: ");
  //debugSerial.println(windowState);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), stateChange, CHANGE);
}

void loop() {
  //debugSerial.println("-- LOOP");
  windowState = digitalRead(interruptPin);
  debugSerial.print("State: ");
  debugSerial.println(windowState);
  sendState();
  delay(300000);
}

void message(const byte* payload, int length, int port) {
  debugSerial.println("-- MESSAGE");

  // Only handle messages of a single byte
  if (length != 1) {
    return;
  }

  if (payload[0] == 0) {
    debugSerial.println("LED: off");
    digitalWrite(LED_BUILTIN, LOW);

  } else if (payload[0] == 1) {
    debugSerial.println("LED: on");
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void sendState() {
  // Prepare array of 1 byte to indicate LED status
  byte data[1];
  //data[0] = (windowSta te == HIGH) ? 1 : 0;
  //data[0] = windowState;
  data[0] = (windowState == HIGH) ? 1 : 0;
  //debugSerial.print("Data to send: ");
  //for (int i = 0; i < sizeof(data); i++) debugSerial.print(data[i], HEX);
  //debugSerial.println();
  // Send it off
  ttn.sendBytes(data, sizeof(data));
}

void stateChange() {
  windowState = digitalRead(interruptPin);
  //windowState = !windowState;
  //debugSerial.println("State changed");
  //debugSerial.println(windowState);
  //sendState();
}
