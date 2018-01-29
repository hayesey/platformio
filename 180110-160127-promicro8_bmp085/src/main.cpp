#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
int RXLED = 17;  // The RX LED has a defined Arduino pin
// The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)

void setup()
{
 pinMode(RXLED, OUTPUT);  // Set RX LED as an output
 // TX LED is set as an output behind the scenes

 Serial.begin(9600); //This pipes to the serial monitor
 Serial1.begin(9600); //This is the UART, pipes to sensors attached to board
 if (!bmp.begin()) {
   Serial.println("Could not find a valid BMP085 sensor, check wiring!");
   while (1) {}
 }
}

void loop()
{
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

 digitalWrite(RXLED, LOW);   // set the LED on
 TXLED0; //TX LED is not tied to a normally controlled pin
 delay(1000);              // wait for a second
 digitalWrite(RXLED, HIGH);    // set the LED off
 TXLED1;
 delay(10000);              // wait for a second
}
