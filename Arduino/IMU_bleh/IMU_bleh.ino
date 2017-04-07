
/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13.
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead().

 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground

 * Note: because most Arduinos have a built-in LED attached
 to pin 13 on the board, the LED is optional.


 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/AnalogInput

 */

#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <HMC5883L.h>

int HEADER = 18500 ;//0x4844; //header of the stream, 0x4844 in ASCII is "HD"

int data[11]; // buffer to send
unsigned long time;

FreeSixIMU sixDOF = FreeSixIMU(); // Initialise IMU

void setup() {
  
  Serial.begin(9600);
  Wire.begin();

  delay(5);
  sixDOF.init(); // Init Acc and Gyro
  delay(5);
  
  time = millis();
}

void loop() {

  data[0] = HEADER;
  data[1] = analogRead(A0);
  data[2] = analogRead(A1);
  data[3] = analogRead(A2);
  
  sixDOF.getRawValues(&data[4]); // Read raw values from IMU

  data[10] = (int)(millis() - time);
  
  //PrintData(); //Prints values
  
  Serial.write((char*)data,sizeof(data));
  Serial.flush();
  
  time = millis();
  delay(200);
}

void PrintData() {
  Serial.print("Raw values");
  Serial.print(data[4]);
  Serial.print(" ");
  Serial.print(data[5]);
  Serial.print(" ");
  Serial.print(data[6]);
  Serial.print(" ");
  Serial.print(data[7]);
  Serial.print(" ");
  Serial.print(data[8]);
  Serial.print(" ");
  Serial.println(data[9]);
}

