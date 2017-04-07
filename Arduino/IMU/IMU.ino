#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

int IMU_Data[6]; // 0-2 accelerometer, 3-5 gyroscope
float TimeStamp = 0;

void setup() {
  Serial.begin(9600);
  
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();
  delay(5);
  sixDOF.init(); //init the Acc and Gyro
  delay(5);
}

void loop() {
  sixDOF.getRawValues(IMU_Data);
  
  //printRawValues();
  sendRawValues();
}

void sendRawValues() {  
  TimeStamp = millis()/1000.0;
  
  //Send Framing char
  Serial.print('A');

  //Send gyroscope and accelerator values with padding to keep buffer length fixed
  for (int i = 0; i < 6; i++) {
    if(IMU_Data[i]>=0) Serial.print(NULL);
    if(IMU_Data[i]<10 && IMU_Data[i]>-10) Serial.print(NULL);
    if (IMU_Data[i]<100 && IMU_Data[i]>-100) Serial.print(NULL);
    if (IMU_Data[i]<1000 && IMU_Data[i]>-1000) Serial.print(NULL);
    if (IMU_Data[i]<10000 && IMU_Data[i]>-10000) Serial.print(NULL);
    Serial.print(IMU_Data[i]);
  }

  //Send current system time with padding to keep buffer length fixed
  if(TimeStamp<10) Serial.print(NULL);
  if (TimeStamp<100) Serial.print(NULL);
  if (TimeStamp<1000) Serial.print(NULL);
  if (TimeStamp<10000) Serial.print(NULL);
  if (TimeStamp<100000) Serial.print(NULL);
  Serial.print(TimeStamp);
  delay(2);
  //Serial.print('\n');
}

void printRawValues() {
  Serial.print("Accelerometer: ");
  Serial.print(IMU_Data[0]);
  Serial.print("  ");
  Serial.print(IMU_Data[1]);
  Serial.print("  ");
  Serial.print(IMU_Data[2]);
  Serial.print("    ");
  Serial.print("Gyroscope: ");
  Serial.print(IMU_Data[3]);
  Serial.print("  ");  
  Serial.print(IMU_Data[4]);
  Serial.print("  ");
  Serial.println(IMU_Data[5]);
}

