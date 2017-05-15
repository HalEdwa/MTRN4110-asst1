#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

int HEADER = 18500 ;//0x4844; //header of the stream, 0x4844 in ASCII is "HD"

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

int IMU_Data[8]; // 0 header 1-3 accelerometer, 4-6 gyroscope 7 dt
float IMU_Buffer[6];
unsigned long time;

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
  IMU_Data[0] = HEADER;
  //sixDOF.getValues1000(&IMU_Data[1]);
  sixDOF.getRawValues(&IMU_Data[1]);
  
  IMU_Data[7] = (int)(millis() - time);
  
  Serial.write((char*)IMU_Data, sizeof(IMU_Data));
  Serial.flush();
  time = millis();
  //printRawValues();

  delay(5);
}

void printRawValues() {
  Serial.print("Header: ");
  Serial.print(IMU_Data[0]);
  Serial.print("  ");
  Serial.print("Accelerometer: ");
  Serial.print(IMU_Data[1]);
  Serial.print("  ");
  Serial.print(IMU_Data[2]);
  Serial.print("  ");
  Serial.print(IMU_Data[3]);
  Serial.print("    ");
  Serial.print("Gyroscope: ");
  Serial.print(IMU_Data[4]);
  Serial.print("  ");  
  Serial.print(IMU_Data[5]);
  Serial.print("  ");
  Serial.println(IMU_Data[6]);
  Serial.print("  ");
  Serial.print("Dt: ");
  Serial.print(IMU_Data[7]);
}

