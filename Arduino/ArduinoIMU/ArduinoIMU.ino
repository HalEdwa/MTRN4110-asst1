int AccX;
int AccY;
int AccZ;
int GyrX;
int GyrY;
int GyrZ;

//we will simulate the 3 acceleration and 3 gyroscope channels

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {

  if(AccX == 200){
    AccX = 0;
    AccY = 0;
    AccZ = 0;
    GyrX = 0;
    GyrY = 0;
    GyrZ = 0;  
    }
  AccX++;
  AccY++;
  AccZ++;
  GyrX++;
  GyrY++;
  GyrZ++;
  //Send Framing char
  Serial.print('A');
  //Send AccX
  
  if(AccX>0) {
    Serial.print(NULL);
  }
  if(AccX<10) {
    Serial.print(NULL);
  }
  if (AccX<100) {
  Serial.print(NULL);
  }

  Serial.print(AccX);
  //Send AccY
  
  if(AccY>0) {
    Serial.print(NULL);
  }
  if(AccY<10) {
    Serial.print(NULL);
  }
  if (AccY<100) {
  Serial.print(NULL);
  }
  
  Serial.print(AccY);
  //Send AccZ
  
  if(AccZ>0) {
    Serial.print(NULL);
  }
  if(AccZ<10) {
    Serial.print(NULL);
  }
  if (AccZ<100) {
  Serial.print(NULL);
  }
  Serial.print(AccZ);
  //Send GyrX
  if(GyrX>0) {
    Serial.print(NULL);
  }
  if(GyrX<10) {
    Serial.print(NULL);
  }
  if (GyrX<100) {
  Serial.print(NULL);
  }
  Serial.print(GyrX);
//Send GyrY
 if(GyrX>0) {
    Serial.print(NULL);
  }
  if(GyrY<10) {
    Serial.print(NULL);
  }
  if (GyrY<100) {
  Serial.print(NULL);
  }
  Serial.print(GyrY);
  //Send GyrZ
   if(GyrX>0) {
    Serial.print(NULL);
  }
  if(GyrZ<10) {
    Serial.print(NULL);
  }
  if (GyrZ<100) {
  Serial.print(NULL);
  }
  Serial.print(GyrZ);
  
  delay(1000);
  
}
