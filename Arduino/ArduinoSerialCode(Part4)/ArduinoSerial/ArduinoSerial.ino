bool Digital2=0;
int analog1 = 0;
int analog2 = 1;
int analog3 = 2;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(2,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Digital2 = digitalRead(2);
  analog1= analogRead(A0)/4; //bring between 0 and 255
  delay(10);
  analog2= analogRead(A1)/4;
  delay(10);
  analog3= analogRead(A2)/4;
  delay(10);
  Serial.print("A");
  Serial.print(analog1); 
  Serial.print("B");
  Serial.print(analog2); 
  Serial.print("C");
  Serial.print(analog3); 
  delay(500);
  
}
