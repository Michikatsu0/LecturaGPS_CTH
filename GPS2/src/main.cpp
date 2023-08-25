#include <Arduino.h>
#include <SoftwareSerial.h>
//objeto       (pinRX,pinTX) en el cerebro
SoftwareSerial gpsserial(2,0);

void setup() {
  // put your setup code here, to run once:
  gpsserial.begin(9600);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (gpsserial.available()>0)
  {
    Serial.write(gpsserial.read());
  }
 
}