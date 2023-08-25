#include <Arduino.h>
#include <TinyGPS.h>
#include "ClosedCube_HDC1080.h"
#include <Wire.h>
#include <SoftwareSerial.h>

enum class StatesMachine { SMART_DELAY, CAL_TEMP, CAL_HUM, PRUN_TEMP, PRUN_HUM, SEND_REQ };
StatesMachine StateMachine = StatesMachine::SMART_DELAY;

ClosedCube_HDC1080 Sensor;
SoftwareSerial gpsserial(2,0);
TinyGPS gps;

bool newData = false;
int iterations;
double dataTemp, promTemp;
double dataHum, promHum;
static void smartdelay(unsigned long ms);

void setup() {
  gpsserial.begin(9600);
  Sensor.begin(0x40);
  Serial.begin(115200);
  iterations = 5;
}

void loop()
{
  if (newData)
  {
    switch (StateMachine)
    {
      case StatesMachine::SMART_DELAY:
      {
        smartdelay(100);
        StateMachine = StatesMachine::CAL_TEMP;
        break;
      }
      case StatesMachine::CAL_TEMP:
      {
        for (int i = 0; i < iterations; i++)
        {
          dataTemp += Sensor.readTemperature();
          smartdelay(5);
        }
        StateMachine = StatesMachine::CAL_HUM;
        break;
      }
      case StatesMachine::CAL_HUM:
      {
        for (int i = 0; i < iterations; i++)
        {
          dataHum += Sensor.readTemperature();
          smartdelay(5);
        }
        StateMachine = StatesMachine::PRUN_TEMP;
        break;
      }
      case StatesMachine::PRUN_TEMP:
      {
        promTemp = dataTemp / iterations;
        StateMachine = StatesMachine::PRUN_HUM;
        break;
      }
      case StatesMachine::PRUN_HUM:
      {
        promHum = dataHum / iterations;
        StateMachine = StatesMachine::SEND_REQ;
        break;
      }
      case StatesMachine::SEND_REQ:
      {
    
        break;
      }
      default:
        break;
    }
  }

  
  else
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print("LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print("ALT=");
    Serial.print(gps.f_altitude() == TinyGPS::GPS_INVALID_F_ALTITUDE ? 0.0 : gps.f_altitude(), 5);
    Serial.print("HORA=");
    Serial.print(age == TinyGPS::GPS_INVALID_AGE ? 0.0 : age, 8);
  }
}



static void smartdelay(unsigned long ms)
{
  for (unsigned long start = millis(); millis() - start < ms;)
  {
    while (gpsserial.available())
    {
      char c = gpsserial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
}