#include <Arduino.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>

enum class StatesMachine { SMART_DELAY, CAL_TEMP, CAL_HUM, PRUN_TEMP, PRUN_HUM };
StatesMachine StateMachine = StatesMachine::SMART_DELAY;

SoftwareSerial gpsserial(2,0);
TinyGPS gps;

bool newData = false;
int iterations = 5;
double dataTemp;
double dataHume;
static void smartdelay(unsigned long ms);

void setup() 
{
  // put your setup code here, to run once:
  gpsserial.begin(9600);
  Serial.begin(115200);
}

void loop()
{
  if (!newData)
  {
    switch (StateMachine)
    {
      case StatesMachine::SMART_DELAY:
      {
        smartdelay(100);
        StateMachine = StatesMachine::CAL_TEMP;          break;
      }
      case StatesMachine::CAL_TEMP:
      {
        for (int i = 0; i < iterations; i++)
        {
          //dataTemp += Sens
        }
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
    Serial.print(gps.f_altitude());
    Serial.print("AGE");
    Serial.print(age == TinyGPS::GPS_INVALID_AGE ? 0.0 : age, 5);
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
      else
        newData = false;
    }
  }
}
