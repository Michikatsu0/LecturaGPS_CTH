#include <Arduino.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

enum class StatesMachine { SMART_DELAY, CAL_TEMP, CAL_HUM, PRUN_TEMP, PRUN_HUM };
StatesMachine StateMachine = StatesMachine::SMART_DELAY;

SoftwareSerial gpsserial(2,0);
TinyGPS gps;

bool newData = false;
int iterations = 5;

static void smartdelay(unsigned long ms);

void setup() {
   // put your setup code here, to run once:
  gpsserial.begin(9600);
  Serial.begin(115200);
}

void loop()
{
  newData = false;
  
  switch (StateMachine)
  {
  case StatesMachine::SMART_DELAY:
  {
    smartdelay(100);
    break;
  }
  case StatesMachine::CAL_TEMP:
  {
    
    break;
  }
  
  default:
    break;
  }


  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" ALT=");
    Serial.print(age == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : age, 6);
    print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
    print_date(gps);
    print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
    Serial.print(" HORA=");
    
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