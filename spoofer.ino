// This uses the RostaBoard to serve as a fake GPS receiver,
// mainly to simulate a quick near-space balloon flight.
//--------------------------------------------------------------------
// INCLUDES
//--------------------------------------------------------------------
#include "SoftwareSerial.h"
#include <avr/pgmspace.h>
#include "SoftwareSerial.h"
#include <avr/pgmspace.h>

//--------------------------------------------------------------------
// CONSTANTS
//--------------------------------------------------------------------
// Ones you may want to mod
#define CLIMBING_ALTITUDE_FT_DELTA 100
#define DESCENDING_ALTITUDE_FT_DELTA 200
#define ACCELERATED_MULTIPLIER 100
#define APOGEE_FT 93000
#define LATITUDE_DELTA_SECONDS 1
#define LONGITUDE_DELTA_SECONDS 1
#define STARTING_TIME_HOURS 12
#define STARTING_TIME_MINUTES 34
#define STARTING_TIME_SECONDS 56
#define STARTING_LATITUDE_INDIGO 3025.00
#define STARTING_LONGITUDE_INDIGO 3025.00
#define STARTING_LATITUDE_AUSTIN 3025.00
#define STARTING_LONGITUDE_AUSTIN 3025.00

// Steady ones
#define BIG_BUFFER_SIZE 200
#define LITTLE_BUFFER_SIZE 50
#define GPS_TX_PIN 2
#define GPS_RX_PIN 3
#define LED_PIN 13
#define SWITCH_PIN 4
#define GROUND_ALTITUDE_FT 900
#define GPGGAa 0
#define GPGGAb 1
#define GPGSA  2 
#define GPGSV1 3 
#define GPGSV2 4 
#define GPGSV3 5 
#define GPRMC  6  

//--------------------------------------------------------------------
// GLOBALS
//--------------------------------------------------------------------int led = 13;
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN, true); 
char bigBuffer[BIG_BUFFER_SIZE];
char littleBuffer[LITTLE_BUFFER_SIZE];
long numGpsLinesWritten = 0L;
long numGpgsasWritten = 0L;
long altitude_ft = GROUND_ALTITUDE_FT;
short time_hours = STARTING_TIME_HOURS;
short time_minutes = STARTING_TIME_MINUTES;
short time_seconds = STARTING_TIME_SECONDS;
short time_milliseconds = 0;
int altitudeDeltaMultiplier = 1;
boolean acceleratedMode = false;
int acceleration = 1;
float latitude = STARTING_LATITUDE_AUSTIN;
float longitude = STARTING_LONGITUDE_AUSTIN;

                     //           1111111111222222222233333333334444444444555555555566666666667777777777
                     // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
char nmea1[] PROGMEM = "$GPGGA,tttttt.000,3028.9534,N,09747.6203,W,1,05,6.3,";
char nmea2[] PROGMEM = ".0,M,-22.5,M,,*6B";
char nmea3[] PROGMEM = "$GPGSA,A,3,22,14,18,11,12,,,,,,,,6.6,6.3,1.7*3E";
char nmea4[] PROGMEM = "$GPGSV,3,1,12,22,67,044,38,14,66,339,34,18,39,094,43,11,13,311,39*7B";
char nmea5[] PROGMEM = "$GPGSV,3,2,12,12,08,081,38,31,29,193,22,19,14,272,21,25,12,116,*7B";
char nmea6[] PROGMEM = "$GPGSV,3,2,12,12,08,081,38,31,29,193,22,19,14,272,21,25,12,116,*7B";
char nmea7[] PROGMEM = "$GPRMC,tttttt.000,A,3028.9534,N,09747.6203,W,0.00,,050414,,,A*69";

PGM_P nmeatable[] PROGMEM = 
{
    nmea1,
    nmea2,
    nmea3,
    nmea4,
    nmea5,
    nmea6,
    nmea7
};

void setup()
{
  delay(5000);
  ss.begin(4800); 
  Serial.begin(115200); 
  pinMode(LED_PIN, OUTPUT);  
  pinMode(SWITCH_PIN, INPUT);
  randomSeed(analogRead(0));
 }

//--------------------------------------------------------------------
// MAIN PROGRAM
//--------------------------------------------------------------------
void loop()
{  
  delay(1000);
  AdvanceTime(1000);

  digitalWrite(LED_PIN, HIGH);
  WriteGPGGA();
  AdvanceTime(14);
  digitalWrite(LED_PIN, LOW);  
  numGpsLinesWritten++;
  delay(100);
  AdvanceTime(100);

  digitalWrite(LED_PIN, HIGH);
  WriteGPGSA();
  AdvanceTime(14);
  digitalWrite(LED_PIN, LOW);  
  numGpsLinesWritten++;   
  numGpgsasWritten++;
  delay(100);
  AdvanceTime(100);

  if (numGpgsasWritten % 5 == 0)
  {
     digitalWrite(LED_PIN, HIGH);
     WriteGPGSV1(); 
     AdvanceTime(14);
     digitalWrite(LED_PIN, LOW);  
     numGpsLinesWritten++;
     delay(100);
     AdvanceTime(100);
  
     digitalWrite(LED_PIN, HIGH);
     WriteGPGSV2(); 
     AdvanceTime(14);
     digitalWrite(LED_PIN, LOW);  
     numGpsLinesWritten++;
     delay(100);
     AdvanceTime(100);
  
     digitalWrite(LED_PIN, HIGH);
     WriteGPGSV3(); 
     AdvanceTime(14);
     digitalWrite(LED_PIN, LOW);  
     numGpsLinesWritten++;
     delay(100);
     AdvanceTime(100);
  }
  
   digitalWrite(LED_PIN, HIGH);
   WriteGPRMC();
   AdvanceTime(14);
   digitalWrite(LED_PIN, LOW);  
   numGpsLinesWritten++;
   
   AdvanceAltitude();
   AdvancePosition();

}
 
//--------------------------------------------------------------------
// AdvanceAltitude
//--------------------------------------------------------------------
void AdvanceAltitude()
{
   if (digitalRead(SWITCH_PIN) == 1)
     acceleration = ACCELERATED_MULTIPLIER;
   else 
     acceleration = 1;
 
   altitude_ft += CLIMBING_ALTITUDE_FT_DELTA * altitudeDeltaMultiplier * acceleration;

   if (altitudeDeltaMultiplier == -1 && altitude_ft <= GROUND_ALTITUDE_FT)
   {
     altitudeDeltaMultiplier = 0;  
     altitude_ft = GROUND_ALTITUDE_FT;
   }
   
   if (altitude_ft >= APOGEE_FT)
     altitudeDeltaMultiplier *= -1;
}

//--------------------------------------------------------------------
// AdvancePosition
//--------------------------------------------------------------------
void AdvancePosition()
{  
  float latMoveDelta = (random(100) - 50) * 0.001f;  // Keeps it going basically straight along the launch lat's parallel
  latitude += latMoveDelta;
  float lonMoveDelta = random(100) * 0.001f;  
  longitude -= lonMoveDelta; // Easterly
}

//--------------------------------------------------------------------
// AdvanceTime
//--------------------------------------------------------------------
void AdvanceTime(short ms)
{
  if (ms == 1000)
    time_seconds++;
    
  else
  {
      time_milliseconds += ms;
      
      if (time_milliseconds >= 1000)
      {
         time_seconds += time_milliseconds / 1000;
         time_milliseconds %= 1000;
      }
  }
  
  if (time_seconds >= 60)
  {
     time_minutes += time_seconds / 60;
     time_seconds %= 60;
  }
    
  if (time_minutes >= 60)
  {
     time_hours += time_minutes / 60;
     time_minutes %= 60;
  }
  
  if (time_hours > 24)
    time_hours = 0;
}

//--------------------------------------------------------------------
// MakeTimeString (puts into littleBuffer
//--------------------------------------------------------------------
void MakeTimeString()
{
  sprintf(littleBuffer, "%02d%02d%02d", time_hours, time_minutes, time_seconds);
}


//--------------------------------------------------------------------
// WriteGPGGA
//--------------------------------------------------------------------
void WriteGPGGA()
{
  // $GPGGA,tttttt.000,3028.9534,N,09747.6203,W,1,05,6.3,
  strcpy_P(bigBuffer, (PGM_P)pgm_read_word(&(nmeatable[GPGGAa])));  
  MakeTimeString();
  strncpy(bigBuffer + 7, littleBuffer, 6);
  double alt_m = (double)altitude_ft;
  alt_m /= 3.28;
  int altitude_m = (int)alt_m;
  
  sprintf(littleBuffer, "%d", altitude_m);
  strcat(bigBuffer, littleBuffer);
  strcat_P(bigBuffer, (PGM_P)pgm_read_word(&(nmeatable[GPGGAb])));  

  dtostrf(latitude, 9, 4, littleBuffer);  
  strncpy(bigBuffer + 18, littleBuffer, strlen(littleBuffer));
  dtostrf(longitude, 9, 4, littleBuffer);  
  strncpy(bigBuffer + 31, littleBuffer, strlen(littleBuffer));
  
  Serial.println(bigBuffer); 
  ss.println(bigBuffer);  
}
//--------------------------------------------------------------------
// WriteGPGSA
//--------------------------------------------------------------------
void WriteGPGSA()
{
  strcpy_P(bigBuffer, (PGM_P)pgm_read_word(&(nmeatable[GPGSA])));  
  Serial.println(bigBuffer); 
  ss.println(bigBuffer);  
}
//--------------------------------------------------------------------
// WriteGPGSV1
//--------------------------------------------------------------------
void WriteGPGSV1()
{
  strcpy_P(bigBuffer, (PGM_P)pgm_read_word(&(nmeatable[GPGSV1])));  
  Serial.println(bigBuffer); 
  ss.println(bigBuffer);  
}
//--------------------------------------------------------------------
// WriteGPGSV2
//--------------------------------------------------------------------
void WriteGPGSV2()
{
  strcpy_P(bigBuffer, (PGM_P)pgm_read_word(&(nmeatable[GPGSV2])));  
  Serial.println(bigBuffer); 
  ss.println(bigBuffer);  
  
}
//--------------------------------------------------------------------
// WriteGPGSV3
//--------------------------------------------------------------------
void WriteGPGSV3()
{
  strcpy_P(bigBuffer, (PGM_P)pgm_read_word(&(nmeatable[GPGSV3])));  
  Serial.println(bigBuffer); 
  ss.println(bigBuffer);  
  
}
//--------------------------------------------------------------------
// WriteGPRMC
//--------------------------------------------------------------------
void WriteGPRMC()
{
  strcpy_P(bigBuffer, (PGM_P)pgm_read_word(&(nmeatable[GPRMC])));  
  MakeTimeString();
  strncpy(bigBuffer + 7, littleBuffer, 6);

  dtostrf(latitude, 9, 4, littleBuffer);  
  strncpy(bigBuffer + 20, littleBuffer, strlen(littleBuffer));
  dtostrf(longitude, 9, 4, littleBuffer);  
  strncpy(bigBuffer + 33, littleBuffer, strlen(littleBuffer));
  
  Serial.println(bigBuffer); 
  ss.println(bigBuffer);  
}

//--------------------------------------------------------------------
// ReadGpsChar
//--------------------------------------------------------------------
char ReadGpsChar()
{
    while (!ss.available())
      delay(10);
    
    return ss.read();  
}
