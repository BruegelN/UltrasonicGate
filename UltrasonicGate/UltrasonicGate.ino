// ---------------------------------------------------------------------------
// This is a modified version of of the NewPing Library example NewPing15Sensors.pde.
// It was made as a gate for RC Drift Cars. The gate is controlled by a servo.
// If there is a car in range of one sensor the gate will open and stays open until the car passes through.
// Or it will move back out of range, after a certain time the gate will close automatically (default 20 sec).
// UltrasonicGate utilise Servo- and NewPing-lib, for license information see below.
// Tested with an Arduino Uno, a regular Servo, two HC-SR04 Sensors and Arduino IDE 1.6.3 and v1.5 of NewPing-lib.
//
// DISCLAIMER:
// This software is furnished "as is", without technical support, and with no 
// warranty, express or implied, as to its usefulness for any purpose.
//
//
// ---------------------------------------------------------------------------
// NewPing Library - v1.5 - 08/15/2012
//
// AUTHOR/LICENSE:
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// LINKS:
// Project home: http://code.google.com/p/arduino-new-ping/
// Blog: http://arduino.cc/forum/index.php/topic,106043.0.html
// NewPing Library on Arduino homepage: http://playground.arduino.cc/Code/NewPing
// ---------------------------------------------------------------------------


/* delete the two slashes will give you the distance measured by both sensors on Serial Monitor with 115200 baud */
// #define debug

/* make use of ultra sonic & servo lib */
#include <NewPing.h>
#include <Servo.h>

/* pin the signal wire of the servo is attached to, make shure it is a PWM-pin */
uint8_t servoPin = 9;

/* pins for the first sensor (A) */
const uint8_t TRIGGER_PIN_A = 12;
const uint8_t ECHO_PIN_A = 11;
/* pins for the second sensor (B) */
const uint8_t TRIGGER_PIN_B = 4;
const uint8_t ECHO_PIN_B = 3;

/* this is the intervall by which the gate will open/close (time in milliseconds). By increasing this value the gate moves slower */
const long speedGate = 10;

/* time to wait until the gate closes automatically, if no car passes trough (time in milliseconds) */
const long waitTimeMillis = 20000; // 20000 = 20 sec

/* Maximum distance (in cm) to ping aka the range to detect a car */
const uint8_t MAX_DISTANCE = 18; 

/* 
* microseconds values for the servo 
* maybe you need to modify these values, depending on your servo and mechanic or even swap them
*/
int16_t posGateOpened = 2000;
int16_t posGateClosed = 1300;


/* 
* ########################################################################### 
* Don't modify the stuff below unless you know what you are doing.
* ###########################################################################
*/

/* some flags to remember the state */
bool isOpen = false;
bool passed = false;
bool passing = false;
bool openGate = false;
bool closeGate = false;

/*  to controll the speed of the gate by moving it in small intervals */
uint32_t currentMillis;
uint32_t previousMillis;

/* save time the gate is complete opend so it can close automatically, if no car passes trough */
uint32_t gateOpenedMillis;

/* at the beginning the gate is closed */ 
int16_t posGate = posGateClosed;

/* instantiate servo */
Servo Gate;

const uint8_t SONAR_NUM = 2; // Number or sensors.
const uint8_t PING_INTERVAL = 50; // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

uint32_t pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
uint16_t cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = 
{
  // Sensor object array.
  NewPing(TRIGGER_PIN_A, ECHO_PIN_A, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIGGER_PIN_B, ECHO_PIN_B, MAX_DISTANCE),
};

void setup()
{
  #ifdef debug
  Serial.begin(115200);
  #endif
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
  {
   pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
 }
 /* attach the servo object to the signal pin */
 Gate.attach(servoPin);  
 /* gate is closed */
 Gate.writeMicroseconds(posGateClosed);
}

void loop()
{
  for (uint8_t i = 0; i < SONAR_NUM; i++)
  {
    // Loop through all the sensors.
    if (millis() >= pingTimer[i])
    {
      // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      #ifdef debug
      if (i == 0 && currentSensor == SONAR_NUM - 1)
      {
        oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      }
      #endif
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // The rest of your code would go here.
  if (cm[0 !=0] && cm[1] == 0)
  {
    if (!openGate && !isOpen)
    {
      openGate = true;
    }
    else if (isOpen && passing)
    {
      passing = false;
      passed = true;
    }
  }
  else if (cm[0] == 0 && cm[1] !=0)
  {
    if (!openGate && !isOpen)
    {
      openGate = true;
    }
    else if ( isOpen && passing)
    {
      passing = false;
      passed = true;
    }
  }
  else if (cm[0] != 0 && cm[1] != 0)
  {
    passing = true;
  }
  else if (cm[0] == 0 && cm[1] == 0)
  {
    if (passed)
    {
      closeGate = true;
      passed = false;
    }
  }

  if (openGate)
  {
    
    if (posGate < posGateOpened)
    {
      isOpen = true;
      currentMillis = millis();
      if((currentMillis - previousMillis) >= speedGate)
      { 
        // save the last time we opend the gate a bit
        previousMillis = currentMillis;
        // increase position 
        posGate++;
        Gate.writeMicroseconds(posGate);
      }
    }
    else
    {
      openGate = false;
      gateOpenedMillis = millis();            
    }
  }

  if (closeGate)
  {
    isOpen = false;
    if (posGate > posGateClosed)
    {
      currentMillis = millis();
      if((currentMillis - previousMillis) >= speedGate)
      { 
        // save the last time we opend the gate a bit
        previousMillis = currentMillis;
        // increase position 
        posGate--;
        Gate.writeMicroseconds(posGate);
      }
    }
    else
    {
      closeGate = false;
    }
  }
  
  if (isOpen && !passing && !passed && (millis()-gateOpenedMillis) >= waitTimeMillis)
  {
    closeGate = true;    
  }
}



void echoCheck()
{
  // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
  cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
#ifdef debug
void oneSensorCycle() 
{
  // Sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}
#endif
