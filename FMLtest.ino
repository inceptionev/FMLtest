#include <PID_v1.h>

/* Copyright 2020, Edwin Chiu

  This file is part of FixMoreLungs.

  FixMoreLungs is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  FixMoreLungs is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FixMoreLungs.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
  Basic test and demo software for COVID-19/ARDS Ventilator

  Objective is to make a minimum-viable ARDS ventilator that can be deployed
  or constructed on-site in countries underserved by commecial global supply
  chains during the COVID-19 outbreak.

  Currently consists of a (1) CPAP-style blower with speed control.

  (2)Feedback from a differential pressure sensor with one side
  measuring delivered pressure to patient, other side ambient.


  created 16 Mar 2020
  Edwin Chiu
  Frost Methane Labs/Fix More Lungs
  Based on example code by Tom Igoe and Brett Beauregard

  Project Description: http://bit.ly/2wYqj3X
  git: https://github.com/inceptionev/FixMoreLungs
  www.pandemicventilator.com

  Outputs can be plotted with Cypress PSoC Programmer (Bridge Control Panel Tool)
  Download and install, connect serial
  Tools > Protocol Configuration > serial 115200:8n1 > hit OK
  In editor, use command RX8 [h=43] @1Key1 @0Key1 @1Key2 @0Key2
  Chart > Variable Settings
  Tick both Key1 and Key2, configure as int, and choose colors > hit OK
  Press >|< icon to connect to com port if necessary
  Click Repeat button, go to Chart tab
  both traces should now be plotting

*/

#include <PID_v1.h>

//PCB Rev 1.0 Pin assignments
#define PIN_PRES PA1
#define PIN_INH PA4
#define PIN_EXH PB0
#define PIN_VSENSE PA0
#define PIN_BUZZER PB4
#define PIN_BLOWER PB3
#define PIN_SOLENOID PA11
#define PIN_HEATER PA8
#define PIN_LED_R PC13
#define PIN_LED_Y PC14
#define PIN_LED_G PC15


#define BLOWER_HIGH 101
#define BLOWER_LOW 66

//state machine variables
#define INSPIRE_TIME 500
#define PIP 303 // = 20cmH2O (10bit scaling)
#define EXPIRE_TIME 750
#define PEEP 200 // = 5cmH2O (10bit scaling)
//not implemented yet
#define AC 0
#define RR 0
#define IE 0


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double ku = 0.7;
double tu = 0.16;
double Kp = 0.45*ku, Ki = 0.54*ku/tu, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// These constants won't change. They're used to give names to the pins used:
const int analogOutPin = LED_BUILTIN; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int flowValueINH = 0;
int flowValueEXH = 0;
int vsense = 0;

unsigned int cyclecounter = 0;
unsigned int state = 0;
unsigned int now = 0;
volatile unsigned long paddlePeriod = 0;
volatile unsigned long paddlePrev = 0;
volatile unsigned long paddlePeriod2 = 0;
volatile unsigned long paddlePrev2 = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_LED_R,OUTPUT);
  pinMode(PIN_LED_Y,OUTPUT);
  pinMode(PIN_LED_G,OUTPUT);
  pinMode(PIN_BUZZER,OUTPUT);
  pinMode(PIN_BLOWER,OUTPUT);
  pinMode(PIN_SOLENOID,OUTPUT);
  pinMode(PIN_HEATER,OUTPUT);

  //Initialize PID
  Input = analogRead(PIN_PRES);
  Setpoint = PEEP;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  pinMode(PIN_SOLENOID, OUTPUT); 
}

void loop() {

  switch(state) {
    case 0:
      cyclecounter++;
      //set command
      Setpoint = PIP;
      digitalWrite(PIN_SOLENOID,1);
      //update state
      if (cyclecounter > INSPIRE_TIME) {
        cyclecounter = 0;
        state = 1;
      }
      break;
            
    case 1:
      cyclecounter++;
      //set command
      Setpoint = PEEP;
      digitalWrite(PIN_SOLENOID,1);
      //update state
      if (cyclecounter > EXPIRE_TIME) {
        cyclecounter = 0;
        state = 0;
      }
      break;

    default:
      state = 0;
      break;
  }

  //Update PID Loop
  sensorValue = analogRead(PIN_PRES); //read sensor
  flowValueINH = analogRead(PIN_INH);
  flowValueEXH = analogRead(PIN_EXH);
  vsense = analogRead(PIN_VSENSE);
 
  Input = sensorValue;
  myPID.Compute(); // computer PID command
  analogWrite(PIN_BLOWER, Output); //write output
  now = (unsigned int)millis();
  //Serial.print("C"); //output to monitor
  //Serial.write(now>>8);
  //Serial.write(now&0xff);
  //Serial.write(int(map(Setpoint,0,255,0,1023))>>8); //output to monitor
  //Serial.write(int(map(Setpoint,0,255,0,1023))&0xff); //output to monitor
  //Serial.write(int(sensorValue)>>8); //output to monitor
  //Serial.write(int(sensorValue)&0xff); //output to monitor
  //Serial.write(int(flowValueINH)>>8); //output to monitor
  //Serial.write(int(flowValueINH)&0xff); //output to monitor
  //Serial.write(int(flowValueEXH)>>8); //output to monitor
  //Serial.write(int(flowValueEXH)&0xff); //output to monitor
  Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print(Output);
  Serial.print("\t");
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print(flowValueINH);
  Serial.print("\t");
  Serial.print(flowValueEXH);
  Serial.print("\t");
  Serial.println(vsense);
  delay(2);  //delay

}
