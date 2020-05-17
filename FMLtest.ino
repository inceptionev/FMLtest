#include <PID_v1.h>
#include <powerSTEP01ArduinoLibrary.h>
#include <SPI.h>
#include <Adafruit_GFX.h> //for OLED display
#include <Adafruit_SSD1306.h> //for OLED display
#include <Wire.h> //for OLED display

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
#define PIN_PRES PA1 //patient pressure sensor
#define PIN_INH PA4 //inhalation venturi dP sensor
#define PIN_EXH PB0 //exhalation ventiri dP sensor
#define PIN_VSENSE PA0 //input voltage resistor divider
#define PIN_BUZZER PB4 //buzzer driver pin
#define PIN_BLOWER PB3 //blower command pin
#define PIN_SOLENOID PA11 //solenoid switch control pin
#define PIN_HEATER PA8 //heater switch control pin
#define PIN_LED_R PC13 //RED alarm led control pin
#define PIN_LED_Y PC14 //YELLOW alarm led control pin (does not work in Rev 1.0 PCB due to pin conflict with oscillator)
#define PIN_LED_G PC15 //GREEN alarm led control pin (does not work in Rev 1.0 PCB due to pin conflict with oscillator)
// Pin definitions for the X-NUCLEO-IHM03A1 (stepper driver)
#define nCS_PIN PB6
#define STCK_PIN PC7
#define nSTBY_nRESET_PIN PA9
#define nBUSY_PIN PB5

//Pinch valve motion settings
#define STARTSTROKE 7000
#define OPENPOS 1000
#define CLOSEDPOS 6750 //stop is a 6900

#define BLOWER_HIGH 101
#define BLOWER_LOW 66

//state machine variables
#define INSPIRE_TIME 400
#define PIP 350 //  (10bit scaling)
#define EXPIRE_TIME 400
#define PEEP 245 // (10bit scaling)
//not implemented yet
#define AC 0
#define RR 0
#define IE 0

//experimental: blower feed forward
#define BLOWER_PIP 220 //blower speed at PIP
#define BLOWER_PEEP 220 //blower speed at PEEP

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double ku = 0.8;  //maximum P gain in P-only mode that generates a stable oscillation
double tu = 0.186; //oscillation period of the above
//double Kp = 0.45*ku, Ki = 0.54*ku/tu, Kd = 0; //calculates gains using ziegler-nichols method of PI control
//double Kp = ku/5, Ki = 2*ku/tu/5, Kd = ku*tu/15; //no overshoot rule
double Kp=0.13, Ki=0.7, Kd=0; //base values - this kinda worked slow but stable
//double Kp=0.8, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// These constants won't change. They're used to give names to the pins used:
const int analogOutPin = LED_BUILTIN; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from pressure sensor
int outputValue = 0;        // value output to the PWM (analog out)
int flowValueINH = 0;       // value read from inhale flow sensor
int flowValueEXH = 0;       // value read from exhale flow sensor
int vsense = 0;

unsigned int cyclecounter = 0; //used to time the cycles in the state machine
unsigned int state = 0; //state machine state
unsigned int now = 0; // carry the time

//instantiate stepper driver
powerSTEP driver(0, nCS_PIN, nSTBY_nRESET_PIN);

//i2c test device definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  //instantiate display

void setup() {
  // initialize serial communications at 115200 bps:
  analogWrite(PIN_BLOWER, 0);  //Blower off until cycle starts.
  Serial.begin(115200);

  //set pin modes
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_LED_R,OUTPUT);
  pinMode(PIN_LED_Y,OUTPUT);
  pinMode(PIN_LED_G,OUTPUT);
  pinMode(PIN_BUZZER,OUTPUT);
  pinMode(PIN_BLOWER,OUTPUT);
  pinMode(PIN_SOLENOID,OUTPUT);
  pinMode(PIN_HEATER,OUTPUT);
  //Stepper driver pins
  pinMode(nSTBY_nRESET_PIN, OUTPUT);
  pinMode(nCS_PIN, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, OUTPUT);

  
  // Reset powerSTEP and set CS
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nSTBY_nRESET_PIN, LOW);
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nCS_PIN, HIGH);

  // Start SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  // Configure powerSTEP
  driver.SPIPortConnect(&SPI); // give library the SPI port
  
  driver.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
                                     
  driver.configStepMode(STEP_FS_128); // 1/128 microstepping, full steps = STEP_FS,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128
                                
  driver.setMaxSpeed(1000); // max speed in units of full steps/s 
  driver.setFullSpeed(2000); // full steps/s threshold for disabling microstepping
  driver.setAcc(2000); // full steps/s^2 acceleration
  driver.setDec(2000); // full steps/s^2 deceleration
  
  driver.setSlewRate(SR_520V_us); // faster may give more torque (but also EM noise),
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)
                                  
  driver.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
                            // used in testing. If your motor stops working for
                            // no apparent reason, it's probably this. Start low
                            // and increase until it doesn't trip, then maybe
                            // add one to avoid misfires. Can prevent catastrophic
                            // failures caused by shorts
  driver.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage
  
  driver.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a sin wave,  
                            // frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2
                            
  driver.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
                                          
  driver.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
                                 // SW_HARD_STOP: TP1 causes hard stop on connection 
                                 // to GND, you get stuck on switch after homing
                                      
  driver.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
  // use a value between 0-255 where 0 = no power, 255 = full power.
  // Start low and monitor the motor temperature until you find a safe balance
  // between power and temperature. Only use what you need
  driver.setRunKVAL(60); //2.8V in voltage mode for 2A max on 1.4ohm coils
  driver.setAccKVAL(60);
  driver.setDecKVAL(60);
  driver.setHoldKVAL(32);

  driver.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
                                   // disable stall detection (not configured),
                                   // disable switch (not using as hard stop)

  driver.getStatus(); // clears error flags

  //home the actuator
  driver.move(REV, STARTSTROKE); // move into the stop
  while(driver.busyCheck()); // wait fo the move to finish
  driver.resetPos(); //establish home

  //Setup display (i2c test)
  Wire.begin();
  Wire.beginTransmission(0x70); //address the i2c switch
  Wire.write(7); //select i2c port, base address 4, cycle thru 5-7
  Wire.endTransmission(); //send and stop
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  //Initialize PID
  Input = analogRead(PIN_PRES);
  Setpoint = PEEP;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}

void loop() {

  //respiration cycle state machine
  //times states and sets setpoint
  switch(state) {
    case 0:
      //set command
      Setpoint = PIP;
      analogWrite(PIN_BLOWER, BLOWER_PIP); //write output to blower
      digitalWrite(PIN_SOLENOID,1);
      analogWrite(PIN_BUZZER, 10);
      //display once at the start of the cycle
      //update state
      cyclecounter++;
      if (cyclecounter > INSPIRE_TIME) {
        cyclecounter = 0;
        state = 1;
      }
      break;
            
    case 1:
      //set command
      Setpoint = PEEP;
      analogWrite(PIN_BLOWER, BLOWER_PEEP); //write output to blower
      digitalWrite(PIN_SOLENOID,1);
      analogWrite(PIN_BUZZER, 0);
      //display once at the start of the cycle
      //update state
      cyclecounter++;
      if (cyclecounter > EXPIRE_TIME) {
        cyclecounter = 0;
        state = 0;
      }
      break;

    default:
      state = 0;
      break;
  }

  //CYCLE ACTION BLOCK
  //read sensors
  sensorValue = analogRead(PIN_PRES); //read sensor
  flowValueINH = analogRead(PIN_INH);
  flowValueEXH = analogRead(PIN_EXH);
  vsense = analogRead(PIN_VSENSE);

  //Update PID Loop
  Input = sensorValue;
  myPID.Compute(); // compute PID command
  driver.goTo(map(Output,0,255,CLOSEDPOS,OPENPOS));
  now = (unsigned int)millis();
  //Output serial data in Cypress Bridge Control Panel format
  Serial.print("C"); //output to monitor
  Serial.write(now>>8);
  Serial.write(now&0xff);
  Serial.write(int(Setpoint)>>8); //output to monitor
  Serial.write(int(Setpoint)&0xff); //output to monitor
  Serial.write(int(Output)>>8);
  Serial.write(int(Output)&0xff);
  Serial.write(int(sensorValue)>>8); //output to monitor
  Serial.write(int(sensorValue)&0xff); //output to monitor
  Serial.write(int(flowValueINH)>>8); //output to monitor
  Serial.write(int(flowValueINH)&0xff); //output to monitor
  Serial.write(int(flowValueEXH)>>8); //output to monitor
  Serial.write(int(flowValueEXH)&0xff); //output to monitor

  //Output serial data in Arduino Plotter format
  /*Serial.print(Setpoint);
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
  */
  delay(10);  //delay

}
