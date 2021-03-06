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
#define OPENPOS 4000 //effective position begins at 4000
#define CLOSEDPOS 6500 //stop is at 6900

//Proportional Solenoid motion settings
#define PSOL_CLOSEDPOS 1000
#define PSOL_OPENPOS 1500

#define BLOWER_HIGH 255 
#define BLOWER_LOW 255 //blower always full throttle in pinch valve control mode (at least for now)

//state machine settings
#define CONTROL_PERIOD 10
#define INSPIRE_TIME 4000 //in ms
//#define PIP 907 // 907 = 35cm  (10bit scaling)
#define PIP 14523 // 14523 = 35cm (14bit scaling)
//#define PIP 305 // set all to PEEP pressure to capture inspire effort
#define EXPIRE_TIME 4000 //in ms
//#define PEEP 305 // 305 = 5cm(10bit scaling)
//#define PEEP 8096 //8096 = 15cm PEEP (14bit scaling)
#define PEEP 4883
//not implemented yet
#define AC 0
#define RR 0
#define IE 0

//experimental: blower feed forward
#define BLOWER_PIP 255 //blower speed at PIP
#define BLOWER_PEEP 255//blower speed at PEEP

//experimental: exhale valve feed forward positions
#define EXH_PIP 5250
#define EXH_PEEP 4750

//experimental: oxygen mixing 0 (no additional oxygen) to 1 (only oxygen)
#define OXYMIX 0  //currently this is only tested for 0 (pure air) and 1 (pure oxygen) with oxygen inlet pressure 0.12Mpa


//Define Variables we'll be connecting to
double SetpointAIR, InputAIR, OutputAIR;
double KuAIR = 0.01875, TuAIR = 0.2; //more aggressive - quicklung values - 14bit values
double KpAIR = 0.45*KuAIR, KiAIR = 0.54*KuAIR/TuAIR, KdAIR = 0; //Ziegler-Nichols PI
PID PID_AIR(&InputAIR, &OutputAIR, &SetpointAIR, KpAIR, KiAIR, KdAIR, DIRECT);

//THESE ARE OTHER VALUES THAT WERE TRIED
//double KpAIR=0.13, KiAIR=0.7, KdAIR=0; //10bit base values - this kinda worked slow but stable THESE ARE THE VALUES THAT WORKED FOR BLOWER PINCH ONLY
//double KuAIR = 1.7, TuAIR = 0.23; //10bit more aggressive - shittylung values
//double KuAIR = 0.2, TuAIR = 0.2; //10bit less aggressive - quicklung values
//double KuAIR = 1.3, TuAIR = 0.2; //10bit less aggressive - shittylung values
//double KpAIR = 0.2*KuAIR, KiAIR = 0.4*KuAIR/TuAIR, KdAIR = KuAIR*TuAIR/15; // Ziegler-Nichols PID no overshoot
//double KpAIR=0.13, KiAIR=0.7, KdAIR=0; //for getting Ku and Tu

//Define Variables we'll be connecting to
double SetpointOXY, InputOXY, OutputOXY;
double KpOXY=0.0003125, KiOXY=0.001875, KdOXY=0; //14bit working values for pure O2 PSOL on shittylung
PID PID_OXY(&InputOXY, &OutputOXY, &SetpointOXY, KpOXY, KiOXY, KdOXY, DIRECT);

//THESE ARE OTHER VALUES THAT WERE TRIED
//double KpOXY=0.005, KiOXY=0.03, KdOXY=0; //10bit working values for pure O2 PSOL on shittylung

// These constants won't change. They're used to give names to the pins used:
const int analogOutPin = LED_BUILTIN; // Analog output pin that the LED is attached to

//Sensor variables
uint16_t pressureValue = 0;        // value read from pressure sensor
uint16_t outputValue = 0;        // value output to the PWM (analog out)
uint16_t flowValueINH = 0;       // value read from inhale flow sensor
uint16_t flowValueEXH = 0;       // value read from exhale flow sensor
uint16_t vsense = 0;
int CommandEXH = OPENPOS;

//State machine variables
unsigned int cyclecounter = 0; //used to time the cycles in the state machine
unsigned int state = 0; //state machine state
unsigned int now = 0; // carry the time
double Setpoint = 0;

//instantiate stepper driver
//instantiate stepper driver
powerSTEP driverINH(0, nCS_PIN, nSTBY_nRESET_PIN);
powerSTEP driverEXH(1, nCS_PIN, nSTBY_nRESET_PIN);

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

  analogReadResolution(14);

  
  // Reset powerSTEP and set CS
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nSTBY_nRESET_PIN, LOW);
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nCS_PIN, HIGH);

  // Start SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  //configure all the driver settings for the stepper drivers.
  configureStepperDrives();  

  //home the actuator
  driverEXH.move(REV, STARTSTROKE);
  driverINH.move(REV, STARTSTROKE); // move into the stop
  //while(driverINH.busyCheck()); // wait fo the move to finish - replaced this with a wait so it becomes non-blocking
  delay(2000);
  driverEXH.resetPos();
  driverINH.resetPos();

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
  pressureValue = analogRead(PIN_PRES);
  InputAIR = pressureValue;
  InputOXY = pressureValue;
  Setpoint = PEEP;
  SetpointAIR = Setpoint;
  SetpointOXY = Setpoint;
  //turn the PID on
  PID_AIR.SetMode(AUTOMATIC);
  PID_OXY.SetMode(AUTOMATIC);

}

void loop() {

  //respiration cycle state machine
  //times states and sets setpoint
  switch(state) {
    case 0:
      //set command
      Setpoint = PIP;
      CommandEXH = EXH_PIP;
      analogWrite(PIN_BLOWER, BLOWER_PIP); //write output to blower
      analogWrite(PIN_BUZZER, 3);
      //display once at the start of the cycle
      //update state
      cyclecounter++;
      if (cyclecounter > int(INSPIRE_TIME/CONTROL_PERIOD)) {
        cyclecounter = 0;
        state = 1;
      }
      break;
            
    case 1:
      //set command
      Setpoint = PEEP;
      CommandEXH = EXH_PEEP;
      analogWrite(PIN_BLOWER, BLOWER_PEEP); //write output to blower
      analogWrite(PIN_BUZZER, 0);
      //display once at the start of the cycle
      //update state
      cyclecounter++;
      if (cyclecounter > int(EXPIRE_TIME/CONTROL_PERIOD)) {
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
  pressureValue = analogRead(PIN_PRES); //read sensor
  flowValueINH = analogRead(PIN_INH);
  flowValueEXH = analogRead(PIN_EXH);
  vsense = analogRead(PIN_VSENSE);

  //Update PID Loop
  SetpointAIR = Setpoint;
  SetpointOXY = Setpoint;
  InputAIR = pressureValue;
  InputOXY = pressureValue;
  PID_AIR.Compute(); // compute air (blower + pinch valve) PID command
  PID_OXY.Compute(); // compute oxygen (PSOL) PID command
  analogWrite(PIN_SOLENOID, map(int(OXYMIX*OutputOXY),0,255,PSOL_CLOSEDPOS,PSOL_OPENPOS));
  driverINH.goTo(map(int((1-OXYMIX)*OutputAIR),0,255,CLOSEDPOS,OPENPOS));
  driverEXH.goTo(CommandEXH);
  
  now = (unsigned int)millis();
  //Output serial data in Cypress Bridge Control Panel format
  Serial.print("C"); //output to monitor
  Serial.write(now>>8);
  Serial.write(now&0xff);
  Serial.write(int(Setpoint)>>8); //output to monitor
  Serial.write(int(Setpoint)&0xff); //output to monitor
  Serial.write(int(OutputAIR)>>8);
  Serial.write(int(OutputAIR)&0xff);
  Serial.write(int(OutputOXY)>>8);
  Serial.write(int(OutputOXY)&0xff);
  Serial.write(int(pressureValue)>>8); //output to monitor
  Serial.write(int(pressureValue)&0xff); //output to monitor
  Serial.write(int(flowValueINH)>>8); //output to monitor
  Serial.write(int(flowValueINH)&0xff); //output to monitor
  Serial.write(int(flowValueEXH)>>8); //output to monitor
  Serial.write(int(flowValueEXH)&0xff); //output to monitor

  //Output serial data in Arduino Plotter format
  /*Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print(Output);
  Serial.print("\t");
  Serial.print(pressureValue);
  Serial.print("\t");
  Serial.print(flowValueINH);
  Serial.print("\t");
  Serial.print(flowValueEXH);
  Serial.print("\t");
  Serial.println(vsense);
  */
  delay(CONTROL_PERIOD);  //delay

}

void configureStepperDrives() {
  // Configure powerSTEP
  driverEXH.SPIPortConnect(&SPI); // give library the SPI port
  driverINH.SPIPortConnect(&SPI); // give library the SPI port
  
  driverEXH.configSyncPin(BUSY_PIN, 0);
  driverINH.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
                                     
  driverEXH.configStepMode(STEP_FS_128);
  driverINH.configStepMode(STEP_FS_128); // 1/128 microstepping, full steps = STEP_FS,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128

  driverEXH.setMaxSpeed(1000);                              
  driverINH.setMaxSpeed(1000); // max speed in units of full steps/s
  driverEXH.setFullSpeed(2000); 
  driverINH.setFullSpeed(2000); // full steps/s threshold for disabling microstepping
  driverEXH.setAcc(2000);
  driverINH.setAcc(2000); // full steps/s^2 acceleration
  driverEXH.setDec(2000);
  driverINH.setDec(2000); // full steps/s^2 deceleration

  driverEXH.setSlewRate(SR_520V_us);
  driverINH.setSlewRate(SR_520V_us); // faster may give more torque (but also EM noise),
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)

  driverEXH.setOCThreshold(8);                                
  driverINH.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
                            // used in testing. If your motor stops working for
                            // no apparent reason, it's probably this. Start low
                            // and increase until it doesn't trip, then maybe
                            // add one to avoid misfires. Can prevent catastrophic
                            // failures caused by shorts

  driverEXH.setOCShutdown(OC_SD_ENABLE);
  driverINH.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage

  driverEXH.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75);
  driverINH.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a sin wave,  
                            // frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2

  driverEXH.setVoltageComp(VS_COMP_DISABLE);
  driverINH.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
  driverEXH.setSwitchMode(SW_USER);                                        
  driverINH.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
                                 // SW_HARD_STOP: TP1 causes hard stop on connection 
                                 // to GND, you get stuck on switch after homing

  driverEXH.setOscMode(INT_16MHZ);
  driverINH.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
  // use a value between 0-255 where 0 = no power, 255 = full power.
  // Start low and monitor the motor temperature until you find a safe balance
  // between power and temperature. Only use what you need
  driverEXH.setRunKVAL(60);
  driverINH.setRunKVAL(60); //2.8V in voltage mode for 2A max on 1.4ohm coils
  driverEXH.setAccKVAL(60);
  driverINH.setAccKVAL(60);
  driverEXH.setDecKVAL(60);
  driverINH.setDecKVAL(60);
  driverEXH.setHoldKVAL(32);
  driverINH.setHoldKVAL(32);

  driverEXH.setParam(ALARM_EN, 0x8F);
  driverINH.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
                                   // disable stall detection (not configured),
                                   // disable switch (not using as hard stop)
  driverEXH.getStatus();
  driverINH.getStatus(); // clears error flags
  
}
