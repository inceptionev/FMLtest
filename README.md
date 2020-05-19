# FMLtest
Simple test code for Edwin's arduino ventilator setup

 Test code with a state machine for running a ventilator setup in closed-loop control mode
 
 * Note: this is personal testing code for validating hardware and is not intended to be supported.  (i.e. proceeed at your own risk)
 * Written for use on Ventilator Mainboard PCB Rev 1.0, running STM32 on STM32duino with a single powerSTEP01 stepper driver.
 
 * The PCB design itself is fully-suported, repository including quick start guide and programming instructions here: https://github.com/inceptionev/pcbreathe-bringup
 * For the main ventilator code, for use on this PCB, see https://github.com/RespiraWorks/VentilatorSoftware.
 * For semi-supported test code for exercising functions on the PCB, see https://github.com/inceptionev/pcbreathe-bringup

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
