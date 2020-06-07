# FMLtest
Simple test code for Edwin's STM32 ventilator setup

 Test code with a state machine for running a ventilator setup in closed-loop control mode
 
 * Note: this is personal testing code for validating hardware and is not intended to be supported.  (i.e. proceeed at your own risk)
 * Written for use on Ventilator Mainboard PCB Rev 1.0, running STM32 on STM32duino with a single powerSTEP01 stepper driver.
 
 * The PCB design itself is fully-suported, repository including quick start guide, list of dependencies, and programming instructions here: https://github.com/inceptionev/pcbreathe-bringup
     * Note: you will NEED the dependencies listed in the documentation above to run this code.  Go there first if you are programming this board for the first time using this test code.
 * For the main ventilator code, for use on this PCB, see https://github.com/RespiraWorks/VentilatorSoftware.
 * For semi-supported test code for exercising functions on the PCB, see https://github.com/inceptionev/pcbreathe-bringup
 
 ## GETTING DATA OUT:
Outputs can be plotted and exported with Cypress PSoC Programmer (Bridge Control Panel Tool)
* Download and install, connect serial
* Tools > Protocol Configuration > serial 115200:8n1 > hit OK
* In the editor tab, use this command:

    ```RX8 [h=43] @1Key1 @0Key1 @1Key2 @0Key2 @1Key3 @0Key3 @1Key4 @0Key4 @1Key5 @0Key5 @1Key6 @0Key6 @1Key7 @0Key7```
* In order, each of these outputs is: Time(ms), Pressure Setpoint, AIR Valve Command, OXY Valve Command, Patient Pressure, Inhale dP, Exhale dP
* The MPXV5004DPs come out as 14bit values and the scaling is ```kPa = 5*Value/16383-1```.
* The MPXV5004DP sensors are assigned to Pressure, Inhale, and Exhale on this board, but of course you can connect them to anything you want to measure
* The hypodermic needles can be useful for picking off pressures anywhere you have rubber tubing.
* Be careful not to poke yourself.
* Watch out for dynamic pressure effects when using the needles.  I have found better results by inserting the needle at an acute angle to the flow, pointing downstream, and retracting the needle such that the opening is near the sidewall of the hose.

![hypodermic needle pickoff](needle_pressure_pickoff.jpg)
* Chart > Variable Settings
* Tick Key1 through Key7, configure as int, and choose colors > hit OK
* Press >|< icon to connect to com port if necessary
* Click REPEAT button, go to Chart tab  
* both traces should now be plotting
* Click STOP button to stop recording.
* Chart > Export Collected Data in the format of your choice.  Note that this method captures a maximum of 10,000 samples.  It will clip the beginning of your experiment if it is longer than 10k samples.
* You can run longer experiments within the 10k sample limit by increasing CONTROL_PERIOD, which will reduce the sampling rate.
* If you need more than 10k samples use a different logger (or use the TO FILE button instead of REPEAT, it will output hex data that you can copy-and-paste, so be prepared to do some post-processing)

## HOW TO USE THIS TEST (generic to pcbreathe-bringup):
* Follow the instructions on the https://github.com/inceptionev/pcbreathe-bringup readme to set up the hardware and the IDE.
    


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
