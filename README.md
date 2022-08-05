# MediaController

Design and programming by Jon Becker

This is a media controller to support Thembi Soddell's sound art installation.
Goals are:
An external 'outside' button to open the door.
An internal 'user' button to indicate the user is in place and ready to begin, and to abort the experience if the user is uncomfortable continuing the experience.
An operator button to assist with aborting the media.
LEDs on the buttons to indicate functions and status.
Bright internal RGBW LEDs (here implemented by Adafruit 4W Dotstars) which can be changed to a different color and brightness and controlled as desired.
A linear actuator to open and close the door.
A relay to control the media (on to reset and play, off to stop playing).

The system is implemented with an Adafruit Feather M4 Express microcontroller, and programmed in C using the Arduino IDE.
It was developed on Arduino IDE 1.8.19.
# Libraries:
To install, the following libraries are needed (they can be installed throught he Arduino Library Manager).
'Sparkfun QWIIC Relay' library
'Adafruit BUSIO' library
'Adafruit seesaw' library

# Hardware used:

## Adafruit 4W RGBW pixels
https://www.adafruit.com/product/5408

## Level shifter
https://learn.sparkfun.com/tutorials/txb0104-level-shifter-hookup-guide

## Sparkfun QWIIC / I2C relay module
COM-15093
https://www.sparkfun.com/products/15093

Works with Sparkfun QWIIC Relay library  (arduino auto library manager)
https://learn.sparkfun.com/tutorials/qwiic-single-relay-hookup-guide

## Buttons:
https://www.adafruit.com/product/3432
the bottom of the button has arrows for + and - on the LED connections.

## QWIIC/STEMMA Button interface:
https://www.adafruit.com/product/5296
driver in library manager is 'adafruit seesaw'
example in 'LEDArcade_1x4'
dependence is 'Adafruit BUSIO'

## Linear Actuator
[Yuelianart 8" Stroke Tracker Actuator Multi Function Linear Electric Actuator 12v Linear Actuator Stroke Speed 10mm/s 900N Load Capacity](https://www.amazon.com/gp/product/B09PYLGBRK/)
The spec says 2A no load, up to 7A with load. 12V@5A should be sufficient for most uses, but 7A might be needed for the highest loads.

## Motor driver:
HiLetgo BTS7960
https://www.amazon.com/HiLetgo-BTS7960-Driver-Arduino-Current/dp/B00WSN98DC/

Some diagrams and code 
https://www.youtube.com/watch?v=QvXa6GKLu6E

motor controller example
https://create.arduino.cc/projecthub/Fouad_Roboticist/dc-motors-control-using-arduino-pwm-with-l298n-h-bridge-d6ec91

Another example and library
https://robojax.com/learn/arduino/?vid=robojax_BTS7960_motor_driver#google_vignette

# Wiring pinout
See [here](https://docs.google.com/spreadsheets/d/1ZAK95VV2u1BcN2Gdv8SZK7Fb-S8OxHLf3jOhQMFysT0/edit?usp=sharing)

# Cables needed:
 outside+operator button 6m,     6c 20ga:    
 motor 6m                 6c 18ga, tripled
 inside button  3m    4c 18ga
 lights 6m                  3c 18ga
 audio footswitch cable 3m    3c 18ga
 
 |   |   |   |
|---|---|---|
| outside+operator button  |  6m | 6c 20ga  |
| motor  | 6m  | 6c 18ga, tripled  |
| inside button  | 3m  | 4c 18ga  |
| lights  | 6m  | 3c 18ga  |
| audio footswitch | 3m | 3c 18ga |
