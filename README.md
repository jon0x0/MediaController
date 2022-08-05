# MediaController

Design and programming by Jon Becker

This is a media controller to support sound art installations.

Goals are:
###  An external 'outside' button to open the door.
###  An internal 'user' button to indicate the user is in place and ready to begin, and to abort the experience if the user is uncomfortable continuing.
###  An operator button to assist with aborting the media.
###  LEDs on the buttons to indicate functions and status.
###  Bright internal RGBW LEDs (here implemented by Adafruit 4W Dotstars) which can be changed to a different color and brightness and controlled as desired.
###  A linear actuator to open and close the door.
###  A relay to control the media (on to reset and play, off to stop playing).

The system is implemented with an Adafruit Feather M4 Express microcontroller, and programmed in C using the Arduino IDE.
It was developed on Arduino IDE 1.8.19.
# Libraries:
To install, the following libraries are needed (they can be installed through the Arduino Library Manager).
## 'Sparkfun QWIIC Relay' library
## 'Adafruit BUSIO' library
## 'Adafruit seesaw' library (for the button controller)

# Hardware used:

## [Adafruit Feather M4 Express](https://www.adafruit.com/product/3857)

## [Featherwing Breakout](https://www.adafruit.com/product/2926)

## [Short headers](https://www.adafruit.com/product/3002) and [stacking headers](https://www.adafruit.com/product/2830)

## [Adafruit 4W RGBW pixels](https://www.adafruit.com/product/5408)


## Level shifter
This [level shifter](https://learn.sparkfun.com/tutorials/txb0104-level-shifter-hookup-guide) is used to shift interface levels from 3.3V of the M4 Express board to the 5V level required by the Dotstar LEDs.


## Sparkfun QWIIC / I2C relay module
[COM-15093 relay module](https://www.sparkfun.com/products/15093)


Works with Sparkfun QWIIC Relay library  (arduino auto library manager)
https://learn.sparkfun.com/tutorials/qwiic-single-relay-hookup-guide

## Buttons:
[Mini Arcade Buttons with integrated LED](https://www.adafruit.com/product/3432)

the bottom of the button has arrows for + and - on the LED connections.

## [Spade to JST-XH connectors](https://www.adafruit.com/product/1152)

## [QWIIC/STEMMA Button interface](https://www.adafruit.com/product/5296)

This is a button interface as a well as an LED controller for buttons with integrated LEDs.
driver in library manager is 'adafruit seesaw'
example in 'LEDArcade_1x4'
dependence is 'Adafruit BUSIO'

## [QWIIC breakout board](https://www.sparkfun.com/products/16790)


## Linear Actuator
[Yuelianart 8" Stroke Tracker Actuator Multi Function Linear Electric Actuator 12v Linear Actuator Stroke Speed 10mm/s 900N Load Capacity](https://www.amazon.com/gp/product/B09PYLGBRK/)
The spec says 2A no load, up to 7A with load. 12V@5A should be sufficient for most uses, but 7A might be needed for the highest loads. 

Note that this actuator has built in limit switches that will stop it at each end of the actuation.

## Motor driver:
[HiLetgo BTS7960](https://www.amazon.com/HiLetgo-BTS7960-Driver-Arduino-Current/dp/B00WSN98DC/)


Some diagrams and code 
https://www.youtube.com/watch?v=QvXa6GKLu6E

motor controller example
https://create.arduino.cc/projecthub/Fouad_Roboticist/dc-motors-control-using-arduino-pwm-with-l298n-h-bridge-d6ec91

Another example and library
https://robojax.com/learn/arduino/?vid=robojax_BTS7960_motor_driver#google_vignette

## Misc JST-SM wire to wire [connectors](https://www.adafruit.com/product/1663) (2,3,4,6 pins)

# Wiring pinout
See [here](https://docs.google.com/spreadsheets/d/1ZAK95VV2u1BcN2Gdv8SZK7Fb-S8OxHLf3jOhQMFysT0/edit?usp=sharing)

# Cables needed:
 These cables may be overkill, but the choice was influenced by how long they needed to be (which makes low resistance desirable), current capacity required (in some cases) and what cable was readily available.
 Cable was purchased from [this](https://www.amazon.com/gp/product/B07Y34F1ZV/) cable family.
 
 | Cable  | Length  | Spec  |
|---|---|---|
| outside+operator button  |  6m | 6c 20ga  |
| motor  | 6m  | 6c 18ga, tripled  |
| inside button  | 3m  | 4c 18ga  |
| lights  | 6m  | 3c 18ga  |
| audio footswitch | 3m | 3c 18ga |

# Operational notes:

When I tried to make an extension cable for the 4W Dotstar chain I couldn't get it working. I tried to build a shorter one which also didn't work. Then I remembered that the Dotstars kind of boost the signal at each stage, so I tried inserting the extension after the first Dotstar, and that worked (with either the 6m or 3m version).  I included both extensions I made. At the moment there are 5 total Dotstars to work with so you could even completely cover up the first one if you want and you still have a decent number to work with.

Probably I could have tried to change the resistor which is in line with the Dotstar data line to increase the drive strength- currently 330 ohm per Adafruit's recommendation of 300-500 ohms (reducing this might have helped). Anyway it takes time to figure those kinds of things out.

The main thing you're going to have to be careful with is to not reverse the 5V and 12V power, that would be disastrous. They are pretty well marked but it would definitely be possible to switch them.

The other thing is that you will still need a MicroSD cable and a USB wall supply to power the processor (Adafruit Feather M4 Express).

I would recommend this powerup order: 5V, then USB power to the processor, then 12V. And the reverse to power down (or all at once with a power strip).

The enclosure was designed (with Fusion 360) so you can mount it to a surface and then screw the cover on top of it (it uses [M3 heat-set threaded inserts](https://www.mcmaster.com/94180A331/)). Also you can route cables either out of a slot on one edge of the cover or you can route cables through a slot in the base (it may not be possible but it's there if useful).
