// Media Controller software
// Jon Becker 2022
// For Thembi Soddell's audio installation

// Master definitions

// Time from when user pushes button to when media starts (includes closing door).
#define BEGIN_CLOSING_DOOR_TO_MEDIA_START_SEC 20.0

// Total duration of media play before lights come on in floating point seconds. This should match the media play possibly plus a few seconds.
#define MEDIA_DURATION_SEC (12.0*60)

// Brightness of neopixel LEDs, 0-255 (255 is brightest).
#define NEOPIXEL_MASTER_BRIGHTNESS 255

// Number of Neopixels in the system.
#define NEOPIXEL1_COUNT 5

// Neopixel color. Each channel is 0 to 255.
#define NEOPIXEL_RED 255
#define NEOPIXEL_GREEN 0
#define NEOPIXEL_BLUE 0
#define NEOPIXEL_WHITE 50

// Speed of door motor (0-255). Probably want to leave this at 255. Even at full speed it's pretty slow. If it is too slow the actuator will not move.
#define MOTOR_SPEED 255

// Include Arduino Wire library for I2C

// Definitions for Arcade button+LED controller 1x4, from LEDArcade_1x4
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define  DEFAULT_I2C_ADDR 0x3A

#define  SWITCH1  18  // PA01
#define  SWITCH2  19 // PA02
#define  SWITCH3  20 // PA03
#define  SWITCH4  2 // PA06
#define  PWM1  12  // PC00
#define  PWM2  13 // PC01
#define  PWM3  0 // PA04
#define  PWM4  1 // PA05


Adafruit_seesaw ss;

enum buttonledstates
{
  B_LEDON_STATE = 0,
  B_LEDON_STATE2,
  B_LEDOFF_STATE,
  B_LEDOFF_STATE2,
  B_LEDONLOW_STATE,
  B_LEDONLOW_STATE2,
  B_LEDGLOW_STATE,
  B_LEDGLOWPULSE_STATE,
  B_LEDGLOWPULSE_STATE2,
  B_LEDFLASH_STATE,
  B_LEDFLASH_STATE2,
  B_LEDFLASH_STATE3,
  B_LEDFADEIN_STATE,
  B_LEDFADEIN_STATE2
};

// Define the number of buttons in the system.
#define NBUTTONS 3

// Pushbutton LED glow pulse width modulation
uint8_t masterbuttonledpwm = 0;  // You could use this to synchronize multiple button glows.
// Button led pwm fro multiple buttons;
uint8_t buttonledpwm[NBUTTONS + 1];


// Definitions for the relay (used to control audio player)
#include <Wire.h>
#include "SparkFun_Qwiic_Relay.h"
#define RELAY_ADDR 0x18 // Alternate address 0x19
Qwiic_Relay relay(RELAY_ADDR);

// Definitions for the 4W GRBW Neopixels
#include <Adafruit_NeoPixel.h>
// Which pin on the Arduino is connected to the NeoPixels?
#define NEOPIXEL1_PIN    14

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip1(NEOPIXEL1_COUNT+1, NEOPIXEL1_PIN, NEO_RGBW + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


// Definitions for motor controller
#define MotorPin_D1_L  9 //Rpwm
#define MotorPin_D2_L  10 //Lpwm
#define MotorPin_E_L   11 //pwm enable (connected to two pins on the motor driver)


void setup()
{
  motor_setup();

  // put your setup code here, to run once:
  Serial.begin(115200);

//  while (!Serial) delay(10);   // wait until serial port is opened

  Serial.println(F("Media Controller"));

  // Initialize arcade button interface
  if (!ss.begin(DEFAULT_I2C_ADDR))
  {
    Serial.println(F("seesaw button controller not found!"));
    while (1) delay(10);
  }

  uint16_t pid;
  uint8_t year, mon, day;

  ss.getProdDatecode(&pid, &year, &mon, &day);
  Serial.print("seesaw button controller found PID: ");
  Serial.print(pid);
  Serial.print(" datecode: ");
  Serial.print(2000 + year); Serial.print("/");
  Serial.print(mon); Serial.print("/");
  Serial.println(day);

  if (pid != 5296)
  {
    Serial.println(F("Wrong seesaw button controller PID"));
    while (1) delay(10);
  }

  Serial.println(F("seesaw started OK!"));
  ss.pinMode(SWITCH1, INPUT_PULLUP);
  ss.pinMode(SWITCH2, INPUT_PULLUP);
  ss.pinMode(SWITCH3, INPUT_PULLUP);
  ss.pinMode(SWITCH4, INPUT_PULLUP);
  ss.analogWrite(PWM1, 127);
  ss.analogWrite(PWM2, 127);
  ss.analogWrite(PWM3, 127);
  ss.analogWrite(PWM4, 127);

  buttonledpwm[0] = 0; // unused but leave nothing uninitialized.
  for (int i = 1; i < NBUTTONS + 1; ++i)
    buttonledpwm[i] = 0;

  SetButtonLEDState(0, B_LEDON_STATE2);
  SetButtonLEDState(1, B_LEDGLOWPULSE_STATE);
  SetButtonLEDState(2, B_LEDGLOWPULSE_STATE);

  // Init the audio player relay
  Wire.begin();

  // Let's see
  if (!relay.begin())
    Serial.println("Check connections to Qwiic Relay.");
  else
    Serial.println("Relay control initialized.");

  Init_Neopixels();

} //setup()



void loop()
{
  // put your main code here, to run repeatedly:

  ProcStateMachine();

} // loop()

int state = 0;

enum states
{
  START_STATE = 0,
  ATTRACT_STATE,
  OPEN_DOOR_STATE,
  OPEN_DOOR_STATE2,
  WAIT_FOR_MEDIA_BUTTON_STATE,
  CLOSE_DOOR_STATE,
  PLAYING_MEDIA_STATE,
  PLAYING_MEDIA_STATE2
};

unsigned long statetimer;
unsigned long mediatimer;
unsigned long operatordebouncetimer;
unsigned long debouncetimer;

void ProcStateMachine()
{
  switch (state)
  {
    case START_STATE: // starting state. Use for some setup / going back to the beginning.
      // Nothing here at the moment
      TurnOffAudioPlayer();
      TurnNeopixelsOn();
      //SetButtonLEDState(0,B_LEDFADEIN_STATE);
      SetButtonLEDState(1, B_LEDGLOWPULSE_STATE);
      SetButtonLEDState(2, B_LEDOFF_STATE);
      operatordebouncetimer = SetTimer_sec(5.0);

      state++;
      Serial.println(F("Start state complete. Moving to attract state."));
      break;

    case ATTRACT_STATE:  // The initial state to glow button and wait for a button push.

      if (! ss.digitalRead(SWITCH1))
      {
        Serial.println("Switch 1 pressed");
        TurnNeopixelsOn();
        SetButtonLEDState(1, B_LEDFLASH_STATE);
        SetButtonLEDState(2, B_LEDGLOWPULSE_STATE);
        state++;
      }
      if (! ss.digitalRead(SWITCH3) && IsTimerFinished(operatordebouncetimer))    // Operator switch
      {
        Serial.println("Switch 3 pressed");
        Serial.println("Closing door.");
        CloseDoor();
        TurnNeopixelsOff();
        operatordebouncetimer = SetTimer_sec(5.0);
        // remain in this state.
      }
      break;

    case OPEN_DOOR_STATE:
      // actuate the door open
      OpenDoor();
      state++;
      break;

    case OPEN_DOOR_STATE2:
      state++;
      break;

    // waiting for the user to push the button inside to begin the media
    case WAIT_FOR_MEDIA_BUTTON_STATE:
      if (! ss.digitalRead(SWITCH3) )    // Operator switch
      {
        state = START_STATE; // reset back to the beginning.
      }

      if (! ss.digitalRead(SWITCH2))
      {
        Serial.println("Switch 2 pressed");
        CloseDoor();
        TurnNeopixelsOff();
        SetButtonLEDState(1, B_LEDFLASH_STATE);
        SetButtonLEDState(2, B_LEDONLOW_STATE); // turn on led steady and dim so it isn't a distraction in the dark. May want to dim it further or turn it off altogether (but nice to have a way to find it in the dark).
        statetimer = SetTimer_sec(BEGIN_CLOSING_DOOR_TO_MEDIA_START_SEC);
        debouncetimer = SetTimer_sec(3.0);
        state++;
      }
      break;

    case CLOSE_DOOR_STATE:
      // wait for a time past button release before accepting an abort
      if (IsTimerFinished(debouncetimer))  // allow an abort with the operator switch during this period after a short debounce period.
      {
        if (! ss.digitalRead(SWITCH3))    // Operator switch
        {
          Serial.println("Switch 3 pressed");
          Serial.println("Opening door.");
          OpenDoor();
          state = START_STATE;  // this will turn off the audio player and turn on the neopixels.
        }
      }
      if (IsTimerFinished(statetimer) == true)
        state++;
      break;

    case PLAYING
      break;

    case PLAYING_MEDIA_STATE2:
      if (! ss.digitalRead(SWITCH3))    // Operator switch
      {
        Serial.println("Switch 3 pressed");
        Serial.println("Opening door.");
        OpenDoor();
        state = START_STATE;  // this will turn off the audio player and turn on the neopixels.
      }

      if (IsTimerFinished(mediatimer) == true)
      {
        Serial.println("Media timer complete");
        OpenDoor();
        state = START_STATE;  // this will turn off the audio player and turn on the neopixels.

      }

      if (! ss.digitalRead(SWITCH2))   // If abort button is pressed
      {
        Serial.println("Switch 2 pressed- abort media");
        Serial.println("Open Door");
        OpenDoor();

        // Tell the neopixels to fade in.
        SetButtonLEDState(0, B_LEDFADEIN_STATE);

        //        TurnOffAudioPlayer();
        //        SetNeopixelsOn();
        state = START_STATE;  // this will turn off the audio player and turn on the neopixels.
      }
      break;


    default:
      state = 0;
      break;

  } //switch

  ProcButtonLEDs();  // These are always processed. May as well do it here.

} //ProcStateMachine


int b1_led_state = 0;

int button_led_state[NBUTTONS + 1];

void SetButtonLEDState(int button, int state)
// Call to change the button LED to a particular mode (off/glow/flash etc).
{
  button_led_state[button] = state;
}

void SetButtonLED_PWM(int button, uint8_t pwm)
// Change the LED brightness for a particular button. Normally called by the button led state machine.
{
  switch (button)
  {
    case 0:  // This is actually the neopixel chain.
      //strip1.setBrightness(NEOPIXEL_MASTER_BRIGHTNESS*pwm/255);
      //SetNeopixelsFade(pwm);
      break;

    case 1:
      ss.analogWrite(PWM1, pwm);  // turn off button led
      break;

    case 2:
      ss.analogWrite(PWM2, pwm);  // turn off button led
      break;

    default:
      break;

  } // switch button

} // SetButtonLED_PWM

int flashcount = 0;
// Define number of cycles to flash, each cycle lasts for whatever the delay is (probably 12 msec). So NCYCLES_TO_FLASH = 10 would be 120 msec for each on/off time in this example.
#define NCYCLES_TO_FLASH 10

void ProcButtonLED(int button)
// Process the LED behavior for a particular button.
{
  int state, ledpwm;
  state = button_led_state[button];
  ledpwm = buttonledpwm[button];

  if (1) // disabled check
  {
    switch (state)
    {

      case B_LEDON_STATE:
        SetButtonLED_PWM(button, 255); // turn on button led
        state++;
        break;

      case B_LEDON_STATE2:
        // led is on now, do nothing
        break;


      case B_LEDOFF_STATE:
        SetButtonLED_PWM(button, 0); // turn off button led
        state++;
        break;

      case B_LEDOFF_STATE2:
        // led is off now, do nothing
        break;

      case B_LEDONLOW_STATE:
        SetButtonLED_PWM(button, 5); // turn on button led, low brightness
        state++;
        break;

      case B_LEDONLOW_STATE2:
        // led is on now, do nothing
        break;

      case B_LEDGLOW_STATE:
        SetButtonLED_PWM(button, ledpwm);
        ledpwm += 5; // glow LED by increasing PWM (it wraps around to zero and keeps increasing)
        break;

      case B_LEDGLOWPULSE_STATE:
        SetButtonLED_PWM(button, ledpwm);
        ledpwm += 5;  // increasing
        if (ledpwm == 255)
          state++;
        break;

      case B_LEDGLOWPULSE_STATE2:
        SetButtonLED_PWM(button, ledpwm);
        ledpwm -= 5;  // decreasing
        if (ledpwm == 0)
          state = B_LEDGLOWPULSE_STATE;  // repeat glowpulse cycle
        break;

      case B_LEDFLASH_STATE:
        SetButtonLED_PWM(button, 255); // turn on button LED
        state++;
        break;

      case B_LEDFLASH_STATE2:
        flashcount++;
        if (flashcount > NCYCLES_TO_FLASH)
        {
          flashcount = 0;
          SetButtonLED_PWM(button, 0); // turn off button LED
          state++;
        }

      case B_LEDFLASH_STATE3:
        flashcount++;
        if (flashcount > NCYCLES_TO_FLASH)
        {
          flashcount = 0;
          state = B_LEDFLASH_STATE;  // repeat the flash cycle
        }
        break;

      case B_LEDFADEIN_STATE:
        ledpwm = 0;
        SetButtonLED_PWM(button, ledpwm);
        ledpwm += 1;  // increasing
        if (ledpwm == 255)
          state++;
        break;

      case B_LEDFADEIN_STATE2:
        // led is on now, do nothing
        break;

      default:
        break;

    } // switch b_led_state
  } // if
  button_led_state[button] = state;  // save any state change for this button
  buttonledpwm[button] = ledpwm;     // save modified led pwm value
} //ProcButtonLED


void LED_Delay()
// This is the system delay after processing the smallest state. Should be quick enough for persistence of vision on the LEDs.
// Call once per state, after updating button PWMs.
{
  delay(12);  // in milliseconds.
}


void ProcButtonLEDs()
// Process all button LEDs and add the delay.
{
  //  ProcButtonLED(0);  // neopixel brightness
  ProcButtonLED(1);
  ProcButtonLED(2);
  LED_Delay();  // call after updating all button LEDs
}

void TurnOnAudioPlayer()
{
  Serial.println("Starting audio player");
  relay.turnRelayOn();
}

void TurnOffAudioPlayer()
{
  Serial.println("Turning off audio player");
  relay.turnRelayOff();
}

void Init_Neopixels()
{
  strip1.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip1.show();            // Turn OFF all pixels ASAP
  strip1.setBrightness(NEOPIXEL_MASTER_BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void TurnNeopixelsOff()
{
  for (int i = 0; i < NEOPIXEL1_COUNT; ++i)
    strip1.setPixelColor(i, strip1.Color(0, 0, 0) );       //  Set pixel's color (in RAM)
  strip1.show();
}

void TurnNeopixelsOn()
{
  for (int i = 0; i < NEOPIXEL1_COUNT; ++i)
    strip1.setPixelColor(i, strip1.Color(NEOPIXEL_RED, NEOPIXEL_GREEN, NEOPIXEL_BLUE, NEOPIXEL_WHITE) );       //  Set pixel's color (in RAM)
  strip1.show();
}

void SetNeopixelsFade(int fade)
// Fade neopixel using the color setting and math.
{
  for (int i = 0; i < NEOPIXEL1_COUNT+1; ++i)
    strip1.setPixelColor(i, strip1.Color(NEOPIXEL_RED * fade / 255, NEOPIXEL_GREEN * fade / 255, NEOPIXEL_BLUE * fade / 255) ); //  Set pixel's color (in RAM)
  strip1.show();
}

unsigned long SetTimer_sec(float duration_sec)
{
  unsigned long endtime = millis() + floor(duration_sec * 1000);
  return endtime;
}

bool IsTimerFinished(unsigned long endtime)
{
  if (millis() > endtime)
    return true;
  else
    return false;
}

void CloseDoor()
{
  motor_cw();
  Serial.println("Close Door");
}

void OpenDoor()
{
  motor_ccw();
  Serial.println("Open Door");
}


void motor_setup()
{
  pinMode(MotorPin_D1_L, OUTPUT);
  pinMode(MotorPin_D2_L, OUTPUT);
  pinMode(MotorPin_E_L, OUTPUT);
  motor_stop();
}

void motor_test()
{
  motor_stop();   delay(1000);
  motor_cw();     delay(1000);
  motor_stop();   delay(1000);
  motor_ccw();     delay(1000);
}

void motor_cw()
{
  analogWrite(MotorPin_E_L, MOTOR_SPEED); //0-255
  digitalWrite(MotorPin_D1_L, HIGH);
  digitalWrite(MotorPin_D2_L, LOW);
}

void motor_ccw()
{
  analogWrite(MotorPin_E_L, MOTOR_SPEED); //0-255
  digitalWrite(MotorPin_D1_L, LOW);
  digitalWrite(MotorPin_D2_L, HIGH);
}

void motor_stop()
{
  digitalWrite(MotorPin_D1_L, LOW);
  digitalWrite(MotorPin_D2_L, LOW);
  analogWrite(MotorPin_E_L, 0);
}
