// Media Controller software
// Jon Becker 2022
// For Thembi Soddell's audio installation

// Master definitions



#define START_DELAY_SEC 26.0
#define DARKNESS_SEC 6.0

#define MEDIA_DURATION_SEC 592.0
#define END_DELAY_SEC 30.0



#define NEOPIXEL_MASTER_BRIGHTNESS 255
#define NEOPIXEL1_COUNT 5

// Neopixel color. Each channel is 0 to 255.
#define NEOPIXEL_RED 255
#define NEOPIXEL_GREEN 0
#define NEOPIXEL_BLUE 0
#define NEOPIXEL_WHITE 0

#define MOTOR_SPEED 255 // Speed of door motor (0-255). Probably want to leave this at 255. Even at full speed it's pretty slow. If it is too slow the actuator will not move.

// Definitions for Arcade button+LED controller 1x4, from LEDArcade_1x4
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define  DEFAULT_I2C_ADDR 0x3A

#define  OUTSIDE_BUTTON  18  // PA01
#define  USER_BUTTON  19 // PA02
#define  E_STOP  20 // PA03
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
uint8_t masterbuttonledpwm = 0;  // You could use this to synchronize multiple button glows.
uint8_t buttonledpwm[NBUTTONS + 1];

bool doorOpen = true;
bool audioPlaying = false;
bool workStarted = false;

// Definitions for the relay (used to control audio player)
#include <Wire.h> 
#include "SparkFun_Qwiic_Relay.h"
#define RELAY_ADDR 0x18 // Alternate address 0x19
Qwiic_Relay relay(RELAY_ADDR);

// Definitions for the 4W GRBW Neopixels
#include <Adafruit_NeoPixel.h>
#define NEOPIXEL1_PIN    14
Adafruit_NeoPixel strip1(NEOPIXEL1_COUNT+1, NEOPIXEL1_PIN, NEO_RGBW + NEO_KHZ800);

// Definitions for motor controller
#define MotorPin_D1_L  9 //Rpwm
#define MotorPin_D2_L  10 //Lpwm
#define MotorPin_E_L   11 //pwm enable (connected to two pins on the motor driver)


void setup()
{
  motor_setup();
  Serial.begin(115200);
  Serial.println(F("Media Controller"));

  if (!ss.begin(DEFAULT_I2C_ADDR)){
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
  ss.pinMode(OUTSIDE_BUTTON, INPUT_PULLUP);
  ss.pinMode(USER_BUTTON, INPUT_PULLUP);
  ss.pinMode(E_STOP, INPUT_PULLUP);
  ss.pinMode(SWITCH4, INPUT_PULLUP);
  ss.analogWrite(PWM1, 127);
  ss.analogWrite(PWM2, 127);
  ss.analogWrite(PWM3, 127);
  ss.analogWrite(PWM4, 127);

  buttonledpwm[0] = 0; // unused but leave nothing uninitialized.
  for (int i = 1; i < NBUTTONS + 1; ++i)
    buttonledpwm[i] = 0;

  SetButtonLEDState(0, B_LEDGLOWPULSE_STATE);
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

enum states
{
  START_STATE = 0,
  INACTIVE,
  BEGIN_WORK,
  START_DELAY,
  DARKNESS,
  START_PLAYING,
  PLAYING,
  END_PLAYING,
  END_DELAY,
  END_WORK,
  STOP,
  
};


int state = 0;

unsigned long startDelay;
unsigned long darkDelay;

unsigned long mediatimer;
unsigned long endDelay;

unsigned long outsideBounceTime = 0;
unsigned long userDebounceTime = 0;

float j = 0;



int b1_led_state = 0;

int button_led_state[NBUTTONS + 1];


void loop(){


  
  CheckForUserStop();
  
  StateMachine();
  
  ProcButtonLEDs();
  
} // loop()

void StateMachine(){
  
  switch (state){
    
    case START_STATE:
      SetButtonLEDState(1, B_LEDON_STATE);
      SetButtonLEDState(2, B_LEDON_STATE);
      TurnNeopixelsOn();
      state++;
    break;

    case INACTIVE:
      if (! ss.digitalRead(OUTSIDE_BUTTON) && IsTimerFinished(outsideBounceTime) ){         //open and close door
        outsideBounceTime = SetTimer_sec(0.5);
        toggleDoor();
      }
      
      if (! ss.digitalRead(USER_BUTTON) && IsTimerFinished(userDebounceTime)){              //start work
        userDebounceTime = SetTimer_sec(0.5);
        state++;
      }
    break;

    case BEGIN_WORK:
      SetButtonLEDState(1, B_LEDGLOWPULSE_STATE);
      SetButtonLEDState(2, B_LEDOFF_STATE);
      workStarted = true;
      CloseDoor();
      startDelay = SetTimer_sec(START_DELAY_SEC);

      state++;
      
    break;
    
    case START_DELAY:
      if (IsTimerFinished(startDelay)){ 
        darkDelay = SetTimer_sec(DARKNESS_SEC);
        TurnNeopixelsOff();

        state++; 
        }
      
    break;

    case DARKNESS:
    
          if (IsTimerFinished(darkDelay)){ state++; }

    break;

    case START_PLAYING:
      TurnAudioOn();
      mediatimer = SetTimer_sec(MEDIA_DURATION_SEC);
      state++;  
    break;
  
    case PLAYING:
      if (IsTimerFinished(mediatimer)){ state++; }
    break;

    case END_PLAYING:
        audioPlaying = false;  
        endDelay = SetTimer_sec(END_DELAY_SEC);
        state++;
    break;
    
    case END_DELAY:
      if (IsTimerFinished(endDelay)){ state++; }
    break;

    case END_WORK:
        workStarted = false;
        OpenDoor();      
        TurnNeopixelsOn();
        state = START_STATE;
    break;
    
    case STOP:
      OpenDoor();
      TurnNeopixelsOn();
      TurnAudioOff();
      SetButtonLEDState(1, B_LEDOFF_STATE);
      SetButtonLEDState(2, B_LEDOFF_STATE);
      state = START_STATE;  
    break;

    default:
      state = 0;
    break;
  }
}

void CheckForUserStop(){
  if (! ss.digitalRead(E_STOP)){
    state = STOP;
  }else if (! ss.digitalRead(USER_BUTTON) && IsTimerFinished(userDebounceTime) && workStarted){
    userDebounceTime = SetTimer_sec(0.5);
    state = STOP;
  }
}

void TurnAudioOn(){
  if (!audioPlaying){ 
    ToggleAudioPlayer(); 
    audioPlaying = true;
  }
}

void TurnAudioOff(){
  if (audioPlaying){
    ToggleAudioPlayer();
    audioPlaying = false; 
  }
}

void ToggleAudioPlayer(){
  relay.turnRelayOn();
  delay(100);
  relay.turnRelayOff();

}

void Init_Neopixels(){
  strip1.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip1.show();            // Turn OFF all pixels ASAP
  strip1.setBrightness(NEOPIXEL_MASTER_BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
}

//void SetNeopixelsFade(int fade){
//  strip1.fill(strip1.Color(strip1.gamma8(fade), 0, 0, 0));
//  strip1.show();
//  delay(10);
//}


void TurnNeopixelsOff(){
  for (int i = 1; i < NEOPIXEL1_COUNT; ++i)
    strip1.setPixelColor(i, strip1.Color(0, 0, 0, 0) );       //  Set pixel's color (in RAM)
  strip1.show();
  delay(10);
}



void TurnNeopixelsOn(){
  for (int i = 1; i < NEOPIXEL1_COUNT; ++i)
    strip1.setPixelColor(i, strip1.Color(NEOPIXEL_RED, NEOPIXEL_GREEN, NEOPIXEL_BLUE, NEOPIXEL_WHITE) );      
  strip1.show();
//  delay(10);
}

unsigned long SetTimer_sec(float duration_sec){
  unsigned long endtime = millis() + floor(duration_sec * 1000);
  return endtime;
}

bool IsTimerFinished(unsigned long endtime){
  if (millis() > endtime)
    return true;
  else
    return false;
}

void LED_Delay(){
  delay(12);  // in milliseconds.
}

void ProcButtonLEDs(){
  ProcButtonLED(0);  // neopixel brightness
  ProcButtonLED(1);
  ProcButtonLED(2);
  LED_Delay();  // call after updating all button LEDs
}

void toggleDoor(){
  if (doorOpen){
    CloseDoor();

  }else{
    OpenDoor();
  }

}

void CloseDoor(){
  doorOpen = false;
  motor_cw();
  Serial.println("Close Door");
}

void OpenDoor(){
  doorOpen = true;
  motor_ccw();
  Serial.println("Open Door");
}

void motor_setup(){
  pinMode(MotorPin_D1_L, OUTPUT);
  pinMode(MotorPin_D2_L, OUTPUT);
  pinMode(MotorPin_E_L, OUTPUT);
  motor_stop();
}

void motor_test(){
  motor_stop();   delay(1000);
  motor_cw();     delay(1000);
  motor_stop();   delay(1000);
  motor_ccw();     delay(1000);
}

void motor_cw(){
  analogWrite(MotorPin_E_L, MOTOR_SPEED); //0-255
  digitalWrite(MotorPin_D1_L, HIGH);
  digitalWrite(MotorPin_D2_L, LOW);
}

void motor_ccw(){
  analogWrite(MotorPin_E_L, MOTOR_SPEED); //0-255
  digitalWrite(MotorPin_D1_L, LOW);
  digitalWrite(MotorPin_D2_L, HIGH);
}

void motor_stop(){
  digitalWrite(MotorPin_D1_L, LOW);
  digitalWrite(MotorPin_D2_L, LOW);
  analogWrite(MotorPin_E_L, 0);
  }




void SetButtonLEDState(int button, int state){
  button_led_state[button] = state;
}


void SetButtonLED_PWM(int button, uint8_t pwm){
  // Change the LED brightness for a particular button. Normally called by the button led state machine.

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
#define NCYCLES_TO_FLASH 10 // Define number of cycles to flash, each cycle lasts for whatever the delay is (probably 12 msec). So NCYCLES_TO_FLASH = 10 would be 120 msec for each on/off time in this example.




void ProcButtonLED(int button){
  // Process the LED behavior for a particular button.

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
