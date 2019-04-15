#include <arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>



#ifndef cbi     // the following is needed to make sure the SLEEP system works.
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define powerPin 9 // this is the pin that will be used to turn on and off the light. It is ACTIVE LOW
#define brightnessPin 10 // this is the Hall effect sensor that will be used to manage the intensity of the light. It is ACTIVE LOW
#define ledPin 5  // this is the pin the LEDs will be driven from. this is a PWM output for dimming control
#define redPin 2  // this it the PWM that will help display the battery power level
#define greenPin 3  // this it the PWM that will help display the battery power level
#define lowIntencity 0 // this is a simple LED to help display the birghtness level
#define highIntencity 1 // this is a simple LED to help display the birghtness level
#define hallPower 8  // this output provides power to the hall efect sensors or power and level control.

int powerLastState = 0; // this is to check that is happening on the input pin
int brightnessLastState = 0; // this is used to stop state cycling. You need to let go of the button to move to the next state.

int PowerState = 0;     // this is used to determin if the power is on or off.
// this is needed because we are using PWM for the output and we can not just read the output pin value.
// a value of zero is off
// a value of one is on

int BrightnessValue = 1;    // this is to keep track of the mode the leds are in
// BrightnessValue 3 is full intensity
// BrightnessValue 2 is 60% intensity
// BrightnessValue 1 is 20% intensity

long timeCheck; // this is used to check the power lever of the battery every second and update the LED to indicate the power level
long powerTime; // used to figure out when to power down.

long sleepTimer = 600000; // how long to wait before going to sleep (in milliseconds)


void setup() {

  setupPins();
  
  delay(10); // Wait for the hall effect sensors to settle. This is way longer than is really needed.
  timeCheck = millis();
  powerTime = millis();
  
}

void loop() {

  if (timeCheck + 5000 < millis()) {  // every five seconds, check the battery voltage and adjust display LED to show voltage
    timeCheck = millis();
    readVcc();
  }

  if (powerTime + sleepTimer < millis() && PowerState == 0) { // if nothing has happened for ten minutes and the light is off. put the system to sleep
    while(digitalRead(powerPin)) {  // wake up every two seconds and see if the power button has been pressed.
        system_sleep();   // if the power button is not pressed, go back to sleep.
     }
                          // if it was pressed reset pins states and turn the light on.
    powerTime = millis();// rest timers 
    timeCheck = millis();
    PowerState = 1;
    powerLastState = 1;
    blink();
    ChangeState();
  }

  if (!digitalRead(brightnessPin) && !brightnessLastState && PowerState) {  // This is the rutine that handles the brightness "button" being pushed.
    brightnessLastState = 1;  // change the tracking variable of the brightbess button pressing
    powerTime = millis();

    BrightnessValue ++;        //  increment the BrightnessValue variable
    if (BrightnessValue > 3) {    // check to see if the BrightnessValue variable is outside our range
      BrightnessValue = 1;        // reset it if it is
    }
  
    if (PowerState) {  // if PowerSate variable is 1 then change the output intensity.
      ChangeState();
    }

    delay(300); // to make sure there is no "bounce" in the button pressing we will wait 1/10th of a second after any events
    // ******* this should not be required since the hall effect sensor is a digital devie and not a button
    // ******* this should however not hurt.
  }


  if (!digitalRead(powerPin) && !powerLastState) { // This rutine handles what to do wth the power "button" is pressed
    powerLastState = 1;       // take note of the button press

    powerTime = millis();

    if (!PowerState) {  // if PowerSate valiable is 0 then and the button was pushed, turn it on and change the powerstate variable
      PowerState = 1;

      ChangeState();
      readVcc();
    } else {          // if powerstate is on, turn it off and reset variable.
      PowerState = 0;
      analogWrite(ledPin, 0);
      digitalWrite(lowIntencity, LOW);
      digitalWrite(highIntencity, LOW);
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
    }

    delay(300); // to make sure there is no "bouce" in the button pressing we will wait 1/10th of a second after any events
    // ******* this should not be required since the hall effect sensor is a digital devie and not a button
    // ******* this should however not hurt.
  }

  if (digitalRead(powerPin) && powerLastState ) {
    powerLastState = 0;        // take note of the new state of the Power pin
  }

  if (digitalRead(brightnessPin) && brightnessLastState ) {
    brightnessLastState = 0;         // take note of the new state of the Brightness pin
  }

}

void ChangeState() {
  switch (BrightnessValue) {    // check the BrightnessValue variable and react to its value
    case 1:
      analogWrite(ledPin, 51);   // set the output to 20%
      digitalWrite(lowIntencity, HIGH);
      digitalWrite(highIntencity, LOW);

      break;
    
    case 2:
      analogWrite(ledPin, 153);   // set the output to 60%
      digitalWrite(lowIntencity, LOW);
      digitalWrite(highIntencity, HIGH);

      break;

    case 3:
      analogWrite(ledPin, 255);   // set the output to 100%
      digitalWrite(lowIntencity, HIGH);
      digitalWrite(highIntencity, HIGH);

      break;

    default:
      analogWrite(ledPin, 255);  // if the BrightnessValue variable is zero turn off the led
      BrightnessValue = 1;
      digitalWrite(lowIntencity, HIGH);
      digitalWrite(highIntencity, HIGH);

      break;
  }
}


// system wakes up when wtchdog is timed out
void system_sleep() {

  sleepPins();
  
  setup_watchdog(7);      // set the watchdog timer to two seconds.
  cbi(ADCSRA, ADEN);                   // switch Analog to Digital converter OFF
  // sleepPins(); // put all pins in input pullup mode to save power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out
  wdt_disable();

  sbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter ON

  pinMode(powerPin, INPUT);  // set the touch controll input pin to input and turn on the internal pull up resistor
  pinMode(hallPower, OUTPUT);
  digitalWrite(hallPower, HIGH);
  delay(5);

  setupPins();
  delay(5);
  
}


// ******************************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;

  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}


void sleepPins() {
  pinMode(powerPin, OUTPUT);  // set all pins to output and go low to save as much power as possible.
  digitalWrite(powerPin, LOW);
  pinMode(brightnessPin, OUTPUT);  
  digitalWrite(powerPin, LOW);
  pinMode(ledPin, OUTPUT);         
  digitalWrite(ledPin, LOW);
  pinMode(redPin, OUTPUT);
  digitalWrite(redPin, LOW);
  pinMode(greenPin, OUTPUT);
  digitalWrite(greenPin, LOW);
  pinMode(lowIntencity, OUTPUT);
  digitalWrite(lowIntencity, LOW);
  pinMode(highIntencity, OUTPUT);
  digitalWrite(highIntencity, LOW);
  pinMode(hallPower, OUTPUT);
  digitalWrite(hallPower, LOW);
}

ISR(WDT_vect) {
  // Don't do anything here but we must include this
  // block of code otherwise the interrupt calls an
  // uninitialized interrupt handler.
}


void blink() {
  for (int i=0; i<3; i++) {
    digitalWrite(lowIntencity, HIGH);
    delay(50);
    digitalWrite(lowIntencity, LOW);
    delay(50);
    
  }
}

void setupPins() {
  pinMode(powerPin, INPUT);  // set the state of all the pins needed to operate
  pinMode(brightnessPin, INPUT_PULLUP);  
  pinMode(ledPin, OUTPUT);          
  digitalWrite(ledPin, LOW);
  pinMode(redPin, OUTPUT);
  analogWrite(redPin, 0);
  pinMode(greenPin, OUTPUT);
  analogWrite(greenPin, 0);
  pinMode(lowIntencity, OUTPUT);
  digitalWrite(lowIntencity, LOW);
  pinMode(highIntencity, OUTPUT);
  digitalWrite(highIntencity, LOW);
  pinMode(hallPower, OUTPUT);
  digitalWrite(hallPower, HIGH);
 
}

void readVcc() {  // this function is used to get the current supply voltage and changes the color of the meter led based on the battery voltage

  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = 1211861L / result - 3200;

  int Full = 1200;    // this is the value of a full battery in mV
  int Empty = 0;    //this is the value of an empty battery

  float batteryPercent = float(result) / Full * 100; // this is used to find the battery power expressed as a percent

  int green = batteryPercent;  // set the green value to the battery percentage

  if (green < 50 ) {
    int green = batteryPercent - 10;  // if the battery percent is below 50% reduce green by 10 percent to make sure red is red as the voltage goes down
  }

  if (green < 0) {  // since we reduce greens value by 10 percent we need to make sure there is no negative values
    green = 0;
  }
  int red = 100 - batteryPercent;
  if (PowerState) {
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
  }
}
