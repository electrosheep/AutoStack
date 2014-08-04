
/*
AutoStack - Alpha code
 
 This is currently test software designed to run on a prototype board and is not for use
 with the current schematics of this project
 
 Note - uses internal clock and PB6 PB7 for data pins so won't work on atmega328 with external oscillator
 - changing limitSwitchEnd and powerSave pin definitions will allow it to work with external osc.
 
 
 */



#include <AccelStepper.h>//http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <SoftwareSerial.h>//for bluetooth
#include <SimpleTimer.h>//http://playground.arduino.cc/Code/SimpleTimer
#include <SPI.h>
#include <PCD8544_SPI.h>//http://forum.arduino.cc/index.php?topic=176794.0
#define INPUT_SIZE 3


//#define USE_FRAME_BUFFER

//#ifdef USE_FRAME_BUFFER
//PCD8544_SPI_FB lcd;
//#else
PCD8544_SPI lcd;
//#endif


const byte switchMode = 2;
const byte switchPinPos = 4;//RHS on breadboard
const byte switchLimitHome = 7;
const byte switchPinNeg = 17;
const byte switchLimitEnd = 20;
const byte powerSave = 21;
const byte bluetoothEnable = A2;


AccelStepper stepper(1, 5, 6);
SimpleTimer timer;
SoftwareSerial Bluetooth(18, 19); // RX, TX
unsigned long prevMillis = 0;
unsigned long stepSize = 100;
unsigned long timeDelay = 1000;//time after stepping before power is disabled to driver

boolean blnEndMove = false;
boolean blnForward = true;
boolean blnPowerSave = true;
boolean blnPosTriggered = false;
boolean blnNegTriggered = false;
boolean blnBluetoothEnable = true;
boolean blnSoftwareSwitchTriggered = false;
byte intMode = 0;
boolean longPress(byte pinID, int longPressTime = 500);//length of time before considered a long press
boolean truePress(int debounceTime = 200);//length of time to consider switch change as a bounce
byte stepTimer = timer.setInterval(3000, goToSleep);


void setup()
{  
  attachInterrupt(0, modeSelect, FALLING);
  Bluetooth.begin(9600); 
  Serial.begin(9600);
  stepper.setMaxSpeed(2400);//set for 1/16 step
  stepper.setAcceleration(3000);//set for 1/16 step
  pinMode(switchMode, INPUT_PULLUP); //interrupt
  pinMode(switchPinPos, INPUT_PULLUP); //push button +
  pinMode(switchPinNeg, INPUT_PULLUP);//push button -
  pinMode(switchLimitHome, INPUT_PULLUP);//micro switch limiter near end
  pinMode(switchLimitEnd, INPUT_PULLUP);//micro switch limiter far end
  pinMode(powerSave, OUTPUT); //power saving (and noise reducing)
  digitalWrite(powerSave, LOW);//HIGH disabled, LOW enabled

  lcd.begin();
  lcd.print("Screen Test");
  moveToLimit('2');
}

void loop()
{
  while (digitalRead(switchLimitHome) == HIGH && digitalRead(switchLimitEnd) == HIGH){
    if (!blnSoftwareSwitchTriggered && (Serial.available() || Bluetooth.available())) softwareSwitchHandler();
    blnPosTriggered = hardwareSwitchHandler(switchPinPos, blnPosTriggered);// RHS on breadboard
    blnNegTriggered = hardwareSwitchHandler(switchPinNeg, blnNegTriggered);
    if (timer.isEnabled(stepTimer)) timer.run();
  }
}


/*   *** handles switch press ***   */
boolean hardwareSwitchHandler(char switchPin, boolean blnTriggered) {
  if (digitalRead(switchPin) == LOW && !blnTriggered) {
    blnTriggered = true;

    char switchSrc = 'h';//source of data - h = hardware
    char switchId;//IDs for physical switches
    char switchEvent;//switch event code

    switchPin == switchPinPos ? switchId = '1' : switchId = '2';//1 = first hardware switch, 2...
    longPress(switchPin, 500) ? switchEvent = 'l' : switchEvent = 'c';//longpress = 'l', normal click = 'c'
    switchChange(switchSrc, switchId, switchEvent);
  }
  else
    if (digitalRead(switchPin) == HIGH && blnTriggered) return false;
  return blnTriggered;
}
/*   ^^^ handles switch press ^^^   */


/*   *** parse serial data ***   */
void softwareSwitchHandler (){
  blnSoftwareSwitchTriggered = true;
  char input[INPUT_SIZE + 1];
  byte size;
  //add some meaning to the input
  char switchSrc;//source of data - b = bluetooth, u = usb
  char switchId;
  char switchEvent;//switch event code

  size = Bluetooth.available() ? Bluetooth.readBytes(input, INPUT_SIZE) : Serial.readBytes(input, INPUT_SIZE);

  // add the final 0 to end the C string
  input[size] = 0;

  // split the serial into values
  switchSrc = input[0];
  switchId = input[1];
  switchEvent = input[2];
  switchId == 'm' ? modeSelect() : switchChange(switchSrc, switchId, switchEvent);
  Bluetooth.println(input);
}
/*   ^^^ parse serial data ^^^   */


/*   *** deal with input from whatever source in a unified way ***   */
void switchChange(char switchSrc, char switchId, char switchEvent) {

  switch (intMode) {

    /*   *** power save/bluetooth toggle ***   */
  case 1: 
    {
      switchId == '1' ? digitalWrite(bluetoothEnable, !digitalRead(bluetoothEnable)) : digitalWrite(powerSave, !digitalRead(powerSave));
      lcd.gotoXY(79,0);
      lcd.print(digitalRead(powerSave));
      lcd.gotoXY(79,1);
      lcd.print(digitalRead(bluetoothEnable));
    }
    break;
    /*   ^^^ power save/bluetooth toggle ^^^   */

    /*   *** carriage movement ***   */
  case 2://movement
    {
      unsigned long localStepSize = stepSize;
      switchEvent == 'l' ? localStepSize = 100000 : localStepSize = stepSize;//1=long press,c=click
      switchId == '1' ? stepper.move(-localStepSize) : stepper.move(localStepSize);//1 = 1st hardware switch, 2 =...
      enableStepperDriver();
      if (localStepSize == 100000) {
        if (switchSrc == 'b') {//it's a bluetooth switch
          while (Bluetooth.read()!= 'u') {
            stepper.run();
          } //block until button released or passes minimum long press threshold - nothing else should be happening
        }
        else {
          while (digitalRead(switchId == '1' ? 4 : 17) == LOW) {
            stepper.run();
          }
        }
        stepper.runToNewPosition(switchId == '1' ? stepper.currentPosition() - 1000 : stepper.currentPosition() + 1000);//decelerate after move
      }
      else {
        stepper.runToNewPosition(switchId == '1' ? stepper.currentPosition() - stepSize : stepper.currentPosition() + stepSize);//decelerate after move
      }
      if (blnPowerSave) disableStepperDriver();
    }
    break;
    /*   ^^^ carriage movement ^^^   */

    /*   *** adjust step size ***   */
  case 3:
    {
      switchId == '1' ? stepSize++ : stepSize--;// simple click
      rJustify(stepSize);
      if (switchEvent == 'l') {
        while (Bluetooth.read()!= 'u' || digitalRead(switchId == '1' ? 4 : 17) == LOW) {
          switchId == '1' ? stepSize++ : stepSize--; //longpress in loop increments like the clappers until released
          delay(50);//slow it down a wee bit
          rJustify(stepSize);
        } //block until button released - nothing else should be happening
      }
    }
    break;
    /*   ^^^ adjust step size ^^^   */

    /*   *** move to home/end ***   */
  case 4:
    {
      moveToLimit(switchId);
      //    lcd.gotoXY(79,0);
      //    lcd.print(digitalRead(powerSave));
      //    lcd.gotoXY(79,1);
      //    lcd.print(digitalRead(bluetoothEnable));
    }
    break;
    /*   ^^^ move to home/end ^^^   */

  }
  blnSoftwareSwitchTriggered = false;
}
/*   ^^^ deal with input from whatever source in a unified way ^^^   */


/*   *** right justify numbers ***   */
// currently prints fixed length at fixed location
void rJustify(long number){
  char buffer [sizeof(long)*8+1];
  char pos[6];
  ltoa(number,buffer,10);
  sprintf(pos, "%05s", buffer);
  //sprintf(pos, "%05d", -stepper.currentPosition());
  lcd.gotoXY(55,0);
  lcd.print(pos);
}
/*   *** right justify numbers ***   */


/*   *** set mode of controller ***   */
void modeSelect() {
  if (truePress()){
    {
      (intMode < 4) ? intMode++ : intMode = 1;

      switch (intMode) {
      case 1: //toggle switches
        lcd.clear();
        lcd.print("Power save:");
        lcd.gotoXY(79,0);
        lcd.print(digitalRead(powerSave));
        lcd.gotoXY(1,1);
        lcd.print("Bluetooth:");
        lcd.gotoXY(79,1);
        lcd.print(digitalRead(bluetoothEnable));
        Bluetooth.println("m1");
        break;
      case 2:
        lcd.clear();
        lcd.print("Move:");
        rJustify(-stepper.currentPosition());
        Bluetooth.println("m2");
        break;
      case 3:
        lcd.clear();
        lcd.print("Store:");
        Bluetooth.println("m3");
        break;
      case 4:
        lcd.clear();
        lcd.print("Home:");
        Bluetooth.println("m4");
        break;
      }   
    }
  }
  blnSoftwareSwitchTriggered = false;

}
/*   ^^^ set mode of controller ^^^   */


/*   *** discard 'invalid' switch presses ***   */
boolean truePress(int debounceTime) {
  // if interrupts come faster than debounceTime (ms), assume it's a bounce and ignore
  if (millis() - prevMillis > debounceTime) {
    prevMillis = millis();
    return true;
  }
  return false;
}
/*   ^^^ discard 'invalid' switch presses ^^^   */


/*   *** determine if it's a long press ***   */
boolean longPress(char switchId, int longPressTime) {
  unsigned long startMillis = millis();
  while ((digitalRead(switchId) == LOW) && (millis() - startMillis) < longPressTime) {
  } //block until button released or passes minimum long press threshold - nothing else should be happening
  if (millis() - startMillis >= longPressTime) 
    return true;
  return false;
}
/*   ^^^ determine if it's a long press ^^^   */


/*   *** set stepper to limit reset position counter ***   */
void moveToLimit(char switchId) {
  enableStepperDriver();
  switchId == '2' ? stepper.move(100000) : stepper.move(-100000);
  while (digitalRead(switchLimitHome) == HIGH && digitalRead(switchLimitEnd) == HIGH) stepper.run();
  //  while (digitalRead(switchLimitHome) != LOW) stepper.run();
  stepper.move(switchId == '2' ? -600 : 600);
  stepper.runToPosition();
  if (switchId == '2') stepper.setCurrentPosition(0);
  if (blnPowerSave) disableStepperDriver();
  rJustify(-stepper.currentPosition());
  Bluetooth.println(-stepper.currentPosition());
}
/*   ^^^ set stepper to limit and reset position counter ^^^   */


/*   *** start timer which call the actual disable routine ***   */
void disableStepperDriver(){
  timer.restartTimer(stepTimer);
  timer.enable(stepTimer);
}
/*   ^^^ start timer which call the actual disable routine ^^^   */


/*   *** enable stepper driver ***   */
void enableStepperDriver() {
  timer.disable(stepTimer);
  digitalWrite(powerSave, LOW);
}
/*   ^^^ enable stepper driver ^^^   */


/*   *** switch on power saving ***   */
void goToSleep() {
  timer.disable(stepTimer);
  blnPowerSave ? digitalWrite(powerSave, HIGH) : digitalWrite(powerSave, HIGH);
}
/*   ^^^ switch on power saving ^^^   */


void debug(String message){
  lcd.clear();
  lcd.print(message);
}
void debugc(char chr){
  char str[2];
  str[0]=chr;
  str[1]='\0';
  lcd.clear();
  lcd.print(str);
}







