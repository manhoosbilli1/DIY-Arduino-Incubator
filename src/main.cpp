//--------libraries-------------
#include <Arduino.h> //libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include "RTClib.h"
#include <JC_Button.h>
#include <EEPROM.h>
#include "MenuData.h"

#define LCD_ROWS 2
#define LCD_COLS 16

enum AppModeValues
{
  APP_NORMAL_MODE,
  APP_MENU_MODE,
  APP_PROCESS_MENU_CMD
};
byte appMode = APP_NORMAL_MODE;
MenuManager rtMenu(runtimeMenu_Root, menuCount(runtimeMenu_Root));

char strbuf[LCD_COLS + 1]; // one line of lcd display
byte btn;

#define MOTORTYPETIMER
//#define USINHHUMIDIFIER
#define SENSORMODEBOTH
//#define USINGRTC
//#define MOTORTYPELIMIT
//#define HEATERMODERELAY
//#define SENSORMODEDHT
//#define SENSORMODEDSB
//#define USINGWATERPUMP
const uint32_t sensorUpdateInterval = 2000;

/*------------Pin definitions----------------*/
#define motorPinA 11
#define motorPinB 12
#ifdef MOTORTYPELIMIT
#define uint8_t limitSw1 = 10;
#define uint8_t limitSw2 = 9;
#endif
#define ONE_WIRE_BUS 8
#define DHTPIN 7
#define DHTTYPE DHT22
#define heater 6
#define humidifier A0
#define dehumFan A1
const byte pinUp(9), pinDw(4), pinSl(5);
#ifdef USINGWATERPUMP
uint8_t waterLevelSensorPin = A7;
#endif

/*EEPROM ADDRESSED*/
#define tempAdd 0            //float
#define humAdd 4             //int
#define freqAdd 6            //int
#define daysAdd 8            //int.
#define initialSetupAdd 1023 //this bit will check wether its initial setup or not.
/*----------Motor controller variables-----------*/
uint8_t frequency = 8;
uint32_t turnInterval;
uint32_t turnDelay = 5000;
uint32_t lastTime = 0;
bool is_time = false;
#define MOTOR_IDLE 0
#define MOTOR_MOVING_LEFT 1
#define MOTOR_MOVING_RIGHT 2

//----------rest of variables------------
#ifdef USINGWATERPUMP
uint8_t waterMinLimit;
uint8_t waterMaxLimit;
#endif
#ifndef HEATERMODERELAY
double Output;
double Kp = 10;
double Ki = 5;
double Kd = 1.8;
#endif
double tempSetpoint = 28.71;
uint8_t humiditySetpoint = 60;
double currentTemp;
double f;
double t;
double h;
double curTemp;
uint32_t currentTime = 0;
uint32_t prevTime1 = 0;
uint8_t prevCounter;
#define counterMax 5
#define counterMin 0
uint8_t daysToHatch = 21;
uint8_t turnCounter;
uint8_t prevState;
uint8_t pageMax = 5;
uint8_t pageMin = 0;
uint8_t prevPage = 0;
uint8_t page = 0;
bool inMenu = 0;
bool inSubMenu = 0;
bool pressed = 1;
bool update_sensor = 0;
bool refreshFlag = true;
bool refreshFlag1 = true;
#define home 0
#define page1 1
#define page2 2
#define page3 3
#define page4 4
#define ROOT 0
volatile bool upState, dwState, slState;
//------------OBJECT initialization------------
#ifdef SENSORMODEDHT
DHT dht(DHTPIN, DHTTYPE);
#endif
#ifdef SENSORMODEDSB
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
#endif
#ifdef SENSORMODEBOTH
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
#endif
#ifndef HEATERMODEREALY //only if not using relay
PID myPID(&currentTemp, &Output, &tempSetpoint, Kp, Ki, Kd, DIRECT);
#endif
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
#ifdef MOTORTYPELIMIT
Button sw1(limitSw1);
Button sw2(limitSw1);
#endif
Button UP(pinUp); //up
Button DW(pinDw); //down
Button SL(pinSl); //select
#ifdef USINGRTC
RTC_DS3231 rtc;
#endif

/*------------function declarations--------------*/
bool isTime();
void motorHandler();
void updateSwitches();
void updateMenu();
void correction();
void menuHandler();
void readFromEEPROM();
bool turnMotorOnce();
void updateSensor();
void homeMenu();
void poll();
extern char *inttostr(char *dest, short integer);
// Apply left padding to string.
extern char *lpad(char *dest, const char *str, char chr = ' ', unsigned char width = LCD_COLS);
// Apply right padding to string.
extern char *rpad(char *dest, const char *str, char chr = ' ', unsigned char width = LCD_COLS);
// Apply string concatenation. argc = number of string arguments to follow.
extern char *fmt(char *dest, unsigned char argc, ...);
char *padc(char chr, unsigned char count);

byte processRuntimeMenuCommand(byte cmdId);
void refreshMenuDisplay(byte refreshMode);
byte getNavAction();

void setup()
{
  // put your setup code here, to run once:
  //EEPROM.put(tempAdd,tempSetpoint);
  //initial setup check.

#ifdef SENSORMODEDHT
  dht.begin();
#endif
#ifdef SENSORMODEDSB
  sensors.begin();
#endif
#ifdef SENSORMODEBOTH
  dht.begin();
  sensors.begin();
#endif
  lcd.init();
  lcd.backlight();
#ifdef MOTORTYPELIMIT
  sw1.begin();
  sw2.begin(); //create a conditional here if mode is limit only then initialize.
#endif
  DW.begin();
  UP.begin();
  SL.begin();
#ifndef HEATERMODERELAY
//  myPID.SetMode(AUTOMATIC);
//  myPID.SetSampleTime(2300); //2.3 seconds
#endif
#ifdef USINGRTC
  if (!rtc.begin()) //make it print to lcd if this happens
  {
    abort();
  }
#endif

  //---------PINS INITIALIZATION-------------
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(dehumFan, OUTPUT);
#ifdef USINGHUMIDIFIER
  pinMode(humidifier, OUTPUT);
  pinMode(waterLevelSensor, OUTPUT);
#endif
  //should be called after values are read from EEPROM
  //readFromEEPROM();
  //turnInterval = (24 / frequency) * 3600000;
  appMode = APP_NORMAL_MODE;
  refreshMenuDisplay(REFRESH_DESCEND);
}

void loop()
{
  // put your main code here, to run repeatedly:
  currentTime = millis();
  poll();
  updateSensor();
  //correction();
  //motorHandler();
  homeMenu();
  switch (appMode)
  {
  case APP_NORMAL_MODE:
    if (UP.pressedFor(1000))
    {
      lcd.clear();
      appMode = APP_MENU_MODE;
      refreshMenuDisplay(REFRESH_DESCEND);
    }
    break;
  case APP_MENU_MODE:
  {
    byte menuMode = rtMenu.handleNavigation(getNavAction, refreshMenuDisplay);

    if (menuMode == MENU_EXIT)
    {
      appMode = APP_NORMAL_MODE;
    }
    else if (menuMode == MENU_INVOKE_ITEM)
    {
      appMode = APP_PROCESS_MENU_CMD;

      // Indicate selected item.
      if (rtMenu.getCurrentItemCmdId())
      {
        lcd.setCursor(0, 1);
        strbuf[0] = 0b01111110; // forward arrow representing input prompt.
        strbuf[1] = 0;
        lcd.print(strbuf);
      }
    }
    break;
  }
  case APP_PROCESS_MENU_CMD:
  {
    byte processingComplete = processRuntimeMenuCommand(rtMenu.getCurrentItemCmdId());

    if (processingComplete)
    {
      appMode = APP_MENU_MODE;
      // clear forward arrow
      lcd.setCursor(0, 1);
      strbuf[0] = ' '; // clear forward arrow
      strbuf[1] = 0;
      lcd.print(strbuf);
    }
    break;
  }
  }
}

/*
Will update the sensors only
*/
void updateSensor()
{
  //Poles the sensors.
  static bool flag = false;
  if (((currentTime - prevTime1) >= sensorUpdateInterval) || (flag == true))
  {

#ifdef SENSORMODEDHT
    h = dht.readHumidity();
    t = dht.readTemperature();
    currentTemp = t;
    prevTime1 = currentTime; //only the user has chosen this mode of sensors. if user is using only one sensor.
    update_sensor = true;    //equal currtemp to that only.
#endif
#ifdef SENSORMODEDSB
    sensors.requestTemperatures();
    currentTemp = sensors.getTempCByIndex(0);
    prevTime1 = currentTime; //only the user has chosen this mode of sensors. if user is using only one sensor.
    updateSensor = true;     //equal currtemp to that only.
#endif
#ifdef SENSORMODEBOTH
    if (!flag)
    {
      h = dht.readHumidity();
      t = dht.readTemperature(); //if sensor mode is both sensors then it will measure one sensor. the turn on the flag
      flag = true;               //for the other sensor to be measured. once that is measured in the next check. flag
    }                            //will turn off. func wont run until one of the conditions become true.
    else
    {
      sensors.requestTemperatures();
      currentTemp = sensors.getTempCByIndex(0) + t / 2;
      prevTime1 = currentTime; //only the user has chosen this mode of sensors. if user is using only one sensor.
      update_sensor = true;    //equal currtemp to that only.
      flag = false;            //this is used to break the time taken from measurement in half.
    }
#endif
  }
}

//Will apply all kinds of correction from temp to humidity.
void correction()
{
#ifndef HEATERMODERELAY
  //myPID.Compute();         //don't forget to uncomment in testing.
  //analogWrite(heater, Output);
#else
  //means using relay.
  if (Input >= Setpoint)
    digitalWrite(heater, LOW);
  else
    digitalWrite(heater, HIGH);
#endif
  //TODO: ifndef humidifier then don't fire the alarm as soon as it decreases.
  //give it sometime. pleasent and good alarm should be there.

  //Makes the adjustment to humidifier.
  /*
#if defined(SENSORMODEDHT) || defined(SENSORMODEBOTH)

  if (h > humiditySetpoint)
  {
#ifdef USINGHUMIDIFIER
    digitalWrite(humidifier, LOW); //if humidifier is on turn off
#endif
    digitalWrite(dehumFan, HIGH); //if dehumidifier fan already on do nothing else turn on
  }
  else
    digitalWrite(dehumFan, LOW); //turn off dehum fine to retain humidity

#ifdef USINGWATERPUMP
  if (analogRead(waterLevelSensor) < (waterMinLimit))
  {

    while (analogRead(waterLevelSensor) >= waterMaxLimit)
    {
      digitalWrite(waterLevelSensor, HIGH);
    }
    digitalWrite(waterLevelSensor, LOW);
  }
#endif
#ifdef USINGHUMIDIFIER
  switch (humStage)
  {
  case 0: //checks if hum is low.
        if(h < (h_setPoint)
        {
      humTimer = millis(); //start the timer to check if it wasn't noise.
      humStage = 1;        //swithc the case to 1.
      prevHum = h;         //record the hum for future reference 
        }
        else
        {
      digitalWrite(humidifier, LOW); //if h is not lower than set point. turn off the humidifier. ez. 
        }
        break;

      case 1: //makes sure it wasn't just some wind or sensor noise.
        if(((currentTime - humTimer) >= 120000) && (prevHum - h >0)) //if still falling. 
        {
      correctionTimer = millis(); //record the timer for next case.
      prevHum = h;
      humStage = 2; 
        }
        break;

      case 2:                       //do the correction. 
        if(h >= h_setPoint)         // while doing the adjustment. value in now equal to desired humidity. 
        {
      repititions = 0;
      humStage = 0; //reset the case to 0. 
        }
        if(((currentTime - correctionTimer) <= 120000) || repititions != 0)   //run the correction program for 2 minutes. 
        {
      digitalWrite(humidifier, HIGH); 
        }                                       //if this flag is true. it will bypass the time calculation and just adjust
        else if((prevHum - h) < 0)              //if difference is negetive. meaning humidity is restoring.
          {
      repititions = 0;
      humStage = 0; //switch to initial stage. 
          }
          else
          {
      //if humidity is not restoring resetting reset the timer. so humidifer can run for 2 minutes more.
      correctionTimer = currentTime; //reset the timer.   
          }
        break;
  }
#endif
#endif
*/
}

/*
Calculates when its time to turn the motors. 
*/

bool isTime()
{
  if ((currentTime - lastTime) >= turnInterval)
  {
    return true;
  }
  return false;
}

/*
when its time, turns the motors. 
this should be run in loop so that it always knows when its time
*/
void motorHandler()
{
  static bool is_time = false;
  static uint8_t currentState = 0;
  if (isTime())
  {
    lastTime = currentTime;
    is_time = true;
  }
  if (is_time) //runs the motor if its time.
  {
//turning the motor.
#ifdef MOTORTYPETIMER
    static uint32_t timer = 0;
    switch (currentState)
    {
    case MOTOR_IDLE:
      if ((prevState != MOTOR_MOVING_LEFT) && (prevState != MOTOR_MOVING_RIGHT)) //then move right
      {
        digitalWrite(motorPinA, HIGH);
        digitalWrite(motorPinB, LOW);     //turn towards any random direction.
        timer = currentTime;              //start the timer
        currentState = MOTOR_MOVING_LEFT; //direction is abstract. doesn't matter
      }
      if (prevState == MOTOR_MOVING_LEFT)
      {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, HIGH);
        timer = currentTime;
        currentState = MOTOR_MOVING_RIGHT;
      }
      if (prevState == MOTOR_MOVING_RIGHT)
      {
        digitalWrite(motorPinA, HIGH);
        digitalWrite(motorPinB, LOW);
        timer = currentTime;
        currentState = MOTOR_MOVING_LEFT;
      }
      break;

    case MOTOR_MOVING_LEFT:
      if ((currentTime - timer) <= turnDelay) //
      {
        //wait for the motor to reach the other extreme position
      }
      else
      {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, LOW);  //turn off the motors since destination reached
        prevState = MOTOR_MOVING_LEFT; //write this to eeprom just to be safe
        turnCounter++;
        is_time = false; //just turned. turn off the flag and wait for it to occur again.
        currentState = MOTOR_IDLE;
      }
      break;

    case MOTOR_MOVING_RIGHT:
      if ((currentTime - timer) <= turnDelay)
      {
        //wait for the motor to reach the other extreme position
      }
      else
      {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, LOW);
        prevState = MOTOR_MOVING_RIGHT;
        turnCounter++; //once done turning the motor turn off the motor toggle variable.
        is_time = false;
        currentState = MOTOR_IDLE;
      }
      break;
    }
  }
#endif
#ifdef MOTORTYPELIMIT
  if (isTime())
  {
    lastTime = currentTime; //reset the timer when we call the isTime function and not before that.
    is_time = true;         //we catch the flag.
  }
  if (is_time)
  {
    switch (currentState)
    {
    case MOTOR_IDLE:
      if ((sw1.read() == false) && (sw2.read() == false)) //tray is in unknown state. needs calibration
      {
        digitalWrite(motorPinA, HIGH);
        digitalWrite(motorPinA, LOW);
        currentState = MOTOR_MOVING_LEFT; //switch state
      }

      if ((sw1.read() == true) && (sw2.read() == false)) //already at left. time to go right
      {
        digitalWrite(motorPinA, HIGH);
        digitalWrite(motorPinB, LOW);
        currentState = MOTOR_MOVING_RIGHT;
      }

      if ((sw1.read() == false) && (sw2.read() == true)) //already at right. time to go left.
      {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, HIGH);
        currentState = MOTOR_MOVING_LEFT;
      }
      break;

    case MOTOR_MOVING_LEFT:
      if ((sw1.read() == true) && (sw2.read() == false)) //means tray is tilted one way.
      {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, LOW);
        is_time = false; //reset the time since tray reached destination
        prevState = MOTOR_MOVING_LEFT;
        turnCounter++;
        currentState = MOTOR_IDLE; //current state = idle when pc reaches switch statement.
      }                            //it should start from idle instead of moving left case.
      break;

    case MOTOR_MOVING_RIGHT:
      if ((sw1.read() == false) && (sw2.read() == true))
      {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, LOW);
        is_time = false;
        prevState = MOTOR_MOVING_RIGHT;
        turnCounter++;
        currentState = MOTOR_IDLE;
      }
      break;
    }
  }
#endif
}

bool turnMotorOnce() //if returns true. turning is done.
{
#ifdef MOTORTYPETIMER
  static uint32_t timer1 = 0;
  static uint8_t currentState1 = 0;
  switch (currentState1)
  {
  case MOTOR_IDLE:
    if ((prevState != MOTOR_MOVING_LEFT) && (prevState != MOTOR_MOVING_RIGHT)) //then move right
    {
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinB, LOW);
      timer1 = currentTime;              //record the time
      currentState1 = MOTOR_MOVING_LEFT; //direction is abstract. doesn't matter
    }

    if (prevState == MOTOR_MOVING_LEFT)
    {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, HIGH);
      timer1 = currentTime;
      currentState1 = MOTOR_MOVING_RIGHT;
    }

    if (prevState == MOTOR_MOVING_RIGHT)
    {
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinB, LOW);
      timer1 = currentTime;
      currentState1 = MOTOR_MOVING_LEFT;
    }
    break;

  case MOTOR_MOVING_LEFT:
    if ((currentTime - timer1) <= turnDelay) //
    {
      //wait for the motor to reach the other extreme position
    }
    else
    {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW); //turn off the motors since destination reached
      currentState1 = MOTOR_IDLE;
      prevState = MOTOR_MOVING_LEFT;
      return true; //break the function when turning is done.
    }
    break;

  case MOTOR_MOVING_RIGHT:
    if ((currentTime - timer1) <= turnDelay)
    {
      //wait for the motor to reach the other extreme position
    }
    else
    {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
      currentState1 = MOTOR_IDLE;
      prevState = MOTOR_MOVING_RIGHT;
      return true; //break the function when turning is done.
    }
    break;
  }
#endif
#ifdef MOTORTYPELIMIT
  static uint8_t currentState1 = 0; //using static so that it remembers its state.
  switch (currentState1)
  {
  case MOTOR_IDLE:
    if ((sw1.read() == false) && (sw2.read() == false)) //tray is in unknown state. needs calibration
    {
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinA, LOW);
      currentState1 = MOTOR_MOVING_LEFT; //switch state
    }

    if ((sw1.read() == true) && (sw2.read() == false)) //already at left. time to go right
    {
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinB, LOW);
      currentState1 = MOTOR_MOVING_RIGHT;
    }

    if ((sw1.read() == false) && (sw2.read() == true)) //already at right. time to go left.
    {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
      currentState1 = MOTOR_IDLE;
    }
    break;

  case STATE_MOVING_LEFT:
    if ((sw1.read() == true) && (sw2.read() == false)) //means tray is tilted one way.
    {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
      currentState1 = MOTOR_IDLE; //current state = idle when pc reaches switch statement.
      prevState = MOTOR_MOVING_LEFT;
      return true; //break the function when turning is done.
    }              //it should start from idle instead of moving left case.
    break;

  case STATE_MOVING_RIGHT:
    if ((sw1.read() == false) && (sw2.read() == true))
    {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
      currentState1 = MOTOR_IDLE;
      prevState = MOTOR_MOVING_RIGHT;
      return true; //break the function when turning is done.
    }
    break;
  }
}
#endif
return false;
}

/*
This can be used in every menu part where parameters have to be changed. 
this takes a variable and when you press select. it modifies that variable. 
TODO: still need to save it to the EEPROM. 
*/

void buttonListener(uint8_t &counter) //passing values by reference.
{
  static uint8_t count = 0;
  if (UP.wasPressed()) //this should modify the variables address directly.
    count++;
  if (DW.wasPressed())
    count--;
  if (SL.wasPressed())
  {
    counter = count;
    count = 0;
  }
}

/*
Checks if the menu is not being used.
it will discard the changes and jump back
to homepage. 
*/

bool timeOutCheck(const uint32_t delay)
{
  static uint32_t timer = 0;
  static bool firstTime = true;
  if (firstTime)
  {
    timer = currentTime;
    firstTime = false;
  }
  if ((currentTime - timer) >= delay)
  {
    appMode = APP_NORMAL_MODE; //reset to main page. get out of all the level.
    timer = 0;
    firstTime = true; //for next time. turn it on.
    return true;
  }
  if (UP.wasPressed())   //this could cause the program to be stuck on that screen only if one of these buttons malfunction
    timer = currentTime; //if there is anychange. reset the timer to prolong the delay.
  if (DW.wasPressed())
    timer = currentTime;
  if (SL.wasPressed())
    timer = currentTime;

  return false;
}

void homeMenu()
{
  if (update_sensor == true && appMode == APP_NORMAL_MODE)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(currentTemp);
    lcd.setCursor(9, 0);
    lcd.print("H:");
    lcd.print(h);
    lcd.setCursor(0, 1);
    lcd.print("TS:");
    lcd.print(tempSetpoint);
    lcd.setCursor(9, 1);
    lcd.print(humiditySetpoint);
    update_sensor = false;
  }
}

void readFromEEPROM()
{

  double tempSetpointTemp;
  int frequencyTemp;
  int humiditySetpointTemp;
  EEPROM.get(tempAdd, tempSetpointTemp);
  EEPROM.get(humAdd, humiditySetpointTemp);
  EEPROM.get(freqAdd, frequencyTemp);
  if (tempSetpointTemp <= 20 || tempSetpointTemp >= 50 || tempSetpointTemp == NAN)
  {
    EEPROM.put(tempAdd, tempSetpoint);
  }
  else
  {
    tempSetpoint = tempSetpointTemp;
  }
  if (humiditySetpointTemp <= 20 || humiditySetpointTemp >= 90)
  {
    EEPROM.put(humAdd, humiditySetpoint);
  }
  else
  {
    humiditySetpoint = humiditySetpointTemp;
  }
  if (frequencyTemp <= 1 || frequencyTemp >= 48)
  {
    EEPROM.put(freqAdd, frequency);
  }
  else
  {
    frequency = frequencyTemp;
  }
  //means the data retreived is corrupt. will resort to default values instead. other wise will load the new values.
}

/*Polls the switches for latest update*/
void poll()
{
  UP.read();
  DW.read();
  SL.read();
}

//menu functions
char *fmt(char *dest, unsigned char argc, ...)
{
  unsigned char buflen = 0;
  char *str;

  va_list ap;
  va_start(ap, argc);

  for (int i = 0; i < argc && buflen < LCD_COLS; i++)
  {
    str = va_arg(ap, char *);
    unsigned char len = strlen(str);
    unsigned char cpylen = (buflen + len) > LCD_COLS ? LCD_COLS - buflen : len;

    strncpy((dest + buflen), str, cpylen);
    buflen += len;
  }
  va_end(ap);
  dest[buflen] = 0;
  return dest;
}

char *rpad(char *dest, const char *str, char chr, unsigned char width)
{
  unsigned char len = strlen(str);

  width = width > LCD_COLS ? LCD_COLS : width;

  if (len < LCD_COLS && width > len)
  {
    strcpy(dest, str);
    strcat(dest, padc(chr, width - len));
  }
  else
  {
    strncpy(dest, str, width + 1);
  }
  return dest;
}

char *lpad(char *dest, const char *str, char chr, unsigned char width)
{
  unsigned char len = strlen(str);

  width = width > LCD_COLS ? LCD_COLS : width;

  if (len < LCD_COLS && width > len)
  {
    strcpy(dest, padc(chr, width - len));
    strcat(dest, str);
  }
  else
  {
    strncpy(dest, str, width + 1);
  }
  return dest;
}

char *padc(char chr, unsigned char count)
{
  static char strbuf[LCD_COLS + 1];

  count = (count > LCD_COLS) ? LCD_COLS : count;

  int i;
  for (i = 0; i < count; i++)
  {
    strbuf[i] = chr;
  }
  strbuf[i] = 0;

  return strbuf;
}

byte processRuntimeMenuCommand(byte cmdId)
{
  byte complete = false; // set to true when menu command processing complete.

  if (SL.wasPressed())
  {
    complete = true;
  }

  switch (cmdId)
  {
    // TODO Process menu commands here:
  case runtimeCmdTurnMotorOnce:
  if(turnMotorOnce()==true)
  {
    complete = true;
    lcd.setCursor(1,1);
    lcd.print("              ");
  }
  else 
  {
    lcd.setCursor(1,1);
    lcd.print("Turning motor.");
  }
    break;
  case runtimeCmdSetTemp:
    break;
  case runtimeCmdSetHum:
    break;
  case runtimeCmdSetFreq:
    break;
  case runtimeCmdSetTurnDelay:
    break;
  case runtimeCmdSetHours:
    break;
  case runtimeCmdSetMins:
    break;
  case runtimeCmdSetSeconds:
    break;
  case runtimeCmdSetYear:
    break;
  case runtimeCmdSetMonth:
    break;
  case runtimeCmdSetDay:
    break;
  case runtimeCmdSetP:
    break;
  case runtimeCmdSetI:
    break;
  case runtimeCmdSetD:
    break;
  case runtimeCmdIncubationTime:
    break;
  case runtimeCmdSaveProfile:
    break;
  case runtimeCmdShowHatchDay:
    break;
  case runtimeCmdCalcHatchDay:
    break;
  case runtimeCmdToggleLight:
    break;
  case runtimeCmdToggleCandler:
    break;
  case runtimeCmdConfirmation:
    break;
  default:
    break;
  }

  return complete;
}

//----------------------------------------------------------------------
// Callback to convert button press to navigation action.
byte getNavAction()
{
  byte navAction = 0;
  byte currentItemHasChildren = rtMenu.currentItemHasChildren();

  if (UP.wasPressed() || UP.pressedFor(1000))
    navAction = MENU_ITEM_PREV;
  else if (DW.wasPressed() || DW.pressedFor(1000))
    navAction = MENU_ITEM_NEXT;
  else if (SL.wasPressed())
    navAction = MENU_ITEM_SELECT;
  //else if (btn == BUTTON_LEFT_PRESSED) navAction = MENU_BACK;
  return navAction;
}

//----------------------------------------------------------------------
const char EmptyStr[] = "";

// Callback to refresh display during menu navigation, using parameter of type enum DisplayRefreshMode.
void refreshMenuDisplay(byte refreshMode)
{
  char nameBuf[LCD_COLS + 1];

  /*
    if (refreshMode == REFRESH_DESCEND || refreshMode == REFRESH_ASCEND)
    {
      byte menuCount = rtMenu.getMenuItemCount();

      // uncomment below code to output menus to serial monitor
      if (rtMenu.currentMenuHasParent())
      {
        Serial.print("Parent menu: ");
        Serial.println(rtMenu.getParentItemName(nameBuf));
      }
      else
      {
        Serial.println("Main menu:");
      }

      for (int i=0; i<menuCount; i++)
      {
        Serial.print(rtMenu.getItemName(nameBuf, i));

        if (rtMenu.itemHasChildren(i))
        {
          Serial.println("->");
        }
        else
        {
          Serial.println();
        }
      }
    }
  */

  lcd.setCursor(0, 0);
  if (rtMenu.currentItemHasChildren())
  {
    rpad(strbuf, rtMenu.getCurrentItemName(nameBuf));
    strbuf[LCD_COLS - 1] = 0b01111110; // Display forward arrow if this menu item has children.
    lcd.print(strbuf);
    lcd.setCursor(0, 1);
    lcd.print(rpad(strbuf, EmptyStr)); // Clear config value in display
  }
  else
  {
    byte cmdId;
    rpad(strbuf, rtMenu.getCurrentItemName(nameBuf));

    if ((cmdId = rtMenu.getCurrentItemCmdId()) == 0)
    {
      strbuf[LCD_COLS - 1] = 0b01111111; // Display back arrow if this menu item ascends to parent.
      lcd.print(strbuf);
      lcd.setCursor(0, 1);
      lcd.print(rpad(strbuf, EmptyStr)); // Clear config value in display.
    }
    else
    {
      lcd.print(strbuf);
      lcd.setCursor(0, 1);
      lcd.print(" ");

      // TODO Display config value.
    }
  }
}

//alarm on lockdown day to notify the user that its time.
//print a pdf with all the pre incubation and post incubation instructions. and include it with the incubator