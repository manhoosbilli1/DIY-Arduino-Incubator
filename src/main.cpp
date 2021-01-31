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

//base configuration struct to store all the modifiable values related to incubator
#define CONFIG_VERSION "chicken V1"
typedef struct
{
  double tempSetpoint;
  uint8_t humiditySetpoint;
  uint8_t frequency;
  uint8_t turnDelay; //in seconds
  uint8_t daysToHatch;
  uint8_t incubationPeriod;
  uint32_t turnInterval;
  double Kp;
  double Ki;
  double Kd;
  uint8_t eggType;
  char version[sizeof(CONFIG_VERSION)];

} config;

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
//#define SENSORMODEBOTH
//#define USINGRTC
//#define MOTORTYPELIMIT
//#define HEATERMODERELAY
#define SENSORMODEDHT
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
//IF USING DSB18B20 CHANGE, OTHERWISE LEAVE IT AS IT IS.
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
/*----------Motor controller variables-----------*/
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
#endif
double currentTemp;
double f;
double t;
double h;
double curTemp;
uint32_t currentTime = 0;
uint32_t prevTime1 = 0;
uint8_t prevCounter;
uint8_t turnCounter;
uint8_t prevState;
bool update_sensor = 0;
uint8_t eggType = 0;
#define CHICKEN 1
double tempSetpoint = 37.5; //this set of variables are unnecessary. might delete it after im done testing this eeprom manager.
uint8_t humiditySetpoint = 60;
uint8_t frequency = 8;
uint8_t turnDelay = 2; //in seconds
uint8_t daysToHatch = 21;
uint8_t incubationPeriod = 21;
uint32_t turnInterval = 5000;
double Kp = 10;
double Ki = 5;
double Kd = 1.8;
uint8_t incubationStart = 255;

config defaultConfig = { //settings default values
    tempSetpoint,
    humiditySetpoint,
    frequency,
    turnDelay,
    daysToHatch,
    incubationPeriod,
    turnInterval,
    Kp,
    Ki,
    Kd,
    eggType,
    CONFIG_VERSION};

config chickenConfig = {
  37.5,
  60,
  8,
  2,
  21,
  21,
  
  10,
  5,
  3,
  CHICKEN,
  CONFIG_VERSION
};

#define chickenConfigAdd 50
#define customConfigAdd1 150
#define customConfigAdd2 250

//Addresses
#define tempAdd 0
#define humAdd 4
#define freqAdd 8
#define turnDelayAdd 12
#define incubationPAdd 16
#define pAdd 20
#define iAdd 24
#define dAdd 28
#define firstSetupAdd 500      //if this byte is 0xff or 255 in decimal its a fresh start otherwise this incubator has been programmed before. 
                               //should help out in setup when im asking the users a couple of questions. wouldn't need to repeat if i know  
                               //if it is infact their first time operating the incubator.
#define incubationStartAdd 499 //if this byte is high means incubation is on going. otherwise it hasn't begun yet.

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
void firstTimeSetup();
bool isTime();
void motorHandler();
void updateMenu();
void correction();
void readFromEEPROM();
bool turnMotorOnce();
void updateSensor();
void homeMenu();
void poll();
void loadConfig(int add);
void saveConfig(int add, config myConfig);
bool editValue(int add, double &value, double incBy);
bool editValue(int add, uint8_t &value, uint8_t incBy);
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
  firstTimeSetup(); //will entirely depends wether that one byte in eeprom. if its 255 meaning its first time. otherwise directly jump into loop
#ifndef HEATERMODERELAY
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(2300); //2.3 seconds because of slow sensors
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
  readFromEEPROM();
  if (incubationStart == true && eggType == CHICKEN)
  {
    loadConfig(chickenConfigAdd);
  }
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
  switch (appMode)
  {
  case APP_NORMAL_MODE:
    if (UP.pressedFor(1000))
    {
      lcd.clear();
      appMode = APP_MENU_MODE;
      refreshMenuDisplay(REFRESH_DESCEND);
    }
    else
    {
      homeMenu();
    }
    break;
  case APP_MENU_MODE:
  {
    byte menuMode = rtMenu.handleNavigation(getNavAction, refreshMenuDisplay);

    if (menuMode == MENU_EXIT)
    {
      appMode = APP_NORMAL_MODE;
      //should also save any other config settings here.
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

//--------------------FUNCTIONS------------------

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
  myPID.Compute(); //don't forget to uncomment in testing.
  analogWrite(heater, Output);
#else
  //means using relay.
  if (Input > tempSetpoint)
    digitalWrite(heater, LOW);
  else if (Input < tempSetpoint)
    digitalWrite(heater, HIGH);
#endif
  //TODO: ifndef humidifier then don't fire the alarm as soon as it decreases.
  //give it sometime. pleasent and good alarm should be there.
  //build support for generic thermistor type temperature sensor.

  //Makes the adjustment to humidifier.
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

//not tested yet.
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

      case 1: //makes sure it wasn't just some wind or sensor noise. by waiting 2 minutes
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
    lcd.print(defaultConfig.incubationPeriod);
    lcd.print(" ");
    lcd.print(defaultConfig.version);
    update_sensor = false;
  }
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
  if (SL.pressedFor(5000))
  {
    complete = true;
  }

  switch (cmdId)
  {
    // TODO Process menu commands here:
  case runtimeCmdTurnMotorOnce:
    if (turnMotorOnce() == true)
    {
      complete = true;
      lcd.setCursor(1, 1);
      lcd.print("              ");
    }
    else
    {
      lcd.setCursor(1, 1);
      lcd.print("Turning motor.");
    }
    break;

  case runtimeCmdSetTemp:
    complete = editValue(tempAdd, tempSetpoint, 0.5);
    break;
  case runtimeCmdSetHum:
    complete = editValue(humAdd, humiditySetpoint, 1);
    break;
  case runtimeCmdSetFreq:
    complete = editValue(freqAdd, frequency, 1);
    break;
  case runtimeCmdSetTurnDelay:
    complete = editValue(turnDelayAdd, turnDelay, 1);
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
    complete = editValue(pAdd, Kp, 0.1);
    break;
  case runtimeCmdSetI:
    complete = editValue(iAdd, Ki, 0.1);
    break;
  case runtimeCmdSetD:
    complete = editValue(dAdd, Kd, 0.1);
    break;
  case runtimeCmdIncubationTime:
    complete = editValue(incubationPAdd, incubationPeriod, 1);
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
    //TODO. save all settings here or maybe make a new profile at a different eeprom address.
#ifndef HEATERMODERELAY
    myPID.SetTunings(Kp, Ki, Kd);
#endif
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

void readFromEEPROM()
{
  EEPROM.get(tempAdd, tempSetpoint);
  EEPROM.get(humAdd, humiditySetpoint);
  EEPROM.get(freqAdd, frequency);
  EEPROM.get(turnDelayAdd, turnDelay);
  EEPROM.get(pAdd, Kp);
  EEPROM.get(iAdd, Ki);
  EEPROM.get(dAdd, Kd);
  EEPROM.get(incubationPAdd, incubationPeriod);
}

bool editValue(int add, uint8_t &value, uint8_t incBy)
{
  lcd.setCursor(1, 1);
  lcd.print(value);
  if (UP.wasPressed())
  {
    value = value + incBy;
    lcd.setCursor(1, 1);
    lcd.print("     ");
  }

  if (DW.wasPressed())
  {

    if (value != 0)
    {
      value = value - incBy;
      lcd.setCursor(1, 1);
      lcd.print("     ");
    }
  }
  if (SL.wasPressed())
  {
    lcd.setCursor(1, 1);
    lcd.print("Saving...");
    EEPROM.put(add, value);
    delay(500);
    lcd.setCursor(1, 1);
    lcd.print("          ");
    return true;
  }
  else
  {
    return false;
  }
}

bool editValue(int add, double &value, double incBy)
{
  lcd.setCursor(1, 1);
  lcd.print(value);
  if (UP.wasPressed())
  {
    value = value + incBy;
    lcd.setCursor(1, 1);
    lcd.print("     ");
  }

  if (DW.wasPressed())
  {

    if (value != 0)
    {
      value = value - incBy;
      lcd.setCursor(1, 1);
      lcd.print("     ");
    }
  }
  if (SL.wasPressed())
  {
    lcd.setCursor(1, 1);
    lcd.print("Saving...");
    EEPROM.put(add, value);
    delay(500);
    lcd.setCursor(1, 1);
    lcd.print("          ");
    return true;
  }
  else
  {
    return false;
  }
}

void firstTimeSetup()
{
  uint8_t value = 0;
  EEPROM.get(firstSetupAdd, value);
  if (value != 0) //if value != 0 meaning this address has been written to before. so assume its not first setup and return.
  {
    return;
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  Welcome to  ");
    lcd.setCursor(0, 1);
    lcd.print(" DIY incubator! ");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press Select to");
    lcd.setCursor(0, 1);
    lcd.print("Start new Hatch");
    delay(5000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Or..Press Down");
    lcd.setCursor(0, 1);
    lcd.print("For 'Not Now'");
    delay(5000);
    lcd.clear();
    bool setupDone = false;
    uint32_t timer = millis();
    do
    {
      lcd.setCursor(0, 0);
      lcd.print(">UP to start");
      lcd.setCursor(0, 1);
      lcd.print(">Down for later");
      poll();
      if (UP.wasPressed())
      {
        incubationStart = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SELECT EGGS    ");
        delay(2000);
        incubationStart = true;
        value = 255;
        EEPROM.put(firstSetupAdd, value); //value just has to be above 0. actual value doesn't matter.
        setupDone = true;
      }
      else if (DW.wasPressed())
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Going to");
        lcd.setCursor(0, 1);
        lcd.print("Home Page...");
        delay(2000);
        incubationStart = false;
        value = 255; //not first setup anymore.
        EEPROM.put(firstSetupAdd, value);
        setupDone = true;
      }
      else if ((millis() - timer) >= 10000) //if in 10 seconds no input assume no hatch needs to be done.
      {
        //load in chicken config.
        lcd.clear();
        lcd.print("Going to");
        lcd.setCursor(0, 1);
        lcd.print("Home Page...");
        delay(2000);
        incubationStart = false;
        setupDone = true;
      }
    } while (setupDone == false);

    setupDone = false;

    if (incubationStart == true)
    {
      timer = millis();
      lcd.clear();
      do
      {
        lcd.setCursor(0, 0);
        lcd.print("Chicken [UP]");
        lcd.setCursor(0, 1);
        lcd.print("Other [DW] ");
        poll();
        if ((millis() - timer) >= 10000)
        {
          incubationStart = false;
          setupDone = true;
        }
        else if (SL.wasPressed())
        {
          incubationStart = true;
          loadConfig(chickenConfigAdd);                    //will load chicken config into default/current config.
          EEPROM.put(incubationStartAdd, incubationStart); //in order to remember it.
          setupDone = true;
        }
        else if (DW.wasPressed())
        {
          incubationStart = false;
          setupDone = true;
        }

      } while (setupDone == false);
    }
  }
}

void loadConfig(int add) //just need to pass one of the above addresses where configurations are stored.
{
  char ver[sizeof(CONFIG_VERSION)];

  EEPROM.get((add + sizeof(defaultConfig) - sizeof(defaultConfig.version)), ver);
  if (strcmp(ver, defaultConfig.version) == 0)
  {
    EEPROM.get(add, defaultConfig);
  }
  //else by default use the values declared above
}

void saveConfig(int add, config myConfig)
{
  EEPROM.put(add, myConfig);
}