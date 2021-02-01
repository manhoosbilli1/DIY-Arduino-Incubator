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
#define CONFIG_VERSION "incubator V1"

//some definitions for easily calling the lcd characters.
//8 character is the limit.
#define tick 0
#define time 1
#define motor 2
#define memory 3
#define sensor 4
#define alarm 5
byte tickChar[] = {
    B11111,
    B11111,
    B11111,
    B11110,
    B11101,
    B01011,
    B10111,
    B11111};

byte timeChar[] = {
    B11111,
    B11111,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100};

byte motorIcon[] = {
    B00000,
    B10001,
    B11011,
    B10101,
    B10001,
    B10001,
    B10001,
    B00000};

byte memoryIcon[] = {
    B01110,
    B01110,
    B10001,
    B11000,
    B00110,
    B00001,
    B11101,
    B11110};

byte sensorIcon[] = {
    B01110,
    B01010,
    B01010,
    B01010,
    B01010,
    B01010,
    B11111,
    B11111};

typedef struct
{
  uint8_t turnDelay;    //in seconds
  uint8_t turnInterval; //in seconds
  double Kp;
  double Ki;
  double Kd;
  char version[sizeof(CONFIG_VERSION)]; //this is needed always to check for the integrity of data received.
} incubatorConfig;

typedef struct
{
  uint8_t daysToHatch;
  uint8_t firstSetup;
  uint8_t incubationInProgress;
  uint8_t incubationStartDay;
  uint8_t incubationStartMonth;
  uint8_t incubationStartYear;
  uint8_t eggType;
  char version[sizeof(CONFIG_VERSION)]; //this is needed always to check for the integrity of data received.
} incubatorStatus;

typedef struct
{
  double tempSetpoint;
  uint8_t humiditySetpoint;
  uint8_t frequency;
  uint8_t incubationPeriod;
  uint8_t lockdownDay;
  double lockdownTemp;
  uint8_t lockdownHum;
  char version[sizeof(CONFIG_VERSION)]; //this is needed always to check for the integrity of data received. in this case to check wether we have loaded the correct egg type
                                        //configuration by comparing its name with prestored names.

} eggConfig;

//CREATING ERROR ENUM IS ALSO USEFUL FOR HANDLING ERRORS IN SWITCH CASE LATER.
enum ERRORVALUES
{
  NOERROR,
  VERCHECKERROR,
  SENSORERROR,
  SAVEFAILED,
  RTCFAILED
} ERROR;

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
#define USINGRTC
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
#define EDITED 255  //255 should be written to a byte in eeprom after its written to. NOTE: it will only be valid if eeprom has been erased and replaced whole eeprom with 0
#define NOTEDITED 0 // 0 = FRESH ERASED. OR RESET
//FALSE IS NATURALLY ZERO SO NEED FOR MACRO.
//TRUE IS NATURALLY 255 SO NO NEED FOR MACRO.
//some variables to be included in the structs below as default. this is necessary so that incubator can fall back to these if eeprom fails.
//might not need these variables, simple constants while instantiating the struct object should be good enough.
uint8_t eggType = 0; //if it is zero no egg type is chosen.
//this will be used with egg type variable. certain number represent that config. and whenever incubator is loaded it will automatically load that incubator.
#define CHICKEN 1
#define PARROT 2
#define QUAIL 3
//might use define instead of variables to save space.
double tempSetpoint = 37.5; //this set of variables are unnecessary. might delete it after im done testing this eeprom manager.
uint8_t humiditySetpoint = 60;
uint8_t frequency = 8;
uint8_t turnDelay = 2; //in seconds
uint8_t daysToHatch = 21;
uint8_t incubationPeriod = 21;
uint8_t turnInterval = 5; //will store value as seconds instead of milliseconds.
double Kp = 10;
double Ki = 5;
double Kd = 1.8;
uint8_t incubationStart = 255;
uint8_t lockdownDay = 18;
double lockdownTemp = 37.5;
uint8_t lockdownHum = 70;
bool configChanged = false;

//if you wanna make other eggconfig for example for parrots etc. make it in a similar manner. create another struct object and fill the default values and give it a name.
//try to store configs like these in the program instead of eeprom to keep the dependance on eeprom as little as possible because of write cycle limit.
eggConfig myEggConfig = //this is the only struct that is going to be used througout the code. whatever changes is done will be done to this struct.
    {
        tempSetpoint,
        humiditySetpoint,
        frequency,        // you might think this value is related to incubator but actually frequency changes with different eggs.
        incubationPeriod, //loading in the default values for a chicken incubation.
        lockdownDay,      //this changes from egg to egg. will be useful when we tell the motor to stop turning after lockdown day.
        lockdownTemp,
        lockdownHum,
        CONFIG_VERSION //this is needed always to check for the integrity of data received.
};                     //very important for a succesful hatch.

eggConfig chickenEggConfig = //CONTAINS HARDCODE VALUES FOR CHICKEN INCUBATION.
    {
        37.5, //this set of variables are unnecessary. might delete it after im done testing this eeprom manager.
        60,
        8,
        21,
        18,
        37.5,
        70,
        CONFIG_VERSION};

//TODO: add more eggconfigurations for different types of eggs. declaring it here is better than relying on eeprom.

incubatorStatus myIncStatus = //this is completely unrelated to the egg config. supposed to be stored in a different location so incubator can remember.
    {
        21, //will keep track of the current day of incubation. will be useful for future updates.
        NOTEDITED,
        false,     //incubationInProgress.
        NOTEDITED, //start day by default is unedited.
        NOTEDITED,
        NOTEDITED,
        eggType, //might use this in setup when choosing egg type. and based on that load config. (will load config according to this variable.)
        CONFIG_VERSION};

incubatorConfig myIncConfig = //completely unrelated to the rest of the structs. this will remember the config of the incubator because it won't be often changed.
    {
        turnDelay,    //in seconds
        turnInterval, //in seconds          incstatus and incconfig should both be saved in eeprom and loaded when arduino starts.
        Kp,
        Ki,
        Kd,
        CONFIG_VERSION};

//working with these structs mean that individually saving the values to eeprom might not be possible. or might be a bit of work to figure out what the precise address is
//so only saving the config all together makes sense.
//might not need these address if working with structs.
//Addresses

//I think zeroth address should contain the currently running values. and whenever incubator reboots it will simply load whatevers in that eeprom. should make things easy
#define tempAdd 0
#define humAdd 4
#define freqAdd 8
#define turnDelayAdd 12
#define incubationPAdd 16
#define pAdd 20
#define iAdd 24
#define dAdd 28
//only the first config add has to be manually set.
#define chickenConfigAdd 50                                           //this address will always store the currently running config address.NOT!. currentconfig running will always be stored in ram and fetched from eeprom.
#define customConfigAdd1 (chickenConfigAdd + sizeof(myEggConfig) + 1) //whenever loading custom config. that config will be simply copied to currentConfig.
#define customConfigAdd2 (customConfigAdd1 + sizeof(myEggConfig) + 1) //these addresses will be loaded when used chooses to save config. if one is already written to. it will ask to overwrite or choose another location.
#define incuStatusAdd (customConfigAdd2 + sizeof(myEggConfig) + 1)
#define incuConfigAdd (incuStatusAdd + sizeof(myIncStatus)) //need to calculate addresses this way as structs size is bound to changed in later updates.

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
bool firstTimeSetup();
bool isTime();
void motorHandler();
void updateMenu();
void correction();
void readFromEEPROM();
bool turnMotorOnce();
void updateSensor();
void homeMenu();
void poll();
void loadConfig(int add); //will be better if its able to load any config passed to it. and only edit that config.
//void saveConfig(int add, config myConfig);  use the simple EEPROM.put method instead of this as its basically the same thing but better
bool editValue(double &value, double incBy);
bool editValue(uint8_t &value, uint8_t incBy);
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
template <typename T>
bool saveConfiguration(int add, T config);
template <typename T>
bool copyConfiguration(T configDest, T configSource);
bool loadConfiguration();
bool startNewHatch(int eggT);
void errorHandler();
//custon config save and load method still needs work 
void setup()
{
  // put your setup code here, to run once:
  //EEPROM.put(tempAdd,tempSetpoint);
  //initial setup check.
  loadConfiguration();
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
  lcd.createChar(tick, tickChar);
  lcd.createChar(time, timeChar);
  lcd.createChar(memory, memoryIcon);
  lcd.createChar(motor, motorIcon);
  lcd.createChar(sensor, sensorIcon);

#ifdef MOTORTYPELIMIT
  sw1.begin();
  sw2.begin(); //create a conditional here if mode is limit only then initialize.
#endif
  DW.begin();
  UP.begin();
  SL.begin();
  //will entirely depends wether that one byte in eeprom. if its 255 meaning its first time. otherwise directly jump into loop
#ifndef HEATERMODERELAY
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(2300); //2.3 seconds because of slow sensors
#endif
#ifdef USINGRTC
  if (!rtc.begin()) //make it print to lcd if this happens
  {
    ERROR = RTCFAILED;
  }

#endif

  if (!firstTimeSetup())
  {
    ERROR = RTCFAILED;
  }
  //---------PINS INITIALIZATION-------------

  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(dehumFan, OUTPUT);
#ifdef USINGHUMIDIFIER
  pinMode(humidifier, OUTPUT);
  pinMode(waterLevelSensor, OUTPUT);
#endif
  lcd.clear();
  appMode = APP_NORMAL_MODE;
  refreshMenuDisplay(REFRESH_DESCEND);
  lcd.clear();
}

void loop()
{
  // put your main code here, to run repeatedly:
  currentTime = millis();
  poll();
  updateSensor();
  homeMenu();
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
    lcd.setCursor(0, 1);
    lcd.print("H:");
    lcd.print(h);
    errorHandler(); //will update the
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
    complete = editValue(myEggConfig.tempSetpoint, 0.5);
    break;
  case runtimeCmdSetHum:
    complete = editValue(myEggConfig.humiditySetpoint, 1);
    break;
  case runtimeCmdSetFreq:
    complete = editValue(myEggConfig.frequency, 1); //there is probably need to seperate egg related menu, incubator related and status menu.
    break;
  case runtimeCmdSetTurnDelay:
    complete = editValue(myIncConfig.turnDelay, 1);
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
    complete = editValue(myIncConfig.Kp, 0.1);
    break;
  case runtimeCmdSetI:
    complete = editValue(myIncConfig.Ki, 0.1);
    break;
  case runtimeCmdSetD:
    complete = editValue(myIncConfig.Kd, 0.1);
    break;
  case runtimeCmdIncubationTime:
    complete = editValue(myEggConfig.incubationPeriod, 1);
    break;
  case runtimeCmdSaveProfile:
    if (SL.wasPressed())
    {
      lcd.clear();
    }
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
    //this command is not working properly. need to figure out whats wrong. seems like getting stuck in while loop.
    char buf[sizeof(CONFIG_VERSION)];
    int checkAdd = customConfigAdd1 + sizeof(myEggConfig) - sizeof(myEggConfig.version);
    EEPROM.get(checkAdd, buf);

    if (strcmp(buf, CONFIG_VERSION) == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Full. overWrite");
      lcd.setCursor(0, 1);
      lcd.print("Custom1 config?");
      delay(2000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("YES  [UP]");
      lcd.setCursor(0, 1);
      lcd.print("NO   [DOWN]");
      uint32_t timer = millis();
      do
      {
        uint32_t delta = millis() - timer;
        poll();
        if (UP.wasPressed())
        {
          EEPROM.put(customConfigAdd1, myEggConfig);
          complete = true;
          lcd.clear();
          break;
        }
        else if (DW.wasPressed())
        {
          complete = true;
          lcd.clear();
          break;
        }
        else if (delta >= 10000)
        {
          complete = true;
          lcd.clear();
          break;
        }
      } while (complete == false);
    }
    else
    {
      EEPROM.put(customConfigAdd1, myEggConfig);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Saving As");
      lcd.setCursor(0, 1);
      lcd.print("Custom 1");
      delay(2000);
      lcd.clear();
      complete = true;
    }
#ifndef HEATERMODERELAY
    myPID.SetTunings(Kp, Ki, Kd);
#endif
    complete = true;
    break;
  default:
    break;
  }

  if (configChanged == true && complete) //TODO: add an and condition to wether its in default cmd or not. if default dont save to eeprom. instead load other values? not sure.
  {
    saveConfiguration(0, myEggConfig);
    configChanged = false;
  }

//if cmdid == resettodefault    EZ. 
//  myEggConfig = chickenEggConfig; 
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

bool editValue(uint8_t &value, uint8_t incBy)
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
    configChanged = true;
    lcd.setCursor(1, 1);
    lcd.print("Saving...");
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

bool editValue(double &value, double incBy)
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
    configChanged = true;
    lcd.setCursor(1, 1);
    lcd.print("Saving...");
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

bool firstTimeSetup()
{
  //be sure to call this after you load the configuration.
  if (myIncStatus.firstSetup == EDITED) //if value != 0 meaning this address has been written to before. so assume its not first setup and return.
  {
    return false;
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
        lcd.clear();
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
        lcd.clear();
        myIncStatus.incubationInProgress = false;
        myIncStatus.firstSetup = EDITED;
        setupDone = true;
      }
      else if ((millis() - timer) >= 10000) //if in 10 seconds no input assume no hatch needs to be done.
      {
        //load in chicken config.
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Going to");
        lcd.setCursor(0, 1);
        lcd.print("Home Page...");
        delay(2000);
        lcd.clear();
        myIncStatus.firstSetup = EDITED;
        myIncStatus.incubationInProgress = false;
        incubationStart = false;
        setupDone = true;
        break;
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
        lcd.print("Other [DOWN] ");
        poll();
        if ((millis() - timer) >= 10000)
        {
          myIncStatus.incubationInProgress = false;
          myIncStatus.firstSetup = EDITED;
          incubationStart = false;
          setupDone = true;
          lcd.clear();
          break;
        }
        else if (UP.wasPressed())
        {
          myIncStatus.daysToHatch = 21; //because chicken. has to be manually set if choosing another type.
          startNewHatch(CHICKEN);
          incubationStart = true;
          setupDone = true;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Starting With");
          lcd.setCursor(0, 1);
          lcd.print("chicken config.");
          delay(2000);
          lcd.clear();
          break;
        }
        else if (DW.wasPressed())
        {
          incubationStart = false;
          lcd.clear();
          setupDone = true;
          break;
        }

      } while (setupDone == false);
    }
    return true;
  }
}

//first load the incubator status config and then based on its egg type variable choose from which address to load the eggConfig from.
//second load the incubator settings config.
//lastly load the egg config as it could be stored in a different place.
bool loadConfiguration() //there is nothing it can take from its flash or ram as its supposed to be dynamic.
{
  byte succesfull = false;
  char buf[sizeof(CONFIG_VERSION)];
  int checkAdd = ((incuStatusAdd + sizeof(myIncStatus) - sizeof(myIncStatus.version)));
  EEPROM.get(checkAdd, buf);            //first read the check address content. after that compare it. if its good then laod the data otherwise. turn on error variable and load default
  if (strcmp(buf, CONFIG_VERSION) == 0) //if equal to zero = both strings are a mathc.
  {
    EEPROM.get(incuStatusAdd, myIncStatus); //will load the values only if check succeeds otherwise load default
  }
  else
  {
    ERROR = VERCHECKERROR;
  }
  checkAdd = ((incuConfigAdd + sizeof(myIncConfig)) - sizeof(myIncConfig.version));
  EEPROM.get(checkAdd, buf);
  if (strcmp(buf, CONFIG_VERSION) == 0)
  {
    EEPROM.get(incuConfigAdd, myIncConfig);
  }
  else
  {
    ERROR = VERCHECKERROR;
  }
  if (myIncStatus.eggType == CHICKEN)
  {
    checkAdd = ((chickenConfigAdd + sizeof(myEggConfig)) - sizeof(myEggConfig.version));
    EEPROM.get(checkAdd, buf);
    if (strcmp(buf, CONFIG_VERSION) == 0)
    {
      EEPROM.get(chickenConfigAdd, myEggConfig);
    }
    else
    {
      ERROR = VERCHECKERROR;
    }
  }
  if (ERROR == VERCHECKERROR)
    succesfull = false;
  else
    succesfull = true;

  return succesfull;
}

template <typename T>
bool saveConfiguration(int add, T config)
{
  char buf[sizeof(CONFIG_VERSION)];
  int checkAdd = ((add + sizeof(config)) - sizeof(config.version)); //calculate to read back if the data written is good.

  EEPROM.put(add, config);
  EEPROM.get(checkAdd, buf);
  if (strcmp(buf, CONFIG_VERSION) == 0)
  {
    return true; //succesful write.
  }
  else
  {
    ERROR = SAVEFAILED;
    return false;
  }
}

template <typename T>
bool copyConfiguration(T configDest, T configSource)
{
  configDest = configSource; //will have to check if this will work. should work as long as your structures do not contain any pointers.
  return true;
}

//do all the bookkeeping before starting a new hatch in here.
bool startNewHatch(int eggT)
{
  byte succesfull = false;
  DateTime now = rtc.now();
  if (!rtc.begin())
  {
    return false;
    ERROR = RTCFAILED;
  }
  if (eggT == CHICKEN)
  {
    myIncStatus.eggType = CHICKEN;
    myIncStatus.daysToHatch = 21;
  }
  myIncStatus.incubationInProgress = true;
  myIncStatus.incubationStartDay = now.day();
  myIncStatus.incubationStartMonth = now.month();
  myIncStatus.incubationStartYear = now.year();
  myIncStatus.firstSetup = EDITED;

  succesfull = saveConfiguration(incuStatusAdd, myIncStatus);
  return succesfull;
}

void errorHandler() //TO BE INCLUDED IN HOME MENU
{
  switch (ERROR)
  {
  case NOERROR:
    lcd.setCursor(14, 0);
    lcd.write(tick);
    lcd.setCursor(15, 0);
    lcd.write(tick);
    lcd.setCursor(14, 1);
    lcd.write(tick);
    lcd.setCursor(15, 1);
    lcd.write(tick);
    break;

  case VERCHECKERROR:
    lcd.setCursor(14, 0);
    lcd.write(memory);
    break;
  case SENSORERROR:
    lcd.setCursor(15, 0);
    lcd.write(sensor);
    break;
  case SAVEFAILED:
    break;
  case RTCFAILED:
    lcd.setCursor(16, 0);
    lcd.write(time);
    break;
  }
}

//TODO: CREATE A CONFIG FUNCTION IN WHICH ALL SETTINGS WILL BE SET TO DEFAULT.
