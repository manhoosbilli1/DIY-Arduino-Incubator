#include <Arduino.h> //libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include "RTClib.h"
#include <JC_Button.h>
//Pins
#define ONE_WIRE_BUS 8
#define DHTPIN 7
#define DHTTYPE DHT22
#define heater 6
#define humidifier A0
#define dehumFan A1

const byte
    pinUp(9),
    pinDw(4),
    pinSl(5);

//motor controller variables

uint8_t m_motor_pinA = 12; //default pin numbers. dont forget to change.
uint8_t m_motor_pinB = 11; //this is defined and limit switches are not because those are assigned to object and these need to be used later on.
#define STATE_IDLE 0
#define STATE_MOVING_RIGHT 1
#define STATE_MOVING_LEFT 2
#define MOTORMODETIMER 1
//#define MOTORMODELIMIT 0            //uncomment whichever mode using. default timer mode

//motor controller user defined variables
unsigned long turnDelay = 5000; //default 20sec
uint8_t FREQ = 8;
unsigned long turnInterval = (24 / FREQ) * 3600000UL; //calculates the turnInterval in milliseconds
uint8_t prevState = 0;
const uint8_t limitSw1 = -1;
const uint8_t limitSw2 = -1;

#define USINHHUMIDIFIER 1 //comment if not using humidifier.
//#define HEATERMODERELAY 1 //comment if using AC or DC heater that can be controlled with mosfet.
//#define sensorModeDHT 1  //uncomment whichever mode you want and comment the other ones.
//#define sensorModeDSB 1
#define sensorModeBOTH 1
#define USINGRTC 1
/*      Humidity related variables      */
//#define USINGWATERPUMP 1 //if using water level sensor uncomment this
#ifdef USINGWATERPUMP
uint8_t waterLevelSensor; //if using water level sensor write the pin number
uint8_t waterMinLimit;
uint8_t waterMaxLimit; //if using water level sensor mark, replace this with values. if not using level sensor use with time.
#endif
    //UNCOMMENT IF USING RTC COMMENT IF NOT.
double Setpoint = 28.71, h_setPoint = 60.0, Input, Output; //default values should be set incase eeprom does not work.
double Kp = 10, Ki = 5, Kd = 1.8;                          //make ki lower if you want slower rise and fall of output.
unsigned long currentTime = 0, lastTime = 0, prevTime1 = 0, prevTime = 0;
volatile boolean TurnDetected, up;
unsigned int counter, prevCounter, counterMax = 5, counterMin = 0, incubationPeriod = 21, turnCounter;
bool pressed = 1, updateSensor = 0, updateMenu = 0,turnedOnce = 0;
float f, t, h, curTemp;

//Object initiation
#ifdef sensorModeDHT
DHT dht(DHTPIN, DHTTYPE);
#endif
#ifdef sensorModeDSB
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
#endif

#ifdef sensorModeBOTH
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
#endif

#ifndef HEATERMODEREALY //only if not using relay
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
#endif

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

//buttons initialization
Button sw1(limitSw1);
Button sw2(limitSw1);
Button UP(pinUp); //up
Button DW(pinDw); //down
Button SL(pinSl); //select

RTC_DS3231 rtc;

//structs  and enums
enum menu
{
  home = 0,
  page1 = 1,
  page2 = 2,
  page3 = 3,
  page4 = 4
} page;

//function declrations
void menu();
void sensor_handler();
void display_handler(
    int sensor_update = 0,
    int arrowPos = 0,
    String menu1 = "menu",
    String menu2 = "menu",
    int menu1x = 1,
    int menu1y = 0,
    int menu2x = 1,
    int menu2y = 1);
void compute();
void motorUpdate();
void button_handler();
bool turnOnce();
bool isTime();
void humidity_handler();
void update_display(uint8_t counter);

void setup()
{
  //begin stuff here
  #ifdef sensorModeDHT
  dht.begin();
#endif
#ifdef sensorModeDSB
  sensors.begin();
#endif
#ifdef sensorModeBOTH
  dht.begin();
  sensors.begin();
#endif
  lcd.init();
  lcd.backlight();

#ifdef MOTORMODELIMIT
  sw1.begin();
  sw2.begin(); //create a conditional here if mode is limit only then initialize.
#endif
  DW.begin();
  UP.begin();
  SL.begin();

#ifndef HEATERMODERELAY
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(2300); //2.3 seconds
#endif

#ifdef USINGRTC
  if (!rtc.begin()) //make it print to lcd if this happens
  {
    abort();
  }
#endif
  //pin intiation*/
  pinMode(m_motor_pinA, OUTPUT);
  pinMode(m_motor_pinB, OUTPUT);
  //pinMode(dehumFan, OUTPUT);
#ifdef USINGHUMIDIFIER
  pinMode(humidifier, OUTPUT);
#endif
}

void loop()
{
  DateTime now = rtc.now();
  currentTime = millis();
  sensor_handler(); //handles requesting data from sensors
  button_handler();
  /*#ifndef HEATERMODERELAY
  myPID.Compute();
  analogWrite(heater, Output);
#else
  //means using relay.
  if (Input >= Setpoint)
    digitalWrite(heater, LOW);
  else
    digitalWrite(heater, HIGH);
#endif
*/
  update_display(counter);
  if (!turnedOnce)
  {
    turnOnce(); //will execute the function until it is true.
  }

}

void display_handler(int sensor_update, int arrowPos, String menu1, String menu2, int menu1x, int menu1y, int menu2x, int menu2y)
{
  if (counter > 0 && updateMenu == true)
  {
    lcd.clear();
    if (arrowPos == 0)
    {
      lcd.setCursor(0, 0);
      lcd.print(">");
    }
    if (arrowPos == 1)
    {
      lcd.setCursor(0, 1);
      lcd.print(">");
    }
    lcd.setCursor(menu1x, menu1y);
    lcd.print(menu1);
    lcd.setCursor(menu2x, menu2y);
    lcd.print(menu2);
    updateMenu = false;
  }

  if (sensor_update == 1 && counter == 0)
  {
    if (updateSensor)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("T: ");
      lcd.print(t);
      lcd.setCursor(9, 0);
      lcd.print("H: ");
      lcd.print(h);
      lcd.setCursor(0, 1);
      lcd.print("Input:");
      lcd.print(Input);
      lcd.setCursor(13, 1);
      lcd.print(counter); //printing output
      updateSensor = false;
    }
  }
}

void sensor_handler()
{
  if ((currentTime - prevTime1) >= 2000)
  {

#ifdef sensorModeDHT
    h = dht.readHumidity();
    t = dht.readTemperature();
    f = dht.readTemperature(true);
    Input = t;
    prevTime1 = currentTime; //only the user has chosen this mode of sensors. if user is using only one sensor.
    updateSensor = true;     //equal currtemp to that only.
#endif
#ifdef sensorModeDSB
    sensors.requestTemperatures();
    Input = sensors.getTempCByIndex(0);
    prevTime1 = currentTime; //only the user has chosen this mode of sensors. if user is using only one sensor.
    updateSensor = true;     //equal currtemp to that only.
#endif
#ifdef sensorModeBOTH
    h = dht.readHumidity();
    t = dht.readTemperature();
    f = dht.readTemperature(true);
    sensors.requestTemperatures();
    float ds_t = sensors.getTempCByIndex(0);
    float average = (t + ds_t) / 2;
    Input = average;
    prevTime1 = currentTime; //only the user has chosen this mode of sensors. if user is using only one sensor.
    updateSensor = true;     //equal currtemp to that only.
#endif
  }
}

void humidity_handler()
{

  if (h > h_setPoint)
  {
#ifdef USINGHUMIDIFIER
    if (digitalRead(humidifier))
      digitalWrite(humidifier, LOW); //if humidifier is on turn off
#endif
    if (!digitalRead(dehumFan))
      digitalWrite(dehumFan, HIGH); //if dehumidifier fan already on do nothing else turn on
  }
  else
    digitalWrite(dehumFan, LOW); //turn off dehum fine to retain humidity

#ifdef USINGWATERPUMP
  if (analogRead(waterLevelSensor) < (waterMinLimit - 5))
  {

    while (analogRead(waterLevelSensor) >= waterMaxLimit)
    {
      pinMode(waterLevelSensor, OUTPUT);
      digitalWrite(waterLevelSensor, HIGH);
    }
    digitalWrite(waterLevelSensor, LOW);
  }
#endif
  if (h < (h_setPoint - 5))
  {
    digitalWrite(dehumFan, LOW);
#ifdef USINGHUMIDIFIER
    digitalWrite(humidifier, HIGH);
#endif
  }
  //turn off humidity low light or alarm.
}

//--------------motor controller part-------------

bool isTime() //will tell if its time to turn
{
  if ((millis() - prevTime) >= turnInterval)
  {
    prevTime = millis();
    return true;
  }
  return false;
  //this should return nothing unless the statement in if condition is returned. if it does, gg
}

void update() //this is going to run in loop.
{
  uint32_t timer; //remember this is local scope. might pose problem. this is used to record millis
  static bool is_time = false;
  if (isTime())
  {
    is_time = true; //we catch the flag.
    if (turnCounter == FREQ)
      turnCounter = 0;
  }

  static uint8_t currentState = 0; //using keyword static so that it remembers its state. and not get reinitialized

#ifdef MOTORMODETIMER
  if (is_time) //if its time to turn
  {
    switch (currentState)
    {

    case STATE_IDLE:
      if ((prevState != STATE_MOVING_LEFT) && (prevState != STATE_MOVING_RIGHT)) //then move right
      {
        digitalWrite(m_motor_pinA, HIGH);
        digitalWrite(m_motor_pinB, LOW);
        timer = millis();                 //record the time
        currentState = STATE_MOVING_LEFT; //direction is abstract. doesn't matter
      }

      if (prevState == STATE_MOVING_LEFT)
      {
        digitalWrite(m_motor_pinA, LOW);
        digitalWrite(m_motor_pinB, HIGH);
        timer = millis();
        currentState = STATE_MOVING_RIGHT;
      }

      if (prevState == STATE_MOVING_RIGHT)
      {
        digitalWrite(m_motor_pinA, HIGH);
        digitalWrite(m_motor_pinB, LOW);
        timer = millis();
        currentState = STATE_MOVING_LEFT;
      }
      break;

    case STATE_MOVING_LEFT:
      if (millis() - timer <= turnDelay) //
      {
        //wait for the motor to reach the other extreme position
      }
      else
      {
        digitalWrite(m_motor_pinA, LOW);
        digitalWrite(m_motor_pinB, LOW); //turn off the motors since destination reached
        prevState = STATE_MOVING_LEFT;   //write this to eeprom just to be safe
        turnCounter++;
        is_time = false; //just turned. turn off the flag and wait for it to occur again.
        currentState = STATE_IDLE;
      }
      break;

    case STATE_MOVING_RIGHT:
      if (millis() - timer <= turnDelay)
      {
        //wait for the motor to reach the other extreme position
      }
      else
      {
        digitalWrite(m_motor_pinA, LOW);
        digitalWrite(m_motor_pinB, LOW);
        prevState = STATE_MOVING_RIGHT;
        turnCounter++;
        is_time = false;
        currentState = STATE_IDLE;
      }
      break;
    }
  }
#endif
#ifdef MOTORMODELIMIT
  static bool is_time = false;
  if (isTime())
  {
    is_time = true; //we catch the flag.
  }
  static uint8_t currentState = 0; //using static so that it remembers its state.
  if (is_time)
  {
    switch (currentState)
    {
    case STATE_IDLE:
      if ((sw1.read() == false) && (sw2.read() == false)) //tray is in unknown state. needs calibration
      {
        digitalWrite(m_motor_pinA, HIGH);
        digitalWrite(m_motor_pinA, LOW);
        currentState = STATE_MOVING_LEFT; //switch state
      }

      if ((sw1.read() == true) && (sw2.read() == false)) //already at left. time to go right
      {
        digitalWrite(m_motor_pinA, HIGH);
        digitalWrite(m_motor_pinB, LOW);
        currentState = STATE_MOVING_RIGHT;
      }

      if ((sw1.read() == false) && (sw2.read() == true)) //already at right. time to go left.
      {
        digitalWrite(m_motor_pinA, LOW);
        digitalWrite(m_motor_pinB, LOW);
        currentState = STATE_IDLE;
      }
      break;

    case STATE_MOVING_LEFT:
      if ((sw1.read() == true) && (sw2.read() == false)) //means tray is tilted one way.
      {
        digitalWrite(m_motor_pinA, LOW);
        digitalWrite(m_motor_pinB, LOW);
        is_time = false;           //reset the time since tray reached destination
        currentState = STATE_IDLE; //current state = idle when pc reaches switch statement.
      }                            //it should start from idle instead of moving left case.
      break;

    case STATE_MOVING_RIGHT:
      if ((sw1.read() == false) && (sw2.read() == true))
      {
        digitalWrite(m_motor_pinA, LOW);
        digitalWrite(m_motor_pinB, LOW);
        is_time = false;
        currentState = STATE_IDLE;
      }
      break;
    }
  }
}
#endif
}

bool turnOnce() //will turn the motor once. will not care about if its time for turning. if needs calibration will calibrate. but you need to run the function again.
{
#ifdef MOTORMODETIMER               //timer based motor
  static uint32_t timer;            //remember this is local scope. might pose problem. this is used to record millis
  static uint8_t currentState1 = 0; //using keyword static so that it remembers its state. and not get reinitialized
  switch (currentState1)
  {
  case STATE_IDLE:
    if ((prevState != STATE_MOVING_LEFT) && (prevState != STATE_MOVING_RIGHT)) //then move right
    {
      digitalWrite(m_motor_pinA, HIGH);
      digitalWrite(m_motor_pinB, LOW);
      timer = millis();                  //record the time
      currentState1 = STATE_MOVING_LEFT; //direction is abstract. doesn't matter
    }

    if (prevState == STATE_MOVING_LEFT)
    {
      digitalWrite(m_motor_pinA, LOW);
      digitalWrite(m_motor_pinB, HIGH);
      timer = millis();
      currentState1 = STATE_MOVING_RIGHT;
    }

    if (prevState == STATE_MOVING_RIGHT)
    {
      digitalWrite(m_motor_pinA, HIGH);
      digitalWrite(m_motor_pinB, LOW);
      timer = millis();
      currentState1 = STATE_MOVING_LEFT;
    }
    break;

  case STATE_MOVING_LEFT:
    if (millis() - timer <= turnDelay) //
    {
      //wait for the motor to reach the other extreme position
    }
    else
    {
      digitalWrite(m_motor_pinA, LOW);
      digitalWrite(m_motor_pinB, LOW); //turn off the motors since destination reached
      currentState1 = STATE_IDLE;
      turnedOnce = true;
      return true; //break the function when turning is done.
    }
    break;

  case STATE_MOVING_RIGHT:
    if (millis() - timer <= turnDelay)
    {
      //wait for the motor to reach the other extreme position
    }
    else
    {
      digitalWrite(m_motor_pinA, LOW);
      digitalWrite(m_motor_pinB, LOW);
      currentState1 = STATE_IDLE;
      turnedOnce = true;
      return true; //break the function when turning is done.
    }
    break;
  }
#endif
#ifdef MOTORMODELIMIT
  static uint8_t currentState1 = 0; //using static so that it remembers its state.
  switch (currentState1)
  {
  case STATE_IDLE:
    if ((sw1.read() == false) && (sw2.read() == false)) //tray is in unknown state. needs calibration
    {
      digitalWrite(m_motor_pinA, HIGH);
      digitalWrite(m_motor_pinA, LOW);
      currentState1 = STATE_MOVING_LEFT; //switch state
    }

    if ((sw1.read() == true) && (sw2.read() == false)) //already at left. time to go right
    {
      digitalWrite(m_motor_pinA, HIGH);
      digitalWrite(m_motor_pinB, LOW);
      currentState1 = STATE_MOVING_RIGHT;
    }

    if ((sw1.read() == false) && (sw2.read() == true)) //already at right. time to go left.
    {
      digitalWrite(m_motor_pinA, LOW);
      digitalWrite(m_motor_pinB, LOW);
      currentState1 = STATE_IDLE;
    }
    break;

  case STATE_MOVING_LEFT:
    if ((sw1.read() == true) && (sw2.read() == false)) //means tray is tilted one way.
    {
      digitalWrite(m_motor_pinA, LOW);
      digitalWrite(m_motor_pinB, LOW);
      currentState1 = STATE_IDLE; //current state = idle when pc reaches switch statement.
      return true;                //break the function when turning is done.
    }                             //it should start from idle instead of moving left case.

    break;

  case STATE_MOVING_RIGHT:
    if ((sw1.read() == false) && (sw2.read() == true))
    {
      digitalWrite(m_motor_pinA, LOW);
      digitalWrite(m_motor_pinB, LOW);
      currentState1 = STATE_IDLE;
      return true; //break the function when turning is done.
    }
    break;
  }
}
#endif
return false;
}

//--------------motor controller part finished----------
void button_handler() //this will read the buttons wherever you are. just call this function before checking and it should update the counter
{
  DW.read();
  UP.read();
  SL.read();
  if (UP.wasPressed())
  {
    if (counter != counterMax)
    {
      counter++;
    }
    else
    {
      counter = counterMin;
    }
  }
  else if (DW.wasPressed())
  {
    if (counter != counterMin)
    {
      counter--;
    }
    else
    {
      counter = counterMax;
    }
  }
  if (prevCounter != counter)
  {
    updateMenu = true;
    prevCounter = counter;
    Serial.println(counter);
  }
}

void update_display(uint8_t counter)
{
  switch (counter) //handles menu
  {
  case home:
    //display sensors
    display_handler(1); //this updates only sensor vaues on screen
    break;

  case page1:
    display_handler(0, 0, "Calc Hatch Day", "Set Turns / Day");
    break;

  case page2:
    display_handler(0, 1, "Calc Hatch Day", "Set Turns / Day");
    break;

  case page3:
    display_handler(0, 0, "Set Turn Delay", "Using Water Pmp?");
    break;

  case page4:
    display_handler(0, 1, "Set Turn Delay", "Using Water Pmp?");
    break;
  default:
    counter = 0;
  }
}