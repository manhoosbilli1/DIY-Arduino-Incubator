#include <Arduino.h> //libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include "RTClib.h"
#include <TimeLib.h>
#include <menu.h>
#include "motor_controller.h"
//variables
#define clk 3
#define dt 4
#define sw 5
#define ONE_WIRE_BUS 8
#define DHTPIN 7
#define DHTTYPE DHT22
#define heater 6
#define humidifier A0
#define dehumFan A1
double Setpoint = 28.71, h_setPoint = 60.0, Input, Output; //default values should be set incase eeprom does not work.

double Kp = 10, Ki = 5, Kd = 1.8; //make ki lower if you want slower rise and fall of output.

unsigned long currentTime = 0, lastTime = 0; //timers

volatile boolean TurnDetected, up;

int counter, arrow, prevCounter, counterMax = 5, counterMin = 0, incubationPeriod = 21, counter1 = 0;

bool pressed = 1, updateSensor = 0, updateMenu = 0;

float f, t, h, curTemp;

//Object initiation
DHT dht(DHTPIN, DHTTYPE);

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

DeviceAddress insideThermometer;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

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


enum heaterMode
{
  DC = 0,
  AC = 1,
  RELAY = 2
}heaterMode;

//function declrations
void isr1();

void menu();

void sensor_handler();

void rotary_handler();

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

void count();

void motor_handler();

void setup()
{
  //begin stuff
  dht.begin();

  lcd.init();

  lcd.backlight();

  sensors.begin();

  myPID.SetMode(AUTOMATIC);

  myPID.SetSampleTime(2300);

  if (!rtc.begin()) //make it print to lcd if this happens
  {
    abort();
  }

  //pin intiation
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(clk, INPUT);

  pinMode(dt, INPUT);

  pinMode(sw, INPUT);

  pinMode(dehumFan, OUTPUT);

  pinMode(humidifier, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(clk), isr1, CHANGE);

  delay(100);

  //run first time setup and detect if it is first time setup.
}

void loop()
{
  DateTime now = rtc.now();
  currentTime = millis();
  rotary_handler();
  sensor_handler(); //handles requesting data from sensors
  //myPID.Compute();
  //analogWrite(heater, Output);
  switch (counter) //handles menu
  {
  case home:
    //display sensors
    display_handler(1); //this updates only sensor vaues on screen
    break;

  case page1:
    display_handler(0, 0, "days to hatch", "menu2");
    break;

  case page2:
    display_handler(0, 1, "set PID const", "menu2");
    break;

  case page3:
    display_handler(0, 0, "menu3", "menu4");
    break;

  case page4:
    display_handler(0, 1, "menu3", "menu4");
    break;

  default:
    counter = 0;
  }
}

void isr1()
{
  TurnDetected = true;
  up = (digitalRead(clk) == digitalRead(dt));
}

void rotary_handler()
{
  if (TurnDetected)
  {
    if (up)
    {
      if (counter != counterMax)
        counter++;
      else
        counter = 0;
    }
    else
    {
      if (counter != counterMin)
        counter--;
      else
        counter = counterMax;
    }
    TurnDetected = false;
  }
  if (prevCounter != counter)
  {
    prevCounter = counter;
    updateMenu = true;
  }
}

void display_handler(
    int sensor_update,
    int arrowPos,
    String menu1,
    String menu2,
    int menu1x,
    int menu1y,
    int menu2x,
    int menu2y)
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
      lcd.print(Output); //printing output
      updateSensor = false;
    }
  }
}

void count()
{
  counter1++;
  if (counter1 > 80)
    counter1 = 0;
}

void sensor_handler()
{
  volatile unsigned long currentTime = millis();
  volatile unsigned long prevTime;
  if ((currentTime - prevTime) >= 2000)
  {
    //dht sensor values
    h = dht.readHumidity();
    t = dht.readTemperature();
    f = dht.readTemperature(true);
    //ds18b20 value
    sensors.requestTemperatures();
    float ds_t = sensors.getTempCByIndex(0);
    Input = (t + ds_t) / 2; //returns average value between two.  TODO: make an if statement that this line will execute
    prevTime = currentTime; //only the user has chosen this mode of sensors. if user is using only one sensor.
    updateSensor = true;    //equal currtemp to that only.
  }
}



/*
*control everything related to humidity 
*TODO: make it control water level 
*/

void humidity_handler()
{

  if (h > h_setPoint)
  {
    if (digitalRead(humidifier))
      digitalWrite(humidifier, LOW); //if humidifier is on turn off

    if (!digitalRead(dehumFan))
      digitalWrite(dehumFan, HIGH); //if dehumidifier fan already on do nothing else turn on
  }
  else
    digitalWrite(dehumFan, LOW); //turn off dehum fine to retain humidity
}

//leave the menu for last. for now just use the available function.
//consider using ArduinoMenu library.