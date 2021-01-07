#include <Arduino.h> //libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//variables
#define clk 3
#define dt 4
#define sw 5
#define ONE_WIRE_BUS 8
#define DHTPIN 7
#define DHTTYPE DHT22

//Object initiation
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

unsigned long currentTime = 0, lastTime = 0;
volatile boolean TurnDetected, up;
int counter, arrow, prevCounter, counterMax = 5, counterMin = 0, pidInterval = 2100;
bool pressed = 1, updateSensor = 0, updateMenu = 0;
float f, t, h, ds_t, t_setPoint = 37.5, h_setPoint = 60, incubationPeriod = 21, curTemp;
//PID configuration
float KP = 0.0;
float KI = 0.0;
float KD = 0.0;
float last_error;
float errSum;
float output = 0;
float GOAL = 0;
unsigned long T;

//structs  and enums

enum menu
{
  home = 0,
  page1 = 1,
  page2 = 2,
  page3 = 3,
  page4 = 5
} page;

//function definition
void isr0();
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

void setup()
{
  lcd.init();
  lcd.backlight();
  Serial.begin(2000000);
  //begin stuff
  dht.begin();
  sensors.begin();
  //pin intiation
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(clk, INPUT);
  pinMode(dt, INPUT);
  pinMode(sw, INPUT);
  attachInterrupt(digitalPinToInterrupt(clk), isr0, CHANGE);
  delay(100);

  //run first time setup and detect if it is first time setup.
}

void loop()
{
  currentTime = millis();
  rotary_handler();
  sensor_handler(); //handles requesting data from sensors

  switch (counter) //handles menu
  {
  case home:
    //display sensors
    display_handler(1); //this updates only sensor vaues on screen
    break;

  case page1:
    display_handler(0, 0, "menu1", "menu2");
    break;

  case page2:
    display_handler(0, 1, "menu1", "menu2");
    break;

  case page3:
    display_handler(0, 0, "menu3", "menu4");
    break;

  case page4:
    display_handler(0, 1, "menu3", "menu4");
    break;
  }
}

void isr0()
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
      {
        counter++;
      }
      else
      {
        counter = 0;
      }
    }
    else
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
      lcd.print("DS_T: ");
      lcd.print(ds_t);
      updateSensor = false;
    }
  }
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
    ds_t = sensors.getTempCByIndex(0);
    curTemp = (t + ds_t) / 2; //returns average value between two.  TODO: make an if statement that this line will execute
    prevTime = currentTime;   //only the user has chosen this mode of sensors. if user is using only one sensor.
    updateSensor = true;      //equal currtemp to that only.
  }
}
void temp_controller()                             //only use this format if using timer interrupt. otherwise time it
{
  unsigned long time = millis();                   //record the current time

  float timeChange = (float)(time - lastTime);     //calculate time change since last executed

  float errorT = t_setPoint - curTemp;             //calculate error for average temp

  errSum += (errorT * timeChange);                 //will increase with time

  float dErr = (errorT - last_error) / timeChange; //time is ommitted from these equation cause timechange is constant

  output = KP * errorT + KI * errSum + KD * dErr;  //output will be delay feeding into mosfet or diac. 0-255 output pwm value

  last_error = errorT;                             //save last error for differential

  lastTime = time;                                 //save time for finding delta time
}
//need to figure out the loop time. should be around 2s because thats when sensors values are going to be updated

void setPIDK(float kp, float ki, float kd)
{
  KP = kp;
  KI = ki;
  KD = kd;
  //TODO: write this to EEPROM to remember.
}