#include "motor_controller.h"
#include <Arduino.h>
#include "JC_Button.h"

//TODO: make debounce class that will perform the same as JC button library or better 
//TODO: find a way to make that class work with this library (might be hard)
void MotorController::begin(uint8_t mode, uint8_t motor_pinA, uint8_t motor_pinB, uint8_t limitSw1 = 0, uint8_t limitSw2 = 1)
{

    setMode(mode);
    m_motor_pinA = motor_pinA;
    m_motor_pinB = motor_pinB;

    if (MODE == TIMER)
    {
        pinMode(m_motor_pinA, OUTPUT);
        pinMode(m_motor_pinB, OUTPUT);
    }
    else if (MODE == LIMIT)
    {
        pinMode(motor_pinA, OUTPUT);
        pinMode(motor_pinB, OUTPUT);
        Button::Button sw1(limitSw1);
        Button::Button sw2(limitSw1);
        sw1.begin(); //see to it that switches are working. im not sure this will work because of scope issue
        sw2.begin();
    }
}

void MotorController::setMode(int mode)
{
    MODE = mode;
}

int MotorController::getMode()
{
    return MODE;
}

void MotorController::setTurnDelay(unsigned long ms)
{
    turnDelay = ms;
}

unsigned long MotorController::getTurnDelay()
{
    return turnDelay;
}

void MotorController::setFreq(int freq)
{
    FREQ = freq;
}

int MotorController::getFreq()
{
    return FREQ;
}

unsigned long MotorController::getTurnInterval()
{
    if (turnInterval != 0)
    {
        return turnInterval;
    }
    return 0; 
}

bool MotorController::isTime() //will tell if its time to turn
{
    if ((millis() - prevTime) >= turnInterval)
    {
        prevTime = millis();
        return true;
    }
    return false;
    //this should return nothing unless the statement in if condition is returned. if it does, gg
}

void MotorController::update() //this is going to run in loop.
{
    uint32_t timer; //remember this is local scope. might pose problem. this is used to record millis
    static bool is_time = false;
    if (MotorController::isTime())
    {
        is_time = true; //we catch the flag.
    }

    static uint8_t currentState = 0; //using keyword static so that it remembers its state. and not get reinitialized

    if (getMode() == TIMER) //timer based motor
    {
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
                    is_time = false;                 //just turned. turn off the flag and wait for it to occur again.
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
                    is_time = false;
                    currentState = STATE_IDLE;
                }
                break;
            }
        }
    }
    else if (getMode() == LIMIT) //execute this block of code if mode is set to LIMIT
    {
        static bool is_time = false;
        if (MotorController::isTime())
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
                }                              //it should start from idle instead of moving left case.
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
}

bool MotorController::turnOnce() //will turn the motor once. will not care about if its time for turning. if needs calibration will calibrate. but you need to run the function again.
{
    uint32_t timer;                   //remember this is local scope. might pose problem. this is used to record millis
    static uint8_t currentState1 = 0; //using keyword static so that it remembers its state. and not get reinitialized
    if (getMode() == TIMER)           //timer based motor
    {
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
                return true; //break the function when turning is done.
            }
            break;
        }
    }
    else if (getMode() == LIMIT) //execute this block of code if mode is set to LIMIT
    {
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
            }                               //it should start from idle instead of moving left case.

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
    return false;
}
