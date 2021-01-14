#ifndef motor_controller_h
#define motor_controller_H

#include <Arduino.h>
#include <JC_Button.h>

const uint8_t limitSw1 = 10;
const uint8_t limitSw2 = 9;
#define TIMER 0
#define LIMIT 1
#define STATE_IDLE 0
#define STATE_MOVING_RIGHT 1
#define STATE_MOVING_LEFT 2

class MotorController : Button
{
private:
    uint8_t MODE = TIMER;
    unsigned long turnDelay = 200000; //default 20sec
    int FREQ = 8;
    unsigned long prevTime = 0;
    unsigned long turnInterval = (24 / FREQ) * 3600000UL; //calculates the turnInterval in milliseconds
    uint8_t m_motor_pinA = 11;                            //default pin numbers. dont forget to change.
    uint8_t m_motor_pinB = 12;                            //this is defined and limit switches are not because those are assigned to object and these need to be used later on.
    uint8_t prevState = 0;

public:
    Button sw1; //declarings objects in order to be used globally
    Button sw2; //default constructor is doing nothing. only when i call this in .cpp is when it works
    void begin(uint8_t mode = 0, uint8_t motor_pinA = 11, uint8_t motor_pinB = 12, uint8_t limitSw1 = 10, uint8_t limitSw2 = 9);

    void setMode(int mode);

    int getMode();

    void setTurnDelay(unsigned long ms);

    unsigned long getTurnDelay();

    void setFreq(int freq);

    int getFreq();

    unsigned long getTurnInterval();

    bool isTime();

    void update();

    bool turnOnce();
};
#endif