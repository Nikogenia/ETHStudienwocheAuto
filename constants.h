#ifndef CONSTANTS
#define CONSTANTS

#include <Arduino.h>

const int IN4 = 4;
const int IN3 = 3;
const int IN2 = 8;
const int IN1 = 7;
const int ENB = 6;
const int ENA = 5;

const int LRED = 11;
const int LGREEN = 10;
const int LBLUE = 9;

const int SENSOR1 = A5;
const int SENSOR2 = A4;
const int SENSOR3 = A3;
const int SENSOR4 = A2;
const int SENSOR5 = A1;
const int SENSOR6 = A0;

const byte FORWARD = 1;
const byte HALT = 0;
const byte BACKWARD = -1;

struct motor
{
    byte speed = 0;
    byte direction = HALT;
    int enablePin;
    int directionPin1;
    int directionPin2;
};

#endif // CONSTANTS
