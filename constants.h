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

enum Direction
{
    FORWARD = 1,
    HALT = 0,
    BACKWARD = -1
};

const int WEIGHTS[6] = {-100, -60, -20, 20, 60, 100};
const int BASE_SPEED = 170;

enum RobotState
{
    FOLLOW_LINE,
    SEARCH_LINE,
    STOPPED,
    INTERSECTION,
};

RobotState state = FOLLOW_LINE;

double previousError = 0.0;
double lastKnownError = 0.0;

unsigned long lastLoopMicros = 0;
unsigned long allBlackStart = 0;

const double KP = 5;
const double KD = 2;

const int SEARCH_SPEED = 100;

const unsigned long BLACK_HOLD_TIME = 300;

const int WHITE_THRESHOLD = 520;
const int BLACK_THRESHOLD = 400;

struct motor
{
    byte speed = 0;
    Direction direction = HALT;
    int enablePin;
    int directionPin1;
    int directionPin2;
};

#endif // CONSTANTS
