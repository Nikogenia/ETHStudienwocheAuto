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

enum RobotState
{
    FOLLOW_LINE,
    SHARP_TURN,
    INTERSECTION,
    SEARCH_LINE_LEFT,
    SEARCH_LINE_RIGHT,
    STOPPED,
};

struct Junction
{
    bool left;
    bool forward;
    bool right;
};

const int WEIGHTS[6] = {-100, -60, -30, 30, 60, 100};
const int BASE_SPEED = 100;
const int ROTATION_SPEED = 150;
const int SHARP_TURN_SPEED = 65;

double previousError = 0.0;
unsigned long lastLoopMicros = 0;

const double KP = 0.8;
const double KD = 0.2;

const unsigned long BLACK_HOLD_TIME = 300;
const unsigned long INTERSECTION_CENTER_TIME = 300;
const unsigned long SEARCH_LINE_TIMEOUT = 00;
const unsigned long MOTOR_TIMEOUT = 30;
Junction currentJunction = {false, false, false};
unsigned int intersectionStart = 0;
unsigned int allBlackStart = 0;
unsigned int searchLineStart = 0;

const int WHITE_THRESHOLD = 450;
const int BLACK_THRESHOLD = 450;

struct motor
{
    byte speed = 0;
    Direction direction = HALT;
    int enablePin;
    int directionPin1;
    int directionPin2;
};

RobotState state = SEARCH_LINE_LEFT;
motor motorLeft, motorRight;

#endif // CONSTANTS
