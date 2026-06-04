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
    CALIBRATION,
    FOLLOW_LINE,
    SHARP_TURN,
    SEARCH_LINE,
    STOPPED,
    INTERSECTION,
};

const int WEIGHTS[6] = {-100, -60, -20, 20, 60, 100};
const int BASE_SPEED = 170;
const int SEARCH_SPEED = 100;
const int SHARP_TURN_SPEED = 65;
const unsigned long MOTOR_ACCEL_LIMIT_PER_SECOND = 600;

double previousError = 0.0;
byte lastDirection = 0;

unsigned long lastLoopMicros = 0;
unsigned long allBlackStart = 0;
unsigned long lastDirectionStart = 0;
unsigned long calibrationStartMillis = 0;
unsigned long calibrationRotationMillis = 0;
bool calibrationLeftStart = false;

const double KP = 8;
const double KD = 1.5;

const unsigned long BLACK_HOLD_TIME = 300;
const unsigned long LAST_DIRECTION_TIMEOUT = 100;

const int WHITE_THRESHOLD = 500;
const int BLACK_THRESHOLD = 400;
const double SHARP_TURN_THRESHOLD = 1200.0;
const double DIRECTION_THRESHOLD = 20.0;
const unsigned long ERROR_HISTORY_WINDOW = 100; // milliseconds
const int ERROR_HISTORY_SIZE = 1000;

struct ErrorSample
{
    double error;
    unsigned long timestamp;
};

ErrorSample errorHistory[ERROR_HISTORY_SIZE];
int errorHistoryIndex = 0;

struct motor
{
    byte speed = 0;
    Direction direction = HALT;
    int signedSpeed = 0;
    unsigned long lastCommandMicros = 0;
    int enablePin;
    int directionPin1;
    int directionPin2;
};

RobotState state = SEARCH_LINE;
motor motorLeft, motorRight;

#endif // CONSTANTS
