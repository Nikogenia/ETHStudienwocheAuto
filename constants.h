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
    GAP_CHECK,
    SEARCH_LINE,
    STOPPED,
    INTERSECTION,
};

struct Junction
{
    bool left;
    bool forward;
    bool right;
};

enum IntersectionScanPhase
{
    INTERSECTION_SCAN_CENTER,
    INTERSECTION_SCAN_LEFT_OUT,
    INTERSECTION_SCAN_LEFT_BACK,
    INTERSECTION_SCAN_RIGHT_OUT,
    INTERSECTION_SCAN_RIGHT_BACK,
    INTERSECTION_SCAN_DONE
};

const int WEIGHTS[6] = {-100, -60, -20, 20, 60, 100};
const int BASE_SPEED = 130;
const int SEARCH_SPEED = 100;
const int SHARP_TURN_SPEED = 65;
const unsigned long MOTOR_ACCEL_LIMIT_PER_SECOND = 1000;

double previousError = 0.0;
byte lastDirection = 0;

const unsigned long INTERSECTION_HOLD_TIME = 60;
const unsigned long INTERSECTION_ADVANCE_TIME = 350;
const unsigned long INTERSECTION_SETTLE_TIME = 80;
const unsigned long GAP_CHECK_TIME = 50;
const int GAP_CHECK_SPEED = 80;
const int INTERSECTION_BLACK_COUNT_THRESHOLD = 4;
const int INTERSECTION_CENTER_SPEED = 80;
const unsigned long INTERSECTION_PROBE_TIME = 140;
const int INTERSECTION_PROBE_SPEED = 90;
unsigned long lastLoopMicros = 0;
unsigned long allBlackStart = 0;
unsigned long lastDirectionStart = 0;
unsigned long calibrationStartMillis = 0;
unsigned long calibrationRotationMillis = 0;
bool calibrationLeftStart = false;

const double KP = 4;
const double KD = 0.9;

const unsigned long BLACK_HOLD_TIME = 300;
const unsigned long LAST_DIRECTION_TIMEOUT = 200;
unsigned long intersectionCandidateStartMillis = 0;
unsigned long intersectionStartMillis = 0;
unsigned long intersectionPhaseStartMillis = 0;
unsigned long gapCheckStartMillis = 0;
RobotState resumeStateAfterGapCheck = FOLLOW_LINE;
IntersectionScanPhase intersectionScanPhase = INTERSECTION_SCAN_CENTER;
bool intersectionArmed = false;
bool intersectionLogged = false;
Junction currentJunction = {false, false, false};

const int WHITE_THRESHOLD = 520;
const int BLACK_THRESHOLD = 470;
const double SHARP_TURN_THRESHOLD = 20;
const double DIRECTION_THRESHOLD = 0;
const unsigned long ERROR_HISTORY_WINDOW = 150; // milliseconds
const int ERROR_HISTORY_SIZE = 200;

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
    int enablePin;
    int directionPin1;
    int directionPin2;
};

RobotState state = SEARCH_LINE;
motor motorLeft, motorRight;

#endif // CONSTANTS
