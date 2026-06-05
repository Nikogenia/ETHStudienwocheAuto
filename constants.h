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
    INTERSECTION,
    SEARCH_LINE_LEFT,
    SEARCH_LINE_RIGHT,
    SEARCH_LINE_RECOVERY,
    STOPPED,
};

enum IntersectionDecisionMode
{
    LEFT_HAND,
    TREMAUX,
};

struct Junction
{
    int left;
    int forward;
    int right;
};

const int WEIGHTS[6] = {-100, -60, -30, 30, 60, 100};
const int BASE_SPEED = 110;
const int ROTATION_SPEED = 130;

double previousError = 0.0;
unsigned long lastLoopMicros = 0;

const double KP = 2;
const double KD = 0.8;

const unsigned long BLACK_HOLD_TIME = 600;
const unsigned long INTERSECTION_CENTER_TIME = 180;
const unsigned long SEARCH_LINE_TIMEOUT = 340;
const unsigned long SEARCH_LINE_RECOVERY_TIME = 220;
const unsigned long FOLLOW_LINE_INTERSECTION_LOCKOUT = 430;
const unsigned long MOTOR_TIMEOUT = 30;
const unsigned long INTERSECTION_TIMEOUT = 900;
const IntersectionDecisionMode INTERSECTION_DECISION_MODE = LEFT_HAND;

const int WHITE_THRESHOLD = 420;
const int BLACK_THRESHOLD = 420;

Junction currentJunction = {0, 0, 0};
unsigned long intersectionStart = 0;
unsigned long allBlackStart = 0;
unsigned long searchLineStart = 0;
bool restrictToLeft = false;
bool restrictToLeftOrRight = false;
unsigned long searchLineRecoveryStart = 0;
unsigned long followLineIntersectionLockoutUntil = 0;

struct motor
{
    byte speed = 0;
    Direction direction = HALT;
    int enablePin;
    int directionPin1;
    int directionPin2;
};

struct TremauxNode
{
    Junction junction;
    byte nextBranchIndex;
};

const int TREMAUX_STACK_SIZE = 100;
const byte TREMAUX_BRANCH_LEFT = 0;
const byte TREMAUX_BRANCH_FORWARD = 1;
const byte TREMAUX_BRANCH_RIGHT = 2;
const byte TREMAUX_BRANCH_UTURN = 3;

TremauxNode tremauxStack[TREMAUX_STACK_SIZE];
int tremauxStackPointer = -1;

RobotState state = FOLLOW_LINE;
motor motorLeft, motorRight;

#endif // CONSTANTS
