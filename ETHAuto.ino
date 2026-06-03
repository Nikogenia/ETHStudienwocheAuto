#include "constants.h"

motor motorLeft, motorRight;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    setupMotor(motorLeft, ENA, IN1, IN2);
    setupMotor(motorRight, ENB, IN3, IN4);
    setMotorSpeed(motorLeft, 80);
    setMotorSpeed(motorRight, 80);
}

void moveBackward()
{
    setMotorDirection(motorLeft, BACKWARD);
    setMotorDirection(motorRight, BACKWARD);
    sendToMotor(motorLeft);
    sendToMotor(motorRight);
};

void moveForward()
{
    setMotorDirection(motorLeft, FORWARD);
    setMotorDirection(motorRight, FORWARD);
    sendToMotor(motorLeft);
    sendToMotor(motorRight);
}

void rotateLeft()
{
    setMotorDirection(motorLeft, BACKWARD);
    setMotorDirection(motorRight, FORWARD);
    sendToMotor(motorLeft);
    sendToMotor(motorRight);
}

void rotateRight()
{
    setMotorDirection(motorLeft, FORWARD);
    setMotorDirection(motorRight, BACKWARD);
    sendToMotor(motorLeft);
    sendToMotor(motorRight);
}

void loop()
{
    moveForward();
    digitalWrite(LED_BUILTIN, LOW);
    delay(3000);
    rotateLeft();
    delay(3000);
    rotateRight();
    delay(3000);
    moveBackward();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
}
