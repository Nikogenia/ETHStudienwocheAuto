#include "constants.h"

int limitMotorSpeedChange(int currentSpeed, int targetSpeed, unsigned long elapsedMicros)
{
    long maxStep =
        (long)(MOTOR_ACCEL_LIMIT_PER_SECOND * elapsedMicros / 1000000UL);

    if (maxStep < 1)
    {
        maxStep = 1;
    }

    if (currentSpeed == targetSpeed)
    {
        return currentSpeed;
    }

    if (currentSpeed != 0 && targetSpeed != 0 &&
        ((currentSpeed > 0) != (targetSpeed > 0)))
    {
        if (currentSpeed > 0)
        {
            return max(0, currentSpeed - (int)maxStep);
        }

        return min(0, currentSpeed + (int)maxStep);
    }

    if (abs(targetSpeed) > abs(currentSpeed))
    {
        if (targetSpeed > currentSpeed)
        {
            return min(currentSpeed + (int)maxStep, targetSpeed);
        }

        return max(currentSpeed - (int)maxStep, targetSpeed);
    }

    return targetSpeed;
}

void setupMotor(motor &motor, int enablePin, int directionPin1, int directionPin2)
{
    motor.enablePin = enablePin;
    motor.directionPin1 = directionPin1;
    motor.directionPin2 = directionPin2;
    motor.lastCommandMicros = micros();
    motor.signedSpeed = 0;

    pinMode(motor.enablePin, OUTPUT);
    pinMode(motor.directionPin1, OUTPUT);
    pinMode(motor.directionPin2, OUTPUT);
}

void sendToMotor(motor &motor)
{
    analogWrite(motor.enablePin, motor.speed);
    switch (motor.direction)
    {
    case (HALT):
        digitalWrite(motor.directionPin1, LOW);
        digitalWrite(motor.directionPin2, LOW);
        break;

    case (FORWARD):
        digitalWrite(motor.directionPin1, LOW);
        digitalWrite(motor.directionPin2, HIGH);
        break;

    case (BACKWARD):
        digitalWrite(motor.directionPin1, HIGH);
        digitalWrite(motor.directionPin2, LOW);
        break;
    }
}

void setMotorSpeed(motor &motor, int speed)
{
    motor.speed = constrain(speed, 0, 255); // Force values between 0-255 (PWM)
}

byte getMotorSpeed(motor &motor)
{
    return motor.speed;
}

void setMotorDirection(motor &motor, Direction direction)
{
    motor.direction = direction;
}

byte getMotorDirection(motor &motor)
{
    return motor.direction;
}

void sendSignedMotorCommand(motor &motor, int speed)
{
    unsigned long currentMicros = micros();
    unsigned long elapsedMicros = currentMicros - motor.lastCommandMicros;
    motor.lastCommandMicros = currentMicros;

    motor.signedSpeed = limitMotorSpeedChange(
        motor.signedSpeed,
        speed,
        elapsedMicros);

    if (motor.signedSpeed > 0)
    {
        setMotorDirection(motor, FORWARD);
        setMotorSpeed(motor, motor.signedSpeed);
    }
    else if (motor.signedSpeed < 0)
    {
        setMotorDirection(motor, BACKWARD);
        setMotorSpeed(motor, abs(motor.signedSpeed));
    }
    else
    {
        setMotorDirection(motor, HALT);
        setMotorSpeed(motor, 0);
    }

    sendToMotor(motor);
}
