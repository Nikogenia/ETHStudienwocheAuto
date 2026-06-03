#include "constants.h"

void setupMotor(motor &motor, int enablePin, int directionPin1, int directionPin2)
{
    motor.enablePin = enablePin;
    motor.directionPin1 = directionPin1;
    motor.directionPin2 = directionPin2;

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

void setMotorDirection(motor &motor, byte direction)
{
    motor.direction = direction;
}

byte getMotorDirection(motor &motor)
{
    return motor.direction;
}
