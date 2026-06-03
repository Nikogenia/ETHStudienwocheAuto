#include "constants.h"

motor motorLeft, motorRight;
double previousError = 0.0;
double smoothLeftSpeed = BASE_SPEED;
double smoothRightSpeed = BASE_SPEED;
unsigned long lastLoopMicros = 0;
int lastLineSide = 0;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    setupMotor(motorLeft, ENA, IN1, IN2);
    setupMotor(motorRight, ENB, IN3, IN4);
    setMotorSpeed(motorLeft, BASE_SPEED);
    setMotorSpeed(motorRight, BASE_SPEED);

    Serial.begin(9600);
    lastLoopMicros = micros();
}

void loop()
{
    unsigned long currentMicros = micros();
    double dt = (currentMicros - lastLoopMicros) / 1000000.0;
    lastLoopMicros = currentMicros;

    if (dt <= 0.0)
    {
        dt = 0.001;
    }

    int *sensorValues = readSensor();
    long valueSum = calculateValueSum(sensorValues);
    int lineOffset = calculateLineOffset(sensorValues);
    long weightedSum = 0;

    for (int index = 0; index < 6; index++)
    {
        weightedSum += (long)sensorValues[index] * WEIGHTS[index];
    }

    if (valueSum > WHITE_THRESHOLD)
    {
        if (lastLineSide == 1)
        {
            sendSignedMotorCommand(motorLeft, 100);
            sendSignedMotorCommand(motorRight, -110);
        }
        else if (lastLineSide == -1)
        {
            sendSignedMotorCommand(motorLeft, -110);
            sendSignedMotorCommand(motorRight, 100);
        }
        else
        {
            sendSignedMotorCommand(motorLeft, -90);
            sendSignedMotorCommand(motorRight, -90);
        }

        smoothLeftSpeed = BASE_SPEED;
        smoothRightSpeed = BASE_SPEED;
    }
    else if (valueSum < BLACK_THRESHOLD)
    {
        sendSignedMotorCommand(motorLeft, 0);
        sendSignedMotorCommand(motorRight, 0);
        smoothLeftSpeed = 0;
        smoothRightSpeed = 0;
    }
    else
    {
        if (weightedSum > 80)
        {
            lastLineSide = 1;
        }
        else if (weightedSum < -80)
        {
            lastLineSide = -1;
        }

        double currentError = (double)weightedSum;
        bool sharpTurnActive = false;
        int targetLeft = BASE_SPEED;
        int targetRight = BASE_SPEED;

        if (currentError > -60 && currentError < 60)
        {
            targetLeft = BASE_SPEED;
            targetRight = BASE_SPEED;
            previousError = currentError;
        }
        else
        {
            double arcCorrection = calculatePDCorrection(currentError, previousError, dt);

            if (arcCorrection > 40)
            {
                targetLeft = constrain(BASE_SPEED + (int)arcCorrection, 0, 255);
                targetRight = constrain(-65 - ((int)arcCorrection / 3), -255, 0);
                sharpTurnActive = true;
            }
            else if (arcCorrection < -40)
            {
                targetRight = constrain(BASE_SPEED - (int)arcCorrection, 0, 255);
                targetLeft = constrain(-65 + ((int)arcCorrection / 3), -255, 0);
                sharpTurnActive = true;
            }
            else
            {
                targetLeft = constrain(BASE_SPEED + (int)arcCorrection, 30, 255);
                targetRight = constrain(BASE_SPEED - (int)arcCorrection, 30, 255);
            }

            previousError = currentError;
        }

        if (sharpTurnActive)
        {
            smoothLeftSpeed = targetLeft;
            smoothRightSpeed = targetRight;
        }
        else
        {
            double filterFactor = 20.0 * dt;
            if (filterFactor > 1.0)
            {
                filterFactor = 1.0;
            }

            smoothLeftSpeed = (targetLeft * filterFactor) + (smoothLeftSpeed * (1.0 - filterFactor));
            smoothRightSpeed = (targetRight * filterFactor) + (smoothRightSpeed * (1.0 - filterFactor));
        }

        if (smoothLeftSpeed >= 0)
        {
            sendSignedMotorCommand(motorLeft, constrain((int)smoothLeftSpeed, 0, 255));
        }
        else
        {
            sendSignedMotorCommand(motorLeft, -constrain((int)abs(smoothLeftSpeed), 0, 255));
        }

        if (smoothRightSpeed >= 0)
        {
            sendSignedMotorCommand(motorRight, constrain((int)smoothRightSpeed, 0, 255));
        }
        else
        {
            sendSignedMotorCommand(motorRight, -constrain((int)abs(smoothRightSpeed), 0, 255));
        }
    }

    Serial.print(sensorValues[0]);
    Serial.print(",");
    Serial.print(sensorValues[1]);
    Serial.print(",");
    Serial.print(sensorValues[2]);
    Serial.print(",");
    Serial.print(sensorValues[3]);
    Serial.print(",");
    Serial.print(sensorValues[4]);
    Serial.print(",");
    Serial.print(sensorValues[5]);
    Serial.print(",");
    Serial.print(lineOffset);
    Serial.print(",");
    Serial.print(weightedSum);
    Serial.print(",");
    Serial.print((int)previousError);
    Serial.print(",");
    Serial.print((int)smoothLeftSpeed);
    Serial.print(",");
    Serial.print((int)smoothRightSpeed);
    Serial.print(",");
    Serial.println(valueSum);

    rainbowStep();
}
