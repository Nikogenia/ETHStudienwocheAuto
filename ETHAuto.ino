#include "constants.h"

motor motorLeft, motorRight;

const double SHARP_TURN_DISTANCE = 60.0;
const double MAX_SENSOR_DISTANCE = 100.0;

void driveSharpTurn(double error, double distanceFromCenter)
{
    double normalizedDistance =
        constrain(
            distanceFromCenter /
                MAX_SENSOR_DISTANCE,
            0.0,
            1.0);

    int outsideSpeed =
        constrain(
            BASE_SPEED +
                (int)(BASE_SPEED * 0.20 * normalizedDistance),
            0,
            255);

    int insideSpeed =
        BASE_SPEED -
        (int)(BASE_SPEED * (0.85 + 0.35 * normalizedDistance));

    if (insideSpeed > 255)
        insideSpeed = 255;

    if (error < 0)
    {
        sendSignedMotorCommand(
            motorLeft,
            insideSpeed);

        sendSignedMotorCommand(
            motorRight,
            outsideSpeed);
    }
    else
    {
        sendSignedMotorCommand(
            motorLeft,
            outsideSpeed);

        sendSignedMotorCommand(
            motorRight,
            insideSpeed);
    }

    previousError = error;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    setupMotor(motorLeft, ENA, IN1, IN2);
    setupMotor(motorRight, ENB, IN3, IN4);
    setMotorSpeed(motorLeft, BASE_SPEED);
    setMotorSpeed(motorRight, BASE_SPEED);
    pinMode(LRED, OUTPUT);
    pinMode(LGREEN, OUTPUT);
    pinMode(LBLUE, OUTPUT);
    showRobotState(state);

    Serial.begin(115200);
    lastLoopMicros = micros();
}

void drivePD(double error, double dt)
{
    double derivative =
        (error - previousError) / dt;

    double turn =
        KP * error +
        KD * derivative;

    previousError = error;

    int left =
        BASE_SPEED - (int)turn;

    int right =
        BASE_SPEED + (int)turn;

    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    sendSignedMotorCommand(
        motorLeft,
        left);

    sendSignedMotorCommand(
        motorRight,
        right);
}

void searchForLine()
{
    if (lastKnownError < 0)
    {
        sendSignedMotorCommand(
            motorLeft,
            -SEARCH_SPEED);

        sendSignedMotorCommand(
            motorRight,
            SEARCH_SPEED);
    }
    else
    {
        sendSignedMotorCommand(
            motorLeft,
            SEARCH_SPEED);

        sendSignedMotorCommand(
            motorRight,
            -SEARCH_SPEED);
    }
}

void loop()
{
    unsigned long currentMicros = micros();

    double dt =
        (currentMicros - lastLoopMicros) /
        1000000.0;

    lastLoopMicros = currentMicros;

    if (dt <= 0)
        dt = 0.001;

    int *sensorValues = readSensor();

    // ------------------
    // STOP MARKER
    // ------------------

    if (isAllBlack(sensorValues))
    {
        if (allBlackStart == 0)
        {
            allBlackStart = millis();
        }

        if (millis() - allBlackStart >
            BLACK_HOLD_TIME)
        {
            state = STOPPED;
        }
    }
    else
    {
        allBlackStart = 0;
    }

    // ------------------
    // STOPPED
    // ------------------

    if (state == STOPPED)
    {
        showRobotState(state);

        sendSignedMotorCommand(
            motorLeft,
            0);

        sendSignedMotorCommand(
            motorRight,
            0);

        return;
    }

    // ------------------
    // LOST LINE
    // ------------------

    if (isAllWhite(sensorValues))
    {
        state = SEARCH_LINE;
    }

    // ------------------
    // FOLLOW LINE
    // ------------------

    if (state == FOLLOW_LINE)
    {
        long valueSum;
        long distanceSum;

        double error =
            computeLineError(
                sensorValues,
                valueSum);

        double distanceFromCenter =
            computeLineDistanceFromCenter(
                sensorValues,
                distanceSum);

        lastKnownError = error;

        if (distanceFromCenter > SHARP_TURN_DISTANCE)
        {
            driveSharpTurn(
                error,
                distanceFromCenter);
        }
        else
        {
            drivePD(error, dt);
        }
    }

    // ------------------
    // SEARCH LINE
    // ------------------

    else if (state == SEARCH_LINE)
    {
        if (!isAllWhite(sensorValues))
        {
            state = FOLLOW_LINE;
        }
        else
        {
            searchForLine();
        }
    }

    showRobotState(state);

}

/*
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
*/
