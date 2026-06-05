#include "constants.h"

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

    lastLoopMicros = micros();
}

void motorTimeout(int timeout = MOTOR_TIMEOUT)
{
    delay(timeout);

    sendSignedMotorCommand(
        motorLeft,
        0);

    sendSignedMotorCommand(
        motorRight,
        0);

    delay(timeout);
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

    motorTimeout(10);
}

void searchForLineLeft()
{
    sendSignedMotorCommand(
        motorLeft,
        ROTATION_SPEED);

    sendSignedMotorCommand(
        motorRight,
        -ROTATION_SPEED);

    motorTimeout();
}

void searchForLineRight()
{
    sendSignedMotorCommand(
        motorLeft,
        -ROTATION_SPEED);

    sendSignedMotorCommand(
        motorRight,
        ROTATION_SPEED);

    motorTimeout();
}

void startSearchRecovery()
{
    state = SEARCH_LINE_RECOVERY;
    searchLineRecoveryStart = millis();
    searchLineStart = 0;
}

void startIntersection()
{
    state = INTERSECTION;
    intersectionStart = millis();
}

void checkIntersection(int *sensorValues)
{
    if (isAllBlack(sensorValues))
    {
        currentJunction.left = 1;
        currentJunction.right = 1;
        startIntersection();
    }
    else if (isSensorBlack(sensorValues[0]) &&
             isSensorBlack(sensorValues[1]) &&
             isSensorBlack(sensorValues[2]))
    {
        currentJunction.left = 1;
        startIntersection();
    }
    else if (
        isSensorBlack(sensorValues[3]) &&
        isSensorBlack(sensorValues[4]) &&
        isSensorBlack(sensorValues[5]))
    {
        currentJunction.right = 1;
        startIntersection();
    }
}

void decideIntersectionLefthand()
{
    if (currentJunction.left)
    {
        state = SEARCH_LINE_LEFT;
        restrictToLeft = true;
        if (currentJunction.left > 20)
        {
            searchLineStart = millis() - 200;
        }
        else
        {
            searchLineStart = millis();
        }
    }
    else if (currentJunction.forward)
    {
        state = FOLLOW_LINE;
    }
    else if (currentJunction.right)
    {
        state = SEARCH_LINE_RIGHT;
        if (currentJunction.right > 20)
        {
            searchLineStart = millis() - 200;
        }
        else
        {
            searchLineStart = millis();
        }
    }
    else
    {
        state = SEARCH_LINE_LEFT;
        restrictToLeft = false;
    }

    currentJunction = {0, 0, 0};
    intersectionStart = 0;
}

void decideIntersection()
{
    if (INTERSECTION_DECISION_MODE == TREMAUX)
    {
        decideIntersectionTremaux();
    }
    else
    {
        decideIntersectionLefthand();
    }
}

void handleIntersection(int *sensorValues, double dt)
{
    if (millis() - intersectionStart > INTERSECTION_CENTER_TIME)
    {
        if (!isAllWhite(sensorValues))
        {
            currentJunction.forward = 1;
        }
        decideIntersection();
        return;
    }

    if (isSensorBlack(sensorValues[0]) &&
        isSensorBlack(sensorValues[1]) &&
        isSensorBlack(sensorValues[2]))
    {
        currentJunction.left += 1;
    }
    else if (
        isSensorBlack(sensorValues[3]) &&
        isSensorBlack(sensorValues[4]) &&
        isSensorBlack(sensorValues[5]))
    {
        currentJunction.right += 1;
    }
    else if (isSensorBlack(sensorValues[0]) && isSensorBlack(sensorValues[1] && !isSensorBlack(sensorValues[2])))
    {
        currentJunction.left += 6;
    }
    else if (!isSensorBlack(sensorValues[3]) && isSensorBlack(sensorValues[4]) && isSensorBlack(sensorValues[5]))
    {
        currentJunction.right += 6;
    }

    if (isAllWhite(sensorValues))
    {
        sendSignedMotorCommand(
            motorLeft,
            BASE_SPEED);
        sendSignedMotorCommand(
            motorRight,
            BASE_SPEED);

        motorTimeout();
    }
    else
    {
        long valueSum;
        long distanceSum;
        double error =
            computeLineError(
                sensorValues,
                valueSum);
        drivePD(error, dt);
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

        if (millis() - allBlackStart > BLACK_HOLD_TIME)
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

        state = FOLLOW_LINE;
        drivePD(error, dt);

        if (millis() >= followLineIntersectionLockoutUntil)
        {
            checkIntersection(sensorValues);

            if (isAllWhite(sensorValues))
            {
                startIntersection();
            }
        }
    }

    // ------------------
    // INTERSECTION
    // ------------------

    else if (state == INTERSECTION)
    {
        handleIntersection(sensorValues, dt);
    }

    // ------------------
    // SEARCH LINE
    // ------------------

    else if (state == SEARCH_LINE_LEFT || state == SEARCH_LINE_RIGHT)
    {
        unsigned long turnTime = millis() - searchLineStart;
        if (turnTime > SEARCH_LINE_TIMEOUT && !isAllWhite(sensorValues))
        {
            if (restrictToLeft)
            {
                if (turnTime < 1000 || turnTime > 2400)
                {
                    startSearchRecovery();
                }
            }
            else
            {
                startSearchRecovery();
            }
        }
        else
        {
            if (state == SEARCH_LINE_LEFT)
            {
                searchForLineLeft();
            }
            else
            {
                searchForLineRight();
            }
        }
    }

    // ------------------
    // SEARCH LINE RECOVERY
    // ------------------

    else if (state == SEARCH_LINE_RECOVERY)
    {
        if (millis() - searchLineRecoveryStart < SEARCH_LINE_RECOVERY_TIME)
        {
            sendSignedMotorCommand(
                motorLeft,
                -BASE_SPEED);

            sendSignedMotorCommand(
                motorRight,
                -BASE_SPEED);

            motorTimeout();
        }
        else
        {
            sendSignedMotorCommand(
                motorLeft,
                0);

            sendSignedMotorCommand(
                motorRight,
                0);

            followLineIntersectionLockoutUntil =
                millis() + FOLLOW_LINE_INTERSECTION_LOCKOUT;

            state = FOLLOW_LINE;
            searchLineRecoveryStart = 0;
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
