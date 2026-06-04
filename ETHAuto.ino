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

    delay(MOTOR_TIMEOUT);

    sendSignedMotorCommand(
        motorLeft,
        0);

    sendSignedMotorCommand(
        motorRight,
        0);

    delay(MOTOR_TIMEOUT);
}

void searchForLineLeft()
{
    sendSignedMotorCommand(
        motorLeft,
        ROTATION_SPEED);

    sendSignedMotorCommand(
        motorRight,
        -ROTATION_SPEED);

    delay(MOTOR_TIMEOUT);

    sendSignedMotorCommand(
        motorLeft,
        0);

    sendSignedMotorCommand(
        motorRight,
        0);

    delay(MOTOR_TIMEOUT);
}

void searchForLineRight()
{
    sendSignedMotorCommand(
        motorLeft,
        -ROTATION_SPEED);

    sendSignedMotorCommand(
        motorRight,
        ROTATION_SPEED);

    delay(MOTOR_TIMEOUT);

    sendSignedMotorCommand(
        motorLeft,
        0);

    sendSignedMotorCommand(
        motorRight,
        0);

    delay(MOTOR_TIMEOUT);
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
        currentJunction.left = true;
        currentJunction.right = true;
        startIntersection();
    }
    else if (isSensorBlack(sensorValues[0]) &&
             isSensorBlack(sensorValues[1]) &&
             isSensorBlack(sensorValues[2]) &&
             isSensorBlack(sensorValues[3]))
    {
        currentJunction.left = true;
        startIntersection();
    }
    else if (isSensorBlack(sensorValues[2]) &&
             isSensorBlack(sensorValues[3]) &&
             isSensorBlack(sensorValues[4]) &&
             isSensorBlack(sensorValues[5]))
    {
        currentJunction.right = true;
        startIntersection();
    }
}

void decideIntersectionLefthand()
{
    if (currentJunction.left)
    {
        state = SEARCH_LINE_LEFT;
        searchLineStart = millis();
    }
    else if (currentJunction.forward)
    {
        state = FOLLOW_LINE;
    }
    else if (currentJunction.right)
    {
        state = SEARCH_LINE_RIGHT;
        searchLineStart = millis();
    }
    else
    {
        state = SEARCH_LINE_LEFT;
    }
    currentJunction = {false, false, false};
    intersectionStart = 0;
}

void handleIntersection(int *sensorValues, double dt)
{
    if (millis() - intersectionStart > INTERSECTION_CENTER_TIME)
    {
        if (!isAllWhite(sensorValues))
        {
            currentJunction.forward = true;
        }
        decideIntersectionLefthand();
        return;
    }

    if (isAllWhite(sensorValues))
    {
        sendSignedMotorCommand(
            motorLeft,
            BASE_SPEED);
        sendSignedMotorCommand(
            motorRight,
            BASE_SPEED);

        delay(MOTOR_TIMEOUT);

        sendSignedMotorCommand(
            motorLeft,
            0);

        sendSignedMotorCommand(
            motorRight,
            0);

        delay(MOTOR_TIMEOUT);
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

        checkIntersection(sensorValues);

        if (isAllWhite(sensorValues))
        {
            startIntersection();
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
            state = FOLLOW_LINE;
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
