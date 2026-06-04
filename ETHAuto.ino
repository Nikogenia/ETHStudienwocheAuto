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

void driveSharpPD(double error, double dt)
{
    double derivative =
        (error - previousError) / dt;

    double turn =
        KP * error +
        KD * derivative;

    previousError = error;

    int left = BASE_SPEED;
    int right = BASE_SPEED;

    if (turn > 0)
    {
        left = -SHARP_TURN_SPEED - ((int)turn / 3);
        right = BASE_SPEED + (int)turn;
    }
    else
    {
        left = BASE_SPEED - (int)turn;
        right = -SHARP_TURN_SPEED + ((int)turn / 3);
    }

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
    if (lastDirection < 0)
    {
        sendSignedMotorCommand(
            motorLeft,
            -SEARCH_SPEED);

        sendSignedMotorCommand(
            motorRight,
            SEARCH_SPEED);
    }
    else if (lastDirection > 0)
    {
        sendSignedMotorCommand(
            motorLeft,
            SEARCH_SPEED);

        sendSignedMotorCommand(
            motorRight,
            -SEARCH_SPEED);
    }
    else
    {
        sendSignedMotorCommand(
            motorLeft,
            -SEARCH_SPEED);

        sendSignedMotorCommand(
            motorRight,
            -SEARCH_SPEED);
    }
}

void enterIntersectionState()
{
    state = INTERSECTION;
    intersectionStartMillis = millis();
    intersectionLogged = false;
    previousError = 0.0;
    lastDirection = 0;
}

void handleIntersectionState(int *sensorValues)
{
    unsigned long elapsed = millis() - intersectionStartMillis;

    if (elapsed < INTERSECTION_ADVANCE_TIME)
    {
        sendSignedMotorCommand(
            motorLeft,
            INTERSECTION_CENTER_SPEED);

        sendSignedMotorCommand(
            motorRight,
            INTERSECTION_CENTER_SPEED);

        return;
    }

    sendSignedMotorCommand(
        motorLeft,
        0);

    sendSignedMotorCommand(
        motorRight,
        0);

    if (elapsed < INTERSECTION_ADVANCE_TIME + INTERSECTION_SETTLE_TIME)
    {
        return;
    }

    if (!intersectionLogged)
    {
        currentJunction = readJunctionSnapshot(sensorValues);
        printJunction(currentJunction);
        intersectionLogged = true;
    }

    state = FOLLOW_LINE;
    intersectionCandidateStartMillis = 0;
    intersectionArmed = true;
}

void calibrate()
{
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

    bool intersectionCandidate = false;

    if (state != CALIBRATION && state != STOPPED)
    {
        intersectionCandidate = isIntersectionCandidate(sensorValues);

        if (intersectionCandidate)
        {
            if (intersectionCandidateStartMillis == 0)
            {
                intersectionCandidateStartMillis = millis();
            }

            if (!intersectionArmed &&
                millis() - intersectionCandidateStartMillis >= INTERSECTION_HOLD_TIME)
            {
                intersectionArmed = true;
                enterIntersectionState();
            }
        }
        else
        {
            intersectionCandidateStartMillis = 0;

            if (state != INTERSECTION)
            {
                intersectionArmed = false;
            }
        }
    }

    // ------------------
    // INTERSECTION
    // ------------------

    if (state == INTERSECTION)
    {
        handleIntersectionState(sensorValues);
        showRobotState(state);
        return;
    }

    // ------------------
    // LOST LINE
    // ------------------

    if (state != CALIBRATION && isAllWhite(sensorValues))
    {
        state = SEARCH_LINE;
    }

    // ------------------
    // CALIBRATION
    // ------------------

    if (state == CALIBRATION)
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

        if (calibrationStartMillis == 0)
        {
            calibrationStartMillis = millis();
        }

        sendSignedMotorCommand(
            motorLeft,
            -SEARCH_SPEED);

        sendSignedMotorCommand(
            motorRight,
            SEARCH_SPEED);

        if (fabs(error) > DIRECTION_THRESHOLD ||
            distanceFromCenter > SHARP_TURN_THRESHOLD)
        {
            calibrationLeftStart = true;
        }

        if (calibrationLeftStart &&
            fabs(error) <= DIRECTION_THRESHOLD &&
            distanceFromCenter <= SHARP_TURN_THRESHOLD)
        {
            calibrationRotationMillis =
                millis() - calibrationStartMillis;

            Serial.print("Calibration rotation time (ms): ");
            Serial.println(calibrationRotationMillis);

            calibrationLeftStart = false;
            calibrationStartMillis = 0;
            previousError = 0.0;
            lastDirection = 0;
            state = SEARCH_LINE;
            showRobotState(state);
            return;
        }

        showRobotState(state);
        return;
    }

    // ------------------
    // FOLLOW LINE OR SHARP TURN
    // ------------------

    if (state == FOLLOW_LINE || state == SHARP_TURN)
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

        if (error > DIRECTION_THRESHOLD)
        {
            lastDirection = 1;
            lastDirectionStart = millis();
        }
        else if (error < -DIRECTION_THRESHOLD)
        {
            lastDirection = -1;
            lastDirectionStart = millis();
        }
        else
        {
            if (millis() - lastDirectionStart >
                LAST_DIRECTION_TIMEOUT)
            {
                lastDirection = 0;
            }
        }

        unsigned long currentMillis = millis();
        addErrorSample(error, currentMillis);
        double errorSum = getErrorSum200ms(currentMillis);

        if (fabs(errorSum) > SHARP_TURN_THRESHOLD)
        {
            state = SHARP_TURN;
            driveSharpPD(error, dt);
        }
        else
        {
            state = FOLLOW_LINE;
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
