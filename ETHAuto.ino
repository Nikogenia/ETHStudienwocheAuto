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

const char *classifyJunction(const Junction &junction)
{
    if (!junction.left && !junction.forward && !junction.right)
    {
        return "U-TURN / DEAD END";
    }

    if (junction.forward && junction.left && junction.right)
    {
        return "CROSSROAD";
    }

    if ((junction.left && junction.right) ||
        (junction.forward && (junction.left || junction.right)))
    {
        return "T-JUNCTION";
    }

    if (junction.left || junction.right)
    {
        return "CORNER";
    }

    return "STRAIGHT";
}

void enterGapCheck(RobotState fallbackState)
{
    resumeStateAfterGapCheck = fallbackState;
    gapCheckStartMillis = millis();
    intersectionCandidateStartMillis = 0;
    intersectionArmed = false;
    state = GAP_CHECK;
}

void enterIntersectionState()
{
    state = INTERSECTION;
    intersectionStartMillis = millis();
    intersectionPhaseStartMillis = millis();
    intersectionScanPhase = INTERSECTION_SCAN_CENTER;
    intersectionLogged = false;
    currentJunction = {false, false, false};
    previousError = 0.0;
    lastDirection = 0;
    intersectionCandidateStartMillis = 0;
}

void handleGapCheck(int *sensorValues)
{
    if (!isAllWhite(sensorValues))
    {
        state = resumeStateAfterGapCheck;
        gapCheckStartMillis = 0;
        return;
    }

    unsigned long elapsed = millis() - gapCheckStartMillis;

    sendSignedMotorCommand(
        motorLeft,
        GAP_CHECK_SPEED);

    sendSignedMotorCommand(
        motorRight,
        GAP_CHECK_SPEED);

    if (elapsed >= GAP_CHECK_TIME)
    {
        sendSignedMotorCommand(motorLeft, 0);
        sendSignedMotorCommand(motorRight, 0);
        enterIntersectionState();
    }
}

void startNextIntersectionPhase(IntersectionScanPhase nextPhase)
{
    intersectionScanPhase = nextPhase;
    intersectionPhaseStartMillis = millis();
}

void handleIntersectionState(int *sensorValues)
{
    unsigned long elapsed = millis() - intersectionPhaseStartMillis;
    int advanceTimeCenter = INTERSECTION_ADVANCE_TIME;
    
    switch (intersectionScanPhase)
    {
    case INTERSECTION_SCAN_CENTER:
        sendSignedMotorCommand(
            motorLeft,
            INTERSECTION_CENTER_SPEED);

        sendSignedMotorCommand(
            motorRight,
            INTERSECTION_CENTER_SPEED);

        if (gapCheckStartMillis != 0)
        {
            advanceTimeCenter -= GAP_CHECK_TIME;
        }

        if (elapsed >= advanceTimeCenter)
        {
            sendSignedMotorCommand(motorLeft, 0);
            sendSignedMotorCommand(motorRight, 0);
            gapCheckStartMillis = 0;
            state = STOPPED;
            // currentJunction.forward = isCenterLineVisible(sensorValues);
            // startNextIntersectionPhase(INTERSECTION_SCAN_LEFT_OUT);
        }
        return;

    case INTERSECTION_SCAN_LEFT_OUT:
        sendSignedMotorCommand(
            motorLeft,
            -INTERSECTION_PROBE_SPEED);

        sendSignedMotorCommand(
            motorRight,
            INTERSECTION_PROBE_SPEED);

        if (!isAllWhite(sensorValues))
        {
            currentJunction.left = true;
        }

        if (elapsed >= INTERSECTION_PROBE_TIME)
        {
            sendSignedMotorCommand(motorLeft, 0);
            sendSignedMotorCommand(motorRight, 0);
            startNextIntersectionPhase(INTERSECTION_SCAN_LEFT_BACK);
        }
        return;

    case INTERSECTION_SCAN_LEFT_BACK:
        sendSignedMotorCommand(
            motorLeft,
            INTERSECTION_PROBE_SPEED);

        sendSignedMotorCommand(
            motorRight,
            -INTERSECTION_PROBE_SPEED);

        if (elapsed >= INTERSECTION_PROBE_TIME)
        {
            sendSignedMotorCommand(motorLeft, 0);
            sendSignedMotorCommand(motorRight, 0);
            startNextIntersectionPhase(INTERSECTION_SCAN_RIGHT_OUT);
        }
        return;

    case INTERSECTION_SCAN_RIGHT_OUT:
        sendSignedMotorCommand(
            motorLeft,
            INTERSECTION_PROBE_SPEED);

        sendSignedMotorCommand(
            motorRight,
            -INTERSECTION_PROBE_SPEED);

        if (!isAllWhite(sensorValues))
        {
            currentJunction.right = true;
        }

        if (elapsed >= INTERSECTION_PROBE_TIME)
        {
            sendSignedMotorCommand(motorLeft, 0);
            sendSignedMotorCommand(motorRight, 0);
            startNextIntersectionPhase(INTERSECTION_SCAN_RIGHT_BACK);
        }
        return;

    case INTERSECTION_SCAN_RIGHT_BACK:
        sendSignedMotorCommand(
            motorLeft,
            -INTERSECTION_PROBE_SPEED);

        sendSignedMotorCommand(
            motorRight,
            INTERSECTION_PROBE_SPEED);

        if (elapsed >= INTERSECTION_PROBE_TIME)
        {
            sendSignedMotorCommand(motorLeft, 0);
            sendSignedMotorCommand(motorRight, 0);
            startNextIntersectionPhase(INTERSECTION_SCAN_DONE);
        }
        return;

    case INTERSECTION_SCAN_DONE:
    default:
        if (!intersectionLogged)
        {
            printJunction(currentJunction);
            Serial.println(classifyJunction(currentJunction));
            intersectionLogged = true;
        }

        if (currentJunction.forward)
        {
            state = FOLLOW_LINE;
        }
        else
        {
            if (currentJunction.left)
            {
                lastDirection = -1;
            }
            else if (currentJunction.right)
            {
                lastDirection = 1;
            }
            else
            {
                lastDirection = 0;
            }

            state = SEARCH_LINE;
        }

        intersectionCandidateStartMillis = 0;
        intersectionArmed = false;
        return;
    }
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
    if (state == GAP_CHECK)
    {
        handleGapCheck(sensorValues);
        showRobotState(state);
        return;
    }

    bool allWhite = isAllWhite(sensorValues);

    if (state != CALIBRATION && state != STOPPED)
    {
        if (allWhite)
        {
            if (state != GAP_CHECK)
            {
                enterGapCheck(state);
            }

            handleGapCheck(sensorValues);
            showRobotState(state);
            return;
        }

        bool intersectionCandidate = isIntersectionCandidate(sensorValues);

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
