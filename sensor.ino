#include "constants.h"

int *readSensor()
{
    static int values[6] = {0, 0, 0, 0, 0, 0};
    values[0] = analogRead(SENSOR1);
    values[1] = analogRead(SENSOR2);
    values[2] = analogRead(SENSOR3);
    values[3] = analogRead(SENSOR4);
    values[4] = analogRead(SENSOR5);
    values[5] = analogRead(SENSOR6);
    return values;
}

bool isSensorBlack(int sensorValue)
{
    return sensorValue <= BLACK_THRESHOLD;
}

int countBlackSensors(int *sensorValues)
{
    int blackCount = 0;

    for (int i = 0; i < 6; i++)
    {
        if (isSensorBlack(sensorValues[i]))
        {
            blackCount++;
        }
    }

    return blackCount;
}

bool isCenterLineVisible(int *sensorValues)
{
    return isSensorBlack(sensorValues[2]) ||
           isSensorBlack(sensorValues[3]);
}

bool isIntersectionCandidate(int *sensorValues)
{
    int blackCount = countBlackSensors(sensorValues);

    if (isAllBlack(sensorValues))
    {
        return false;
    }

    return blackCount >= INTERSECTION_BLACK_COUNT_THRESHOLD;
}

Junction readJunctionSnapshot(int *sensorValues)
{
    Junction junction;

    junction.left =
        isSensorBlack(sensorValues[0]) ||
        isSensorBlack(sensorValues[1]);

    junction.forward =
        isSensorBlack(sensorValues[2]) ||
        isSensorBlack(sensorValues[3]);

    junction.right =
        isSensorBlack(sensorValues[4]) ||
        isSensorBlack(sensorValues[5]);

    return junction;
}

void printJunction(const Junction &junction)
{
    Serial.print("Junction: L=");
    Serial.print(junction.left ? "1" : "0");
    Serial.print(" F=");
    Serial.print(junction.forward ? "1" : "0");
    Serial.print(" R=");
    Serial.println(junction.right ? "1" : "0");
}

double computeLineError(
    int *sensorValues,
    long &valueSum)
{
    long weightedSum = 0;
    valueSum = 0;

    for (int i = 0; i < 6; i++)
    {
        int lineStrength = 1023 - sensorValues[i];

        weightedSum +=
            (long)lineStrength * WEIGHTS[i];

        valueSum += lineStrength;
    }

    if (valueSum == 0)
        return 0;

    return (double)weightedSum /
           (double)valueSum;
}

double computeLineDistanceFromCenter(
    int *sensorValues,
    long &valueSum)
{
    long distanceSum = 0;
    valueSum = 0;

    for (int i = 0; i < 6; i++)
    {
        int lineStrength = 1023 - sensorValues[i];

        distanceSum +=
            (long)lineStrength * abs(WEIGHTS[i]);

        valueSum += lineStrength;
    }

    if (valueSum == 0)
        return 0;

    return (double)distanceSum /
           (double)valueSum;
}

bool isAllWhite(int *sensorValues)
{
    for (int i = 0; i < 6; i++)
    {
        if (sensorValues[i] < WHITE_THRESHOLD)
        {
            return false;
        }
    }

    return true;
}

bool isAllBlack(int *sensorValues)
{
    for (int i = 0; i < 6; i++)
    {
        if (!isSensorBlack(sensorValues[i]))
        {
            return false;
        }
    }

    return true;
}

void addErrorSample(double error, unsigned long currentMillis)
{
    errorHistory[errorHistoryIndex].error = error;
    errorHistory[errorHistoryIndex].timestamp = currentMillis;
    errorHistoryIndex = (errorHistoryIndex + 1) % ERROR_HISTORY_SIZE;
}

double getErrorSum200ms(unsigned long currentMillis)
{
    double sum = 0;
    unsigned long windowStart = currentMillis - ERROR_HISTORY_WINDOW;

    for (int i = 0; i < ERROR_HISTORY_SIZE; i++)
    {
        if (errorHistory[i].timestamp >= windowStart &&
            errorHistory[i].timestamp <= currentMillis)
        {
            sum += errorHistory[i].error;
        }
    }

    return sum;
}