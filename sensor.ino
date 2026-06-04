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
        if (sensorValues[i] > BLACK_THRESHOLD)
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