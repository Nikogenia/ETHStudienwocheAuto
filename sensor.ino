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

bool isSensorWhite(int sensorValue)
{
    return sensorValue >= WHITE_THRESHOLD;
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

bool isAllWhite(int *sensorValues)
{
    for (int i = 0; i < 6; i++)
    {
        if (!isSensorWhite(sensorValues[i]))
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
