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

long calculateValueSum(int input[6]) {
    long valueSum = 0;

    for (int index = 0; index < 6; index++)
    {
        valueSum += input[index];
    }

    return valueSum;
}

int calculateLineOffset(int input[6])
{
    long weightedSum = 0;
    long valueSum = calculateValueSum(input);

    for (int index = 0; index < 6; index++)
    {
        weightedSum += (long)input[index] * WEIGHTS[index];
    }

    if (valueSum == 0)
    {
        return 0;
    }

    long rounded = 0;
    if (weightedSum >= 0)
    {
        rounded = (weightedSum + valueSum / 2) / valueSum;
    }
    else
    {
        rounded = (weightedSum - valueSum / 2) / valueSum;
    }

    return (int)rounded;
}

double calculatePDCorrection(double currentError, double previousError, double dt)
{
    double derivative = 0.0;

    if (dt > 0.0)
    {
        derivative = (currentError - previousError) / dt;
    }

    return (currentError * Kp) + (derivative * Kd);
}
