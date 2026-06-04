void ledColor(byte r, byte g, byte b)
{
    analogWrite(LRED, 255 - r);
    analogWrite(LBLUE, 255 - g);
    analogWrite(LGREEN, 255 - b);
}

void ledOff()
{
    ledColor(0, 0, 0);
}

void showRobotState(RobotState currentState)
{
    switch (currentState)
    {
    case CALIBRATION:
        ledColor(0, 255, 255);
        break;
    case FOLLOW_LINE:
        ledColor(0, 255, 0);
        break;
    case SHARP_TURN:
        ledColor(255, 0, 128);
        break;
    case GAP_CHECK:
        ledColor(0, 128, 255);
        break;
    case SEARCH_LINE:
        ledColor(0, 0, 255);
        break;
    case STOPPED:
        ledColor(255, 0, 0);
        break;
    case INTERSECTION:
        ledColor(255, 255, 0);
        break;
    }
}
