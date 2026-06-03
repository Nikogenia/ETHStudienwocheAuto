int rainbowIndex = 0;

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

void rainbowStep()
{
    rainbowIndex = (rainbowIndex + 1) % 1530;
    byte r, g, b;
    if (rainbowIndex < 255)
    {
        r = 255;
        g = rainbowIndex;
        b = 0;
    }
    else if (rainbowIndex < 510)
    {
        r = 255 - (rainbowIndex - 255);
        g = 255;
        b = 0;
    }
    else if (rainbowIndex < 765)
    {
        r = 0;
        g = 255;
        b = rainbowIndex - 510;
    }
    else if (rainbowIndex < 1020)
    {
        r = 0;
        g = 255 - (rainbowIndex - 765);
        b = 255;
    }
    else if (rainbowIndex < 1275)
    {
        r = rainbowIndex - 1020;
        g = 0;
        b = 255;
    }
    else
    {
        r = 255;
        g = 0;
        b = 255 - (rainbowIndex - 1275);
    }
    ledColor(r, g, b);
}
