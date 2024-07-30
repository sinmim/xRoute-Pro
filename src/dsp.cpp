#include <math.h>
char RAMP_BUSSY_FLAG = 0;

float absf(float a)
{
    if (a > 0)
        return a;
    else
        return -a;
}

float LOW_PASS_FILTER(float val, float last, double lpfVal)
{
    return last * lpfVal + val * (1 - lpfVal);
}

float RAMPIT(float val, float last, float step)
{
    if (absf(last - val) < step)
    {
        RAMP_BUSSY_FLAG = 0;
        return last;
    }
    RAMP_BUSSY_FLAG = 1;
    if (last < val)
        last += step;
    else
        last -= step;
    return last;
}
float scaleInto(float n, float maxOfn, float targetMax)
{
    float val = n * targetMax / maxOfn;
    if (val > targetMax)
        return targetMax;
    else
        return (n * targetMax / maxOfn);
}
float limit(float number, float max)
{
  if (number > max)
    return max;
  else
    return number;
}
float sigmoid(float x, float max, float base, float offset)
{
  return (max / (1 + (pow(base, (x - offset)))));
}
