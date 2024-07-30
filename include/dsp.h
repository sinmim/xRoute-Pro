#ifndef _DSP_H_INCLUDED
#define _DSP_H_INCLUDED
extern char RAMP_BUSSY_FLAG;

float LOW_PASS_FILTER(float val, float last, double lpfVal);
float RAMPIT(float val,float last,float step);
float absf(float a);
float scaleInto(float n,float maxOfn,float targetMax);
float limit(float number, float max);
float sigmoid(float x, float max, float base, float offset);

#endif
