#ifndef _LOW_PASS_H_
#define _LOW_PASS_H_

typedef struct {
    float last;
    float alpha;
} LowPassFilter;

float lowPassFilter(LowPassFilter * param, float now);
#endif