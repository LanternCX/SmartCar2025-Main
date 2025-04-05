#include "LowPass.h"
float lowPassFilter(LowPassFilter * param, float now) {
    float alpha = param->alpha;
    float last = param->last;
    float output = alpha * now + (1 - alpha) * last;
    last = output;
    return output;
}