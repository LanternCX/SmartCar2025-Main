#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "Vision.h"

typedef struct{
    int line_speed;
    int curve_speed;
    int current;
} speed_param;
void to_center(int now, int target);
void control_init(int v);
void set_statue(Type type);
#endif