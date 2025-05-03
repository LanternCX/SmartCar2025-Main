#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "Vision.h"
/**
 * @file PID.h
 * @brief 方向控制主要操作
 * @author Cao Xin
 * @date 2025-04-06
 */

 /**
  * @brief 速度参数
  */
typedef struct{
    // 直线速度
    int line_speed;
    // 弯道速度
    int curve_speed;
    // 当前速度
    int current;
} speed_param;

void to_center(int now, int target);
void control_init(int line_speed, int curve_speed);
void set_statue(Type type);
#endif