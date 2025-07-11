/**
 * @file PID.h
 * @brief 方向控制主要操作
 * @author Cao Xin
 * @date 2025-04-06
 */

#pragma once

// #include "Vision.h"

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

struct control_param {
    float kp;
    float kd;
    float gyro_p;
    int gain;

    control_param(float _kp, float _kd, float _gyro_p, int _gain)
        : kp(_kp), kd(_kd), gyro_p(_gyro_p), gain(_gain){}
    control_param(){}
};



void to_center(int now, int target);
void control_init(int line_speed, int curve_speed);
// void set_statue(ElementType type);