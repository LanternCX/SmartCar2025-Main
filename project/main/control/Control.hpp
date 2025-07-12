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
  int center;
  int left;
  int right;
} speed_param;

/**
 * @brief 控制参数
 */
struct control_param {
    float kp;
    float kd;
    float gyro_p;
    int gain;
    int speed;

    control_param(float _kp, float _kd, float _gyro_p, int _gain, int _speed)
        : kp(_kp), kd(_kd), gyro_p(_gyro_p), gain(_gain), speed(_speed){}
    control_param(){}
};



void to_center(int now, int target);
void control_init(int line_speed, int curve_speed);
void calc_speed_det(const int & angle, const int & v_center, int & v_in, int & v_out);
// void set_statue(ElementType type);