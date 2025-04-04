#include "PID.h"
#include <algorithm>
#include <cmath>
#include <iostream>

float minmax(float a, float min, float max){
    if(a < min){
        return min;
    }
    if(a > max){
        return max;
    }
    return a;
}
// 常规PID
float pid_solve(pid_param_t *pid, float error) {
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = error;
    pid->out_i += error;

    if (pid->ki != 0) pid->out_i = minmax(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki);

    return pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d;
}

// 增量式PID
float increment_pid_solve(pid_param_t *pid, float error) {
    pid->out_d = minmax(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = minmax(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = minmax(pid->ki * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return pid->out_p + pid->out_i + pid->out_d;
}


float change_kib = 4;

//变积分PID，e大i小
float changable_pid_solve(pid_param_t *pid, float error) {
    pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error);
    pid->out_p = pid->kp * (error - pid->pre_error);
    float ki_index = pid->ki;
    if (error + pid->pre_error > 0) {
        ki_index = (pid->ki) - (pid->ki) / (1 + exp(change_kib - 0.2 * std::abs(error)));    //变积分控制
    }

    pid->out_i = ki_index * error;
    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return minmax(pid->out_p, -pid->p_max, pid->p_max)
           + minmax(pid->out_i, -pid->i_max, pid->i_max)
           + minmax(pid->out_d, -pid->d_max, pid->d_max);
}
