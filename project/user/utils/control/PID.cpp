#include "PID.h"

float minmax(float a, float min, float max){
    if(a < min){
        return min;
    }
    if(a > max){
        return max;
    }
    return a;
}

// 常规 PID
float pid_solve(pid_param_t *pid, float error) {
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = error;
    pid->out_i += error;

    if (pid->ki != 0) pid->out_i = minmax(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki);

    return minmax(pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d, pid->out_min, pid->out_max);
}

// 增量式 PID
float increment_pid_solve(pid_param_t *pid, float error) {
    pid->out_d = minmax(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = minmax(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = minmax(pid->ki * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return minmax(pid->out_p + pid->out_i + pid->out_d, pid->out_min, pid->out_max);
}
