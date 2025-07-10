#include "PID.h"
#include "Math.h"

/**
 * @file PID.cpp
 * @brief PID 控制相关函数
 * @author Cao Xin
 * @date 2025-04-05
 */

/**
 * @brief 位置式 PID 控制器
 * @param param 控制参数结构体
 * @param error 与目标的误差值
 * @return 滤波之后的数据
 * @author Cao Xin
 * @date 2025-04-05
 */
float pid_slove(pid_param *pid, float error) {
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = error;
    pid->out_i += error;

    if (pid->ki != 0) pid->out_i = clip(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki);

    return clip(pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d, pid->out_min, pid->out_max);
}

/**
 * @brief 增量式 PID 控制器
 * @param param 控制参数结构体
 * @param error 与目标的误差值
 * @return 滤波之后的数据
 * @author Cao Xin
 * @date 2025-04-05
 */
float increment_pid_solve(pid_param *pid, float error) {
    pid->out_d = clip(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = clip(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = clip(pid->ki * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return clip(pid->out_p + pid->out_i + pid->out_d, pid->out_min, pid->out_max);
}

float change_kib = 4;
//变积分PID，e大i小
float changable_pid_solve(pid_param *pid, float error) {
    pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error);
    pid->out_p = pid->kp * (error - pid->pre_error);
    float ki_index = pid->ki;
    if (error + pid->pre_error > 0) {
        ki_index = (pid->ki) - (pid->ki) / (1 + exp(change_kib - 0.2 * fabs(error)));    //变积分控制
    }

    pid->out_i = ki_index * error;
    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return clip(pid->out_p, -pid->p_max, pid->p_max)
           + clip(pid->out_i, -pid->i_max, pid->i_max)
           + clip(pid->out_d, -pid->d_max, pid->d_max);
}

float bangbang_out = 0;

float bangbang_pid_solve(pid_param *pid, float error, float BangBang_output, float BangBang_error) {
    pid->error = error;

    //BangBang
    if (error > BangBang_error || error < -BangBang_error) {
        bangbang_out = (error > 0) ? BangBang_output : (-BangBang_output);
    } else {
        pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error);

        pid->out_p = pid->kp * (error - pid->pre_error);

        pid->out_i = pid->ki * error;

        bangbang_out = clip(pid->out_p, -pid->p_max, pid->p_max)
                       + clip(pid->out_i, -pid->i_max, pid->i_max)
                       + clip(pid->out_d, -pid->d_max, pid->d_max);

    }
    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return bangbang_out;
}