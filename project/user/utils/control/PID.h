#ifndef PID_H_
#define PID_H_

/**
 * PID 控制头文件
 */

typedef struct {
    /**
     * kp
     */
    float kp;
    /**
     * ki
     */
    float ki;
    /**
     * kd
     */
    float kd;
    
    /**
     * I max
     */
    float i_max;
    /**
     * P max
     */
    float p_max;
    /**
     * D max
     */
    float d_max;

    /**
     * Out min
     */
    float out_min;
    /**
     * Out max
     */
    float out_max;

    /**
     * Low Pass
     */
    float low_pass;

    /**
     * 输出的 P
     */
    float out_p;
    /**
     * 输出的 I
     */
    float out_i;
    /**
     * 输出的 D
     */
    float out_d;

    /**
     * 当前的误差
     */
    float error;
    /**
     * 上一次的误差
     */
    float pre_error;
    /**
     * 上上次的误差
     */
    float pre_pre_error;
} pid_param;


float pid_slove(pid_param *pid, float error);

float increment_pid_solve(pid_param *pid, float error);

#endif /* _PID_H_ */