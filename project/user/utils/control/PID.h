#ifndef PID_H_
#define PID_H_

typedef struct {
    float kp;    //P
    float ki;    //I
    float kd;    //D
    float i_max; //integrator_max
    float p_max; //integrator_max
    float d_max; //integrator_max

    float low_pass;

    float out_p;
    float out_i;
    float out_d;

    float error;
    float pre_error;
    float pre_pre_error;
} pid_param_t;


float pid_solve(pid_param_t *pid, float error);

float increment_pid_solve(pid_param_t *pid, float error);

float bangbang_pid_solve(pid_param_t *pid, float error);

float changable_pid_solve(pid_param_t *pid, float error);

#endif /* _PID_H_ */