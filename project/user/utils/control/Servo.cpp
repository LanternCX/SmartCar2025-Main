#include <iostream>

#include "zf_common_headfile.h"

#include "Servo.h"
#include "PID.h"
#include "LowPass.h"


struct pwm_info servo_pwm_info;
pid_param_t pid;
LowPassFilter centerLowPass;

void initServoPID(pid_param_t &pid){
    pid.kp = 0.3;
    pid.ki = 0.00;
    pid.kd = 0.20;

    pid.p_max = 30.0;
    pid.i_max = 30.0;
    pid.d_max = 30.0;

    pid.out_min = -15.0;
    pid.out_max = 15.0;

    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;

    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}

void initServoLowPass(LowPassFilter &lowpass){
    lowpass.alpha = 0.1;
    lowpass.last = 0.0;
}

void setServoDuty(int duty){
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(duty));
}

void servoToCenter(int now, int target){
    // now = lowPassFilter(&centerLowPass, (float) now);
    int error = target - now;
    int duty = 90;
    if(std::abs(error) < 50){
        duty = pid_solve(&pid, error);
        std::cerr << "target: " << target << ' ';
        std::cerr << "now: " << now << ' '; 
        std::cerr << "duty: " << duty << ' ';
        std::cerr << "error: " << error << '\n';
    }
    setServoDuty(90 + duty);
}

void servoInit(){
    // 获取PWM设备信息
    pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
    initServoPID(pid);
    initServoLowPass(centerLowPass);
}