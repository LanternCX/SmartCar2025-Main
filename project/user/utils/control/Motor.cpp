#include <algorithm>
#include <iostream>

#include "Motor.h"
#include "PID.h"
#include "LowPass.h"
#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#include "zf_device_ips200_fb.h"

// Motor PWM Info
struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;

// PID
pid_param_t leftPID;
pid_param_t rightPID;

// Low Pass Filter
LowPassFilter leftLowPass;
LowPassFilter rightLowPass;

// Duty
static int leftDuty = 10;
static int rightDuty = 10;

float min(float a, float b){
    return a > b ? b : a;
}

float max(float a, float b){
    return a < b ? b : a;
}

void initMotorPID(pid_param_t &pid){
    pid.kp = 0.030;
    pid.ki = 0.045;
    pid.kd = 0.025;
    pid.p_max = 30.0;
    pid.i_max = 30.0;
    pid.d_max = 30.0;
    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;
    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}

void initLowPass(LowPassFilter &lowpass){
    lowpass.alpha = 0.3;
    lowpass.last = 0.0;
}

void leftMotorRun(int duty, int dir){
    duty = min(duty, 40);
    gpio_set_level(MOTOR1_DIR, dir);
    pwm_set_duty(MOTOR1_PWM, duty * (MOTOR1_PWM_DUTY_MAX / 100));
}

void rightMotorRun(int duty, int dir){
    duty = min(duty, 40);
    gpio_set_level(MOTOR2_DIR, dir);
    pwm_set_duty(MOTOR2_PWM, duty * (MOTOR2_PWM_DUTY_MAX / 100));
}

void motorInit(){
    // 获取PWM设备信息
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);

    initMotorPID(leftPID);
    initMotorPID(rightPID);

    initLowPass(leftLowPass);
    initLowPass(rightLowPass);

    leftMotorRun(15, 1);
    rightMotorRun(15, 1);
    
    if(MOTOR_DEBUG){
        std::cout << "Motor Param:" << '\n';
        std::cout << "P: " << leftPID.kp;
        std::cout << "I: " << leftPID.ki;
        std::cout << "D: " << leftPID.kd << '\n';
        std::cout << "Alpha: " << leftLowPass.alpha << '\n';
    }
}

void setLeftSpeed(int speed){
    float now = encoder_get_count(ENCODER_2);
    now = lowPassFilter(&leftLowPass, now);
    float error = speed - now;
    float det = increment_pid_solve(&leftPID, error);

    leftDuty += det;
    
    leftMotorRun(abs(leftDuty), leftDuty > 0 ? 1 : -1);

    if(MOTOR_DEBUG){
        static int cnt1 = 0;
        static int leftOut[100];
        cnt1++;
        cnt1 %= 100;
        std::cout << "duty-l: " <<  leftDuty << " now-l: " << now << " target-l: " << speed << " error-l: " << error << '\n';
        leftOut[cnt1] = now;
        for(int i = 0; i < 100; i++){
            ips200_draw_point((10 + i), (uint16)max(320 - leftOut[i] / 3.0, 1), 0x00FF);
            ips200_draw_point((10 + i), (uint16)max(320 - speed / 3.0, 1), 0x00FF);
        }
    }

}

void setRightSpeed(int speed){
    float now = -encoder_get_count(ENCODER_1);
    now = lowPassFilter(&rightLowPass, now);
    float error = speed - now;
    float det = increment_pid_solve(&rightPID, error);

    rightDuty += det;

    rightMotorRun(abs(rightDuty), rightDuty > 0 ? 1 : -1);

    if(MOTOR_DEBUG){
        static int cnt2 = 0;
        static int rightOut[100];
        cnt2++;
        cnt2 %= 100;
        std::cout << "duty-r: " <<  rightDuty << " now-r: " << now << " target-l: " << speed << " error-r: " << error << '\n';
        rightOut[cnt2] = now;
        for(int i = 0; i < 100; i++){
            ips200_draw_point((10 + i), (uint16)max(220 - rightOut[i] / 3.0, 1), 0xF800);
            ips200_draw_point((10 + i), (uint16)max(220 - speed / 3.0, 1), 0xF800);
        }
    }
}
