#ifndef MOTOR_H
#define MOTOR_H
#define MOTOR1_DIR "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM "/dev/zf_device_pwm_motor_2"
#define ENCODER_1 "/dev/zf_encoder_1"
#define ENCODER_2 "/dev/zf_encoder_2"

// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX (motor_1_pwm_info.duty_max)       
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
#define MOTOR2_PWM_DUTY_MAX (motor_2_pwm_info.duty_max)        

#define MAX_DUTY (30) // 最大 MAX_DUTY% 占空比

#include "PID.h"
void motorInit();
void leftMotorRun(int duty, int dir);
void rightMotorRun(int duty, int dir);
pid_param_t getPID();
void setPID(float kp, float ki, float kd);
void setLeftSpeed(int speed);
void setRightSpeed(int speed);
#endif
