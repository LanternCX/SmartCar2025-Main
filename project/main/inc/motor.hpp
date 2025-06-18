#ifndef MOTOR_H
#define MOTOR_H

#include"base.hpp"

#define GPIO_MUX_BASE_ADDR 0x16000490
#define GTIM_BASE_ADDR 0x16119000
#define GTIM_CR1_OFFSET 0x00
#define GTIM_CR2_OFFSET 0x04
#define GTIM_SMCR_OFFSET 0x08
#define GTIM_DIER_OFFSET 0x0C
#define GTIM_SR_OFFSET 0x10
#define GTIM_EGR_OFFSET 0x14
#define GTIM_CCMR1_OFFSET 0x18
#define GTIM_CCMR2_OFFSET 0x1C
#define GTIM_CCER_OFFSET 0x20
#define GTIM_CNT_OFFSET 0x24
#define GTIM_PSC_OFFSET 0x28
#define GTIM_ARR_OFFSET 0x2C
#define GTIM_CCR1_OFFSET 0x34
#define GTIM_CCR2_OFFSET 0x38
#define GTIM_CCR3_OFFSET 0x3C
#define GTIM_CCR4_OFFSET 0x40
#define GTIM_INSTA_OFFSET 0x50

#define PWM_BASE_ADDR 0x1611B000
#define PWM_OFFSET 0x10
#define LOW_BUFFER_OFFSET 0x4
#define FULL_BUFFER_OFFSET 0x8
#define CONTROL_REG_OFFSET 0xC

#define EN_BIT (1 << 0)
#define OE_BIT (1 << 3)
#define SINGLE_BIT (1 << 4)
#define INTE_BIT (1 << 5)
#define INT_BIT (1 << 6)
#define RST_BIT (1 << 7)
#define CAPTE_BIT (1 << 8)
#define INVERT_BIT (1 << 9)
#define DZONE_BIT (1 << 10)
  



class PWM_ls
{
public:
    PWM_ls(int pwmNum2, int period_, int duty_cycle_);
    ~PWM_ls(void);

    void setPeriod(unsigned int period_10ns_);
    void setDutyCycle(unsigned int duty_cycle_10ns_);
    void enable(void);
    uint32_t period_10ns, duty_cycle_10ns;

private:
    uint32_t base_addr2;
    void *low_buffer2;
    void *full_buffer2;
    void *control_buffer2;
};


extern void export_gpio(uint32 gpio_num);
extern void set_gpio_dir(const char* gpio_num_i,const char* model);
extern void set_gpio_value(const char* gpio_num_i,const char* potential);

extern void Speed_decision();
extern void Motor_Control();

extern int bmq_record;//记录编码器
extern int16 Speed_Goal;
extern int16 Speed_PID_OUT_l;
extern int16 Speed_PID_OUT_r;
extern int MOTOR_R_PID;
extern int MOTOR_L_PID;
extern int16 Speed_Goal_r;//目标速度
extern int16 Speed_Goal_l;//目标速度
extern float Left_Speed_Co_one_minus , Right_Speed_Co_one_minus ;   //0.06

#define OX  (50/ 3000.0)  //标度变换
extern float Disf;

void set_duty_cycle(const char* pwmchip, uint32 duty_cycle_10ns);

extern void set_gpio_value(const char* gpio_num_i, const char* potential);

class PWM_GTIM
{
public:
    PWM_GTIM(int gpio, int mux, int chNum_, int period_, int duty_cycle_);
    ~PWM_GTIM(void);

    void enable(void);
    void disable(void);
    void setPeriod(unsigned int period_10ns_);
    void setDutyCycle(unsigned int duty_cycle_10ns_);
    uint32_t period_10ns, duty_cycle_10ns;

private:
    uint32_t chNum;
    void *ccmr_buffer[2];
    void *ccer_buffer;
    void *period_buffer;
    void *duty_cycle_buffer;
    void *cnt_buffer;
};

#endif