#ifndef ENCODER_H
#define ENCODER_H

#include"base.hpp"
#include"register.hpp"
#include"gpio.hpp"

#define ENCODER_PPR 1024 // 编码器线数

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

class ENCODER
{
public:
    ENCODER(int pwmNum1, int gpioNum1);
    ~ENCODER(void);

    double pulse_counter_update(void);

private:
    uint32_t base_addr1;
    GPIO directionGPIO;
    void *low_buffer1;
    void *full_buffer1;
    void *control_buffer1;
    void PWM_Init(void);
    void reset_counter(void);
};

extern double speed_l;
extern double speed_r;

#endif