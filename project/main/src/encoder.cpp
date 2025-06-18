#include "encoder.hpp"

double speed_l ;
double speed_r ;
// pwmNum为STEP引脚连接的PWM通道编号，gpioNum为DIR引脚所接的GPIO编号
ENCODER::ENCODER(int pwmNum1, int gpioNum1) : base_addr1(PWM_BASE_ADDR + pwmNum1 * PWM_OFFSET), directionGPIO(gpioNum1)
{
    directionGPIO.setDirection("in");

    control_buffer1 = map_register(base_addr1 + CONTROL_REG_OFFSET, PAGE_SIZE);
    low_buffer1 = map_register(base_addr1 + LOW_BUFFER_OFFSET, PAGE_SIZE);
    full_buffer1 = map_register(base_addr1 + FULL_BUFFER_OFFSET, PAGE_SIZE);

    printf("Registers mapped successfully\n");

    PWM_Init();
}

// 析构函数
ENCODER::~ENCODER(void)
{
    // directionGPIO.~GPIO();
    munmap(control_buffer1, PAGE_SIZE);
    munmap(low_buffer1, PAGE_SIZE);
    munmap(full_buffer1, PAGE_SIZE);
}

// 初始化PWM控制器为计数模式
void ENCODER::PWM_Init(void)
{
    uint32_t control_reg = 0;

    control_reg |= EN_BIT;
    control_reg |= CAPTE_BIT;
    control_reg |= INTE_BIT;

    REG_WRITE(control_buffer1, control_reg);

    printf("PWM initialized with control register: 0x%08X\n", control_reg);
}

// 清空计数器
void ENCODER::reset_counter(void)
{
    uint32_t control_reg = REG_READ(control_buffer1);
    control_reg |= RST_BIT;
    REG_WRITE(control_buffer1, control_reg);
}

// 返回编码器的RPS
double ENCODER::pulse_counter_update(void)
{
    double value = 100000000.0 / REG_READ(full_buffer1) / ENCODER_PPR * (directionGPIO.readValue() * 2 - 1);
    return value;
}
