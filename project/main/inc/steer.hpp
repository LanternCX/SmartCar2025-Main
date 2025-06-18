#ifndef STEER_H
#define STEER_H

#include"base.hpp"
#include"imgdeal.hpp"
#include"register.hpp"
#include"gpio.hpp"

#define PAGE_SIZE 0x10000
#define REG_READ(addr) (*(volatile uint32_t *)(addr))
#define REG_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))
#define GPIO_MUX_BASE_ADDR 0x16000490
#define ATIM_BASE_ADDR 0x16118000
#define ATIM_CR1_OFFSET 0x00
#define ATIM_CR2_OFFSET 0x04
#define ATIM_SMCR_OFFSET 0x08
#define ATIM_DIER_OFFSET 0x0C
#define ATIM_SR_OFFSET 0x10
#define ATIM_EGR_OFFSET 0x14
#define ATIM_CCMR1_OFFSET 0x18
#define ATIM_CCMR2_OFFSET 0x1C
#define ATIM_CCER_OFFSET 0x20
#define ATIM_CNT_OFFSET 0x24
#define ATIM_PSC_OFFSET 0x28
#define ATIM_ARR_OFFSET 0x2C
#define ATIM_RCR_OFFSET 0x30
#define ATIM_CCR1_OFFSET 0x34
#define ATIM_CCR2_OFFSET 0x38
#define ATIM_CCR3_OFFSET 0x3C
#define ATIM_CCR4_OFFSET 0x40
#define ATIM_BDTR_OFFSET 0x44
#define ATIM_INSTA_OFFSET 0x50

class PWM_ATIM
{
public:
    PWM_ATIM(int gpio, int mux, int chNum_, int period_, int duty_cycle_, int NEG_);
    ~PWM_ATIM(void);

    void enable(void);
    void disable(void);
    void setPeriod(unsigned int period_10ns_);
    void setDutyCycle(unsigned int duty_cycle_10ns_);
    uint32_t period_10ns, duty_cycle_10ns;

private:
    uint32_t chNum, NEG;
    void *ccmr_buffer[2];
    void *ccer_buffer;
    void *period_buffer;
    void *duty_cycle_buffer;
    void *cnt_buffer;
    void *bdtr_buffer;
};

#define steer_middle  767
#define steer_right   655            //舵机右打 512  85   342  1865   1600 623
#define steer_left    855            //舵机左打 685 512  600 1000  2325  2300 793
#define LimitLeft(Left)    (Left = ((Left > steer_left) ? steer_left : Left))
#define LimitRight(Right)  (Right = ((Right < steer_right) ? steer_right : Right))

typedef struct 
{
  float P;
  float I;
  float D;
  float Dl;
  float Dh;
  float Diff_pl;/*二次PID*/
  float Diff_ph;
  float Diff_d;
  int LastError;  // Error[-1]
  int PrevError;  // Error[-2]
  int EC;
  float Kdin;   //入弯D
  float Kdout;  //出弯D
} PID_Datatypedef;

/*模糊控制部分*/
typedef struct {
  //基础模糊表
  uint8_t         ui8_Table[4][4];
} MH_Table;


typedef struct {
  //舵机P值表L
  float         f_DuoJiP_TableL[7];
  //舵机P值表R
  float         f_DuoJiP_TableR[7];
  //舵机模糊表
  MH_Table      mt_Duoji[4];
  //电机P值表
  float         f_DianJiP_Table[7];
  //电机I值表
  float         f_DianJiI_Table[7];
  //电机模糊表
  MH_Table      mt_DianJi[4];
  //出入弯标志
  uint8_t         ui8_IO;
  //左右出入弯标志
  uint8_t         ui8_IOLR;
  //出入弯加减速标志
  uint8_t         ui8_IOAS;
  //反向可视距离变化范围
  float         f_SizeOfViewH;
  //中值偏差变化范围
  float         f_SizeOfViewE;
  //脉冲偏差变化范围
  float         f_SizeOfPulseE;
  //上次反向可视距离
  short         i16_ViewH;
  //
} MH;
extern MH MHstructFastCar;
extern MH MHstruct;

float f_Get_E_approximation(short i16_E, float f_E_Size);
float f_Get_H_approximation(short i16_ViewH);
void DuoJi_GetP(float* i32p_P, short i16_ViewH, short i16_ViewE);
void InitMH(void);

extern void steer_init(void);
extern void SteerControl(int pwm);
extern void SteerPID_Realize(float offset);  //位置式    //模糊的偏差维度参数发生变化
extern uint PWM_STEER;
extern PID_Datatypedef SteerPIDdata;

#endif