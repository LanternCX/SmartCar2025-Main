#include"steer.hpp"
#define H_Min   2

PID_Datatypedef SteerPIDdata;    //舵机的PID参数
uint PWM_STEER;

float iError,//当前误差   原来是int类型 5.23更改为float
      SteerErr,
      SteerErr_pre;
int PWM;

PWM_ATIM::PWM_ATIM(int gpio, int mux, int chNum_, int period_10ns_, int duty_cycle_10ns_, int NEG_ = 0)
    : period_10ns(period_10ns_), duty_cycle_10ns(duty_cycle_10ns_), chNum(chNum_ - 1), NEG(NEG_)
{
    { // 配置功能复用
        void *gpio_mux_buffer = map_register(GPIO_MUX_BASE_ADDR + (gpio / 16) * 0x04, PAGE_SIZE);
        REG_WRITE(gpio_mux_buffer, REG_READ(gpio_mux_buffer) | (mux << (gpio % 16 * 2)));
    }
    // 初始化所有寄存器
    REG_WRITE(map_register(ATIM_BASE_ADDR + ATIM_EGR_OFFSET, PAGE_SIZE), 0x01);
    // 启动计数器
    REG_WRITE(map_register(ATIM_BASE_ADDR + ATIM_CR1_OFFSET, PAGE_SIZE), 0x01);
    period_buffer = map_register(ATIM_BASE_ADDR + ATIM_ARR_OFFSET, PAGE_SIZE);
    duty_cycle_buffer = map_register(ATIM_BASE_ADDR + ATIM_CCR1_OFFSET + chNum * 0x04, PAGE_SIZE);
    ccmr_buffer[0] = map_register(ATIM_BASE_ADDR + ATIM_CCMR1_OFFSET, PAGE_SIZE);
    ccmr_buffer[1] = map_register(ATIM_BASE_ADDR + ATIM_CCMR2_OFFSET, PAGE_SIZE);
    ccer_buffer = map_register(ATIM_BASE_ADDR + ATIM_CCER_OFFSET, PAGE_SIZE);
    cnt_buffer = map_register(ATIM_BASE_ADDR + ATIM_CNT_OFFSET, PAGE_SIZE);
    bdtr_buffer = map_register(ATIM_BASE_ADDR + ATIM_BDTR_OFFSET, PAGE_SIZE);
    // 清除chNum的PWM模式
    REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) & ~(0x7 << (chNum % 2 * 8 + 4)));
    // 配置chNum的PWM模式 0x6为模式1 0x7为模式2
    REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) | (0x7 << (chNum % 2 * 8 + 4)));
    // 清除chNum的输出极性
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 1 + NEG * 2))); // 1为反相
    // 配置chNum的输出极性
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 1 + NEG * 2))); // 1为反相
    REG_WRITE(period_buffer, period_10ns);
    REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);
    REG_WRITE(bdtr_buffer, 0x1 << 15); // 主输出使能
    REG_WRITE(cnt_buffer, 0);
    printf("Registers mapped successfully\n");
}
PWM_ATIM::~PWM_ATIM(void)
{
    munmap(ccmr_buffer[0], PAGE_SIZE);
    munmap(ccmr_buffer[1], PAGE_SIZE);
    munmap(period_buffer, PAGE_SIZE);
    munmap(duty_cycle_buffer, PAGE_SIZE);
    munmap(ccer_buffer, PAGE_SIZE);
    munmap(cnt_buffer, PAGE_SIZE);
    munmap(bdtr_buffer, PAGE_SIZE);
}
void PWM_ATIM::enable(void)
{
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 0 + NEG * 2)));
}
void PWM_ATIM::disable(void)
{
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 0 + NEG * 2)));
}
// 设置周期（以10纳秒为单位）
void PWM_ATIM::setPeriod(unsigned int period_10ns_)
{
    period_10ns = period_10ns_;
    REG_WRITE(period_buffer, period_10ns);

    REG_WRITE(cnt_buffer, 0);
}
// 设置低电平时间（以10纳秒为单位）
void PWM_ATIM::setDutyCycle(unsigned int duty_cycle_10ns_)
{
    duty_cycle_10ns = duty_cycle_10ns_;
    REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);
    REG_WRITE(cnt_buffer, 0);
}
void steer_init(void)
{
    PWM_ATIM test(86, 0b11, 3, 20000000, 1700);
    test.enable();
    test.setPeriod(20000000);
    test.setDutyCycle(1700000);
    //test.disable();
}

MH      MHstruct;
MH      MHstructFastCar = {

        {2.00,2.30,2.75,3.00,3.10,3.35,3.60 },//L
        {2.00,2.30,2.75,3.00,3.10,3.35,3.60 },//R

   {
    {
      {//L-IN
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    },

    {
      {//R-IN
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    },

    {
      {//L-OUT
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    },

    {
      {//R-OUT
              { 0, 1, 2, 3, },
              { 1, 2, 3, 4, },
              { 3, 4, 5, 6, },
              { 5, 6, 6, 6, }
      }
    }
   }
};

void DuoJi_GetP(float *i32p_P, int16_t i16_ViewH, int16_t i16_ViewE)
{

  MHstruct.i16_ViewH = i16_ViewH;

  float VH = f_Get_H_approximation(i16_ViewH - H_Min);
  float VE = f_Get_E_approximation(i16_ViewE, MHstruct.f_SizeOfViewE);
  float X2Y = 0;
  float X1Y = 0;
  float Y2X = 0;
  float Y1X = 0;

  int8_t VH1 = (int)VH;
  if (VH1 > VH) {
    VH--;
  }
  int8_t VH2 = VH1 + 1;

  int8_t VE1 = (int)VE;
  if (VE1 > VE) {
    VE1--;
  }
  int8_t VE2 = VE1 + 1;

  if (VH1 > 3) {
    VH1 = 3;
  }

  if (VH2 > 3) {
    VH2 = 3;
  }

  if (VE1 > 3) {
    VE1 = 3;
  }

  if (VE2 > 3) {
    VE2 = 3;
  }

  X2Y = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1]) *
            (VE - VE1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1];

  X1Y = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1]) *
            (VE - VE1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1];

  Y2X = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE1] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1]) *
            (VH - VH1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE1];

  Y1X = (MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH2][VE2] -
         MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2]) *
            (VH - VH1) +
        MHstruct.mt_Duoji[MHstruct.ui8_IOLR].ui8_Table[VH1][VE2];

  float P_approximation = (X2Y + X1Y + Y2X + Y1X) / 4.0;

  int8_t P1 = (int)P_approximation;
  if (P1 > P_approximation) {
    P1--;
  }
  int8_t P2 = P1 + 1;

    if (i16_ViewE < 0) {

  *i32p_P = (MHstruct.f_DuoJiP_TableL[P2] - MHstruct.f_DuoJiP_TableL[P1])*(P_approximation - P1) +MHstruct.f_DuoJiP_TableL[P1];
    }
    else
    {
  *i32p_P = (MHstruct.f_DuoJiP_TableR[P2] - MHstruct.f_DuoJiP_TableR[P1])*(P_approximation - P1) + MHstruct.f_DuoJiP_TableR[P1];
    }

}

float f_Get_H_approximation(short i16_ViewH) {
  float H_approximation;

  if (i16_ViewH < 0) {
    i16_ViewH = 0;
  }

  H_approximation = i16_ViewH * 3 / MHstruct.f_SizeOfViewH;

  return H_approximation;
}

float f_Get_E_approximation(short i16_E, float f_E_Size) {
  float E_approximation;

  if (i16_E < 0) {
    i16_E = -i16_E;
  }

  E_approximation = i16_E * 3 / f_E_Size;

  return E_approximation;
}

void InitMH(void) 
{
    MHstruct = MHstructFastCar;
    MHstruct.f_SizeOfViewE = 30; //有效偏差
    MHstruct.f_SizeOfViewH = 45; //有效可视距离
  }

void SteerPID_Realize(float offset)  //位置式    //模糊的偏差维度参数发生变化
{
  float KD;//二次d
  iError = offset;  //计算当前误差
  
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,offset);

      KD = 0.1 * (iError * iError) / (SteerPIDdata.Dl) + (SteerPIDdata.Dh);
      if(KD > 8.7) KD = 8.7;

  SteerErr =ImageStatus.MU_P * iError +SteerPIDdata.D * (iError - SteerPIDdata.LastError);  //位置式PID算式  模糊PD
  SteerPIDdata.LastError = iError;  //更新上次误差
  PWM = steer_middle - SteerErr;  //- +
  ImageStatus.pwm_now=PWM;
  LimitLeft(PWM);
  LimitRight(PWM);

  ImageStatus.pwm_out=PWM;
  PWM_STEER=200*PWM;
}