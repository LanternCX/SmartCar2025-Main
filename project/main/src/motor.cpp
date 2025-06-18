#include"motor.hpp"
#include"register.hpp"
#include"imgdeal.hpp"
#include"steer.hpp"
#include"encoder.hpp"
int bmq_record=0;//记录编码器

// uint Speed_PID_OUT_l;//左电机输出PWM
// uint Speed_PID_OUT_r;//右电机输出PWM
int MOTOR_L_PID;
int MOTOR_R_PID;
int16 Speed_Goal;
float Left_Speed = 0.8,Right_Speed = 0.8;//差速系数（0-1之间）//0.75//0.83
//左轮
int16 Speed_Goal_l;//目标速度
int16 Speed_Encoder_l;//当前速度
float Speed_P_l,Speed_I_l,Speed_D_l;
int16 Speed_Erro_l;//当前误差
int16 Speed_PID_OUT_l;//占空比输出
int16 Speed_Lasterro_l;//上次误差
int16 Speed_Preverro_l;//上上次误差

//右轮
int16 Speed_Goal_r;//目标速度
int16 Speed_Encoder_r;//当前速度
float Speed_P_r,Speed_I_r,Speed_D_r;
int16 Speed_Erro_r;//当前误差
int16 Speed_PID_OUT_r;//占空比输出
int16 Speed_Lasterro_r;//上次误差
int16 Speed_Preverro_r;//上上次误差

float ZYCS = 0.42;
float YYCS = 0.42;
float WDCS = 0.3875;
float CS;

float Left_Speed_Co_one_minus , Right_Speed_Co_one_minus ;   //在数据初始化里为0.06

int16 Speed_Z = 20;
int16 Speed_C = 30;
int16 Speed_M = 30;
int16 Speed_R = 30;
int VW = 16;
int16 speed1_Last,speed2_last;

float Disf;

void export_gpio(uint32 gpio_num)
{
    std::string command = "echo " + std::string(std::to_string(gpio_num)) + " > /sys/class/gpio/export";
    system(command.c_str());
}


void set_gpio_dir(const char* gpio_num_i, const char* model)
{
    std::string command = "echo " + std::string(model) + " > /sys/class/gpio/" + std::string(gpio_num_i) + "/direction";
    system(command.c_str());
}


void set_gpio_value(const char* gpio_num_i, const char* potential)
{
    std::string command = "echo " + std::string(potential) + " > /sys/class/gpio/" + std::string(gpio_num_i) + "/value";
    system(command.c_str());
}

PWM_GTIM::PWM_GTIM(int gpio, int mux, int chNum_, int period_10ns_, int duty_cycle_10ns_)
    : period_10ns(period_10ns_), duty_cycle_10ns(duty_cycle_10ns_), chNum(chNum_ - 1)
{
    { // 配置功能复用
        void *gpio_mux_buffer = map_register(GPIO_MUX_BASE_ADDR + (gpio / 16) * 0x04, PAGE_SIZE);
        REG_WRITE(gpio_mux_buffer, (REG_READ(gpio_mux_buffer) & ~(0b11 << (gpio % 16 * 2))) | (mux << (gpio % 16 * 2)));
    }

    // 初始化所有寄存器
    REG_WRITE(map_register(GTIM_BASE_ADDR + GTIM_EGR_OFFSET, PAGE_SIZE), 0x01);

    // 启动计数器
    REG_WRITE(map_register(GTIM_BASE_ADDR + GTIM_CR1_OFFSET, PAGE_SIZE), 0x01);

    period_buffer = map_register(GTIM_BASE_ADDR + GTIM_ARR_OFFSET, PAGE_SIZE);
    duty_cycle_buffer = map_register(GTIM_BASE_ADDR + GTIM_CCR1_OFFSET + chNum * 0x04, PAGE_SIZE);
    ccmr_buffer[0] = map_register(GTIM_BASE_ADDR + GTIM_CCMR1_OFFSET, PAGE_SIZE);
    ccmr_buffer[1] = map_register(GTIM_BASE_ADDR + GTIM_CCMR2_OFFSET, PAGE_SIZE);
    ccer_buffer = map_register(GTIM_BASE_ADDR + GTIM_CCER_OFFSET, PAGE_SIZE);
    cnt_buffer = map_register(GTIM_BASE_ADDR + GTIM_CNT_OFFSET, PAGE_SIZE);

    // 清除chNum的PWM模式
    REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) & ~(0x7 << (chNum % 2 * 8 + 4)));
    // 配置chNum的PWM模式 0x6为模式1 0x7为模式2
    REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) | (0x7 << (chNum % 2 * 8 + 4)));

    // 清除chNum的输出极性// 设置周期（以10纳秒为单位）
    // 配置chNum的输出极性
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 1))); // 1为反相

    REG_WRITE(period_buffer, period_10ns);
    REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);

    REG_WRITE(cnt_buffer, 0);
    printf("Registers mapped successfully\n");
}

PWM_GTIM::~PWM_GTIM(void)
{
    munmap(ccmr_buffer[0], PAGE_SIZE);
    munmap(ccmr_buffer[1], PAGE_SIZE);
    munmap(period_buffer, PAGE_SIZE);
    munmap(duty_cycle_buffer, PAGE_SIZE);
    munmap(ccer_buffer, PAGE_SIZE);
    munmap(cnt_buffer, PAGE_SIZE);
}

void PWM_GTIM::enable(void)
{
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 0)));
}

void PWM_GTIM::disable(void)
{
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 0)));
}


void PWM_GTIM::setPeriod(unsigned int period_10ns_)
{
    period_10ns = period_10ns_;
    REG_WRITE(period_buffer, period_10ns);

    REG_WRITE(cnt_buffer, 0);
}


void PWM_GTIM::setDutyCycle(unsigned int duty_cycle_10ns_)
{
    duty_cycle_10ns = duty_cycle_10ns_;
    REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);

    REG_WRITE(cnt_buffer, 0);
}

PWM_ls::PWM_ls(int pwmNum2, int period_10ns_, int duty_cycle_10ns_) : period_10ns(period_10ns_), duty_cycle_10ns(duty_cycle_10ns_), base_addr2(PWM_BASE_ADDR + pwmNum2 * PWM_OFFSET)
{
	control_buffer2 = map_register(base_addr2 + CONTROL_REG_OFFSET, PAGE_SIZE);
    low_buffer2 = map_register(base_addr2 + LOW_BUFFER_OFFSET, PAGE_SIZE);
    full_buffer2 = map_register(base_addr2 + FULL_BUFFER_OFFSET, PAGE_SIZE);

    REG_WRITE(full_buffer2, period_10ns);
    REG_WRITE(low_buffer2, duty_cycle_10ns);

   // REG_WRITE(cnt_buffer, 0);
    printf("Registers mapped successfully\n");

}

PWM_ls::~PWM_ls(void)
{
    munmap(control_buffer2, PAGE_SIZE);
    munmap(low_buffer2, PAGE_SIZE);
    munmap(full_buffer2, PAGE_SIZE);
}


void PWM_ls::enable()
{
    uint32_t ctrl_reg = REG_READ(control_buffer2);
    // 设置使能位
    ctrl_reg |= EN_BIT;
    // 清除 OE_BIT 所代表的位（将其设置为 0）
    ctrl_reg &= ~OE_BIT;
    ctrl_reg &= ~SINGLE_BIT;
    REG_WRITE(control_buffer2, ctrl_reg); 

    std::cout << "Pulse output enabled." << std::endl;
}   


// 设置周期（以10纳秒为单位）
void PWM_ls::setPeriod(unsigned int period_10ns_)
{

    period_10ns = period_10ns_;
    REG_WRITE(full_buffer2, period_10ns);

   // REG_WRITE(cnt_buffer, 0);
}

//设置高电平时间（以10纳秒为单位）
void PWM_ls::setDutyCycle(unsigned int duty_cycle_10ns_)
{
    duty_cycle_10ns = duty_cycle_10ns_;
    REG_WRITE(low_buffer2, 10000-duty_cycle_10ns);


   // REG_WRITE(cnt_buffer, 0);
}


void speed_PIDL(void)
{
    Speed_Encoder_l=speed_l;//编码器采集当前速度
    Speed_Erro_l=Speed_Goal_l-Speed_Encoder_l;//更新本次误差

    Speed_PID_OUT_l+=(Speed_P_l*(Speed_Erro_l-Speed_Lasterro_l)+Speed_I_l*Speed_Erro_l+
                      Speed_D_l*(Speed_Erro_l-2*Speed_Lasterro_l+Speed_Preverro_l));//PID计算

    if(Speed_PID_OUT_l> 6666)Speed_PID_OUT_l = 6666;if(Speed_PID_OUT_l<-6666)Speed_PID_OUT_l =-6666;//限幅
    
    Speed_Preverro_l =  Speed_Lasterro_l;//更新前次误差
    Speed_Lasterro_l =  Speed_Erro_l;//更新上次误差
}

void speed_PIDR(void)
{

    Speed_Encoder_r=speed_r;//编码器采集当前速度
    Speed_Erro_r=Speed_Goal_r-Speed_Encoder_r;//更新本次误差

    Speed_PID_OUT_r+=(Speed_P_r*(Speed_Erro_r-Speed_Lasterro_r)+Speed_I_r*Speed_Erro_r+
                      Speed_D_r*(Speed_Erro_r-2*Speed_Lasterro_r+Speed_Preverro_r));//PID计算

    if(Speed_PID_OUT_r> 6666)Speed_PID_OUT_r = 6666;if(Speed_PID_OUT_r<-6666)Speed_PID_OUT_r =-6666;//限幅
    
    Speed_Preverro_r =  Speed_Lasterro_r;//更新前次误差
    Speed_Lasterro_r =  Speed_Erro_r;//更新上次误差
}

void Ackerman_diff(void){   /*4.14优化后的阿克曼差速*/
    double y,x;
    float limit = 0.00;
        if(ImageStatus.pwm_out >= steer_middle)//左转
        {
            x = (ImageStatus.pwm_out - steer_middle) * Left_Speed * 0.51619;
            if(x > 271)
            {
                x = 271; //y = 1.052999     Expected_Speed * 1.40803711
            }
            y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
            Speed_Goal_r = Speed_Goal * (1 + 0.3875 * y * limit);
            Speed_Goal_l = Speed_Goal * (1 - 0.3875 * y);
        }
        else if(ImageStatus.pwm_out < steer_middle)//右转
        {
            x = (steer_middle - ImageStatus.pwm_out) * Right_Speed * 0.51619;
            if(x > 271)
            {
                x = 271;
            }
            y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
            Speed_Goal_l = Speed_Goal * (1 + 0.3875 * y * limit);
            Speed_Goal_r = Speed_Goal * (1 - 0.3875 * y);
        }
}

void seepd_dif_one_minus(void)                //单减差速控制
{
    float y,x;

    if(ImageFlag.image_element_rings_flag != 0 && ImageFlag.image_element_rings == 1){ CS = ZYCS; }
    else if(ImageFlag.image_element_rings_flag != 0 && ImageFlag.image_element_rings == 2){ CS = YYCS;}
    else {CS = WDCS;}

    if(PWM_STEER >= steer_middle)  //左转
    {

        x = (PWM_STEER - steer_middle) * Left_Speed_Co_one_minus;
        if(x > 300)
        {
            x = 300;
        }
        y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
        Speed_Goal_r = Speed_Goal;// * (1 + 0.3875 * y)*0.9;// * (1 + 0.3875 * y);
        Speed_Goal_l = Speed_Goal * (1 - CS * y);

    }
    else if(PWM_STEER < steer_middle)//右转
    {
        x = (steer_middle - PWM_STEER)* Right_Speed_Co_one_minus;
        if(x>300)
        {
            x = 300;
        }
        y = -0.014344 + 0.0078637*x - 0.000014484*x*x;
        Speed_Goal_l = Speed_Goal;//* (1 + 0.3875 * y)*0.9;//+ (Speed_Goal * (0.3875 * y)* Disf); //* (1 + 0.3875 * y);
        Speed_Goal_r = Speed_Goal * (1 - CS * y);
    }


}

void Control_Speed()
{


//    if(ABS(ImageStatus.Det_True - ImageStatus.MiddleLine)>6)
//    {
//        Speed_Goal = Speed_Z - ABS(ImageStatus.Det_True - ImageStatus.MiddleLine); //150
//    }
//    else
//    {
//        Speed_Goal = Speed_Z;
//    }

    //220-30 稳



    Speed_Goal = Speed_Z ;
  //  Speed_Goal = Speed_Z + (Speed_M - Speed_Z)*exp(-0.01*abs(ImageStatus.Det_True - ImageStatus.MiddleLine));
    if(ImageStatus.Det_True - ImageStatus.MiddleLine > 7) //右转
    {
        Speed_Goal = VW;
    }
    if(ImageStatus.Det_True - ImageStatus.MiddleLine < -7) //左转
    {
        Speed_Goal = VW ;
    }



   // Speed_Goal = Speed_Z - cof * ABS(ImageStatus.Det_True - ImageStatus.MiddleLine); //150
    if(ImageStatus.Road_type ==LeftCirque|| ImageStatus.Road_type ==RightCirque) 
    {
       Speed_Goal =Speed_C; //150
    }
    if(ImageStatus.Road_type ==LeftCirque)
    {
        Speed_Goal =Speed_C + 15;
    }
}

void Speed_decision()
{
   // 重中之重： 电池电压低于12V 去充电 否则参数都是不对的
   // P是抑制 如果看到舵机反应是可以了，但是车还是出去了，说明速度环的P小了，没有抑制住，或者说I大了
   // Speed_Goal = 75;
    Speed_P_l = 80; //200  100
    Speed_I_l = 20; //40    25
    Speed_D_l = 5; //5      5
    Speed_P_r = 80;//200
    Speed_I_r = 20; //40
    Speed_D_r = 5;  //5
}

void Motor_Control()
{
            bmq_record=(speed_l+speed_r)/2;               //记录里程数，调试用，比赛删掉
            // SystemData.SpeedData.Length +=bmq_record;

            if (ImageStatus.CirquePass == 'T')  //成功入环后用距离消抖 防止直接判断出环
              ImageStatus.Pass_Lenth += bmq_record;
            else
              ImageStatus.Pass_Lenth = 0;

            if (ImageStatus.IsCinqueOutIn == 'T')
             ImageStatus.Cirque1lenth += bmq_record;
            else
             ImageStatus.Cirque1lenth = 0;

            if (ImageStatus.IsCinqueOutIn == 'T'
                  || ImageStatus.CirquePass == 'T'
                     || ImageStatus.CirqueOut == 'T')
                 ImageStatus.Cirque2lenth += bmq_record;
            else  ImageStatus.Cirque2lenth = 0;

            if (ImageStatus.CirqueOff == 'T')  //防止二次入环
              ImageStatus.Out_Lenth += bmq_record;
            else
              ImageStatus.Out_Lenth = 0;

            if (ImageStatus.sanchaju == 1)  //防止二次入环
              ImageStatus.pansancha_Lenth += bmq_record;
            else ImageStatus.pansancha_Lenth=0;
          //   if(ImageStatus.pansancha_Lenth>650)ImageStatus.pansancha_Lenth=0;


            if (SystemData.Stop == 2)
              ImageStatus.Stop_lenth += bmq_record;
            else
              ImageStatus.Stop_lenth = 0;

            if (ImageStatus.Road_type == Ramp)
              ImageStatus.Ramp_lenth += bmq_record;
            else
              ImageStatus.Ramp_lenth = 0;

            if (ImageStatus.Road_type == Cross_ture)
              ImageStatus.Cross_ture_lenth += bmq_record;
            else
              ImageStatus.Cross_ture_lenth = 0;

            if(ImageStatus.rukuwait_flag==1)
                ImageStatus.rukuwait_lenth+= bmq_record;

            Control_Speed();              //变速控制
            //seepd_dif_one_minus();        //单减差速
            //Ackerman_diff();
            Speed_Goal_l=20;
            Speed_Goal_r=20;
            speed_PIDL();
            speed_PIDR();
}

