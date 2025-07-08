#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "zf_common_headfile.h"

#define CAMERA_H 60
#define CAMERA_W 80

#define uint8 uint8_t
#define uint32 uint32_t
#define int16 int16_t
#define int32 int32_t

#define OX (50 / 3000.0) //标度变换

#define ImageSensorMid 39
#define LimitL(L) (L = ((L < 1) ? 1 : L))   //限制幅度
#define LimitH(H) (H = ((H > 78) ? 78 : H)) //限制幅度

extern int img1[60][80];           //图像二值化之后的数组
extern int imgdisplay[60][80];     //打印的图像数组
extern uint8 Garage_Location_Flag; //斑马线累积初始次数

void Element_Test(void);
extern int imageprocess(void);
extern void Data_Settings(void);
extern int ImageScanInterval; //扫边范围    上一行的边界+-ImageScanInterval
extern int ImageScanInterval_Cross2;

static int BottomBorderRight = 79, BottomBorderLeft = 0,
           BottomCenter = 0; //底行左右边界，中点

//元素类型
typedef enum {
  zero,
  Normol,      //无任何特征
  Straight,    ////直道
  Cross,       ////十字
  Ramp,        //坡道
  LeftCirque,  ////左圆环
  RightCirque, ////右圆环
  Cross_ture,
  Barn_in, //入库
} RoadType_e;

extern uint8 TP, TP_O1, TP_O2;
extern uint8 Circle[5];

typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;

typedef struct {
  /*以下关于全局图像正常参数*/

  //图像信息
  int TowPoint;            //初始比较点           给定前瞻
  int TowPointAdjust_v;    //初始比较点适应速度    没用
  int TowPoint_True;       //实际比较点           没用
  int TowPoint_Gain;       //增益系数             没用
  int TowPoint_Offset_Max; //最大偏置系数         没用
  int TowPoint_Offset_Min; //最小偏置系数         没用
  int Det_True;            //由GetDet()函数解算出来的平均图像偏差
  int Det_all;             //图像的近端到远端的总偏差
  float Det_all_k;         //斜率

  uint8 Threshold;         //二值化阈值
  uint32 Threshold_static; //二值化静态下限
  uint8 Threshold_detach;  //阳光算法分割阈值
  uint8 MiddleLine;        //屏幕中心
  int Foresight;           //偏差速度因子   远端的偏差  用于控速
  uint8 Left_Line;         //左边丢边数
  uint8 Right_Line;        //右边丢边数
  uint8 OFFLine;           /////图像顶边
  uint8 WhiteLine;         ////双边丢边数
  float ExpectCur;         /////图像期望曲率    没用
  float White_Ritio;       //有效行白点比例     没用
  int Black_Pro_ALL;       //整体黑点比例%      没用

  // PID
  float Piriod_P; //分段P   值     没用
  float MU_P;

  RoadType_e Road_type; //元素类型

  /****圆环***/
  int16_t image_element_rings; /*0 :��Բ��          1 :��Բ��       2 :��Բ��*/
  int16_t image_element_rings_flag; /*Բ������*/
  int16_t ring_big_small;           /*0:��
          
uint8 IsCinqueOutIn;  //进出圆环
uint8 CirquePass;     //圆环中
uint8 CirqueOut;      //出圆环
uint8 CirqueOff;      //圆环结束
          
//左右手法则扫线数据
          
int16 WhiteLine_L;        //左边丢线数
int16 WhiteLine_R;        //右边丢线数
int16 OFFLineBoundary;   //八领域截止行
          
int Pass_Lenth;       //入环距离  用于防抖
            /****圆环***/
  int Cirque1lenth;                 // tly1
  int Cirque2lenth;                 // tly2
  int Out_Lenth;                    //
  int Fork_Out_Len;                 //出岔口防抖
  int Dowm_lenth;                   //三叉减速距离  //g5.13
  int Cross_Lenth; // 270°转过之后一般是十字路口   作为三岔口的消抖
  int Cross_ture_lenth;
  int Sita; //据此判断在过去一段路程中转过多少角度
  int pansancha_Lenth;

  //车库
  int Barn_Flag;  //判断库的次数
  int Barn_Lenth; //入库停止距离
  int sanchaju;

  //保护
  int Stop_lenth; //出界保护放误判距离
  //坡道减速
  int Ramp_lenth;

  int variance; //直道检测阈值方差

  int straight_acc; //直道加速标志位
  int variance_acc; //用于加速的阈值方差

  int ramptestlenth; //坡道检测间隔

  int rukuwait_lenth;
  int rukuwait_flag;
  // int Blue_lenth;
  int newblue_flag;

} ImageParametertypedef;

typedef struct {
  int mid_temp; //短直线检测计算用
  /*左右边边界标志    T为正常跳变边    W为无边   P为障碍类多跳边的内边*/
  uint8 IsRightFind;     //右边有边标志
  uint8 IsLeftFind;      //左边有边标志
  uint8 isBlackFind;     //三叉边
  int Wide;              //边界宽度
  int LeftBorder;        //左边界
  int RightBorder;       //右边界
  int close_LeftBorder;  //靠边边界
  int close_RightBorder; //靠边边界
  int opp_LeftBorder;    //反向边界
  int opp_RightBorder;   //反向边界
  int Center;            //中线
  int RightTemp;         //右边临时值
  int LeftTemp;          //左边临时值
  int CenterTemp;        //中线临时值
  int Black_Point;       //单行黑点数量

  //左右手法则扫线数据
  int LeftBoundary_First;  //左边界 保存第一次数据
  int RightBoundary_First; //右边界 保存第一次数据
  int LeftBoundary;        //左边界 保存最后一次数据
  int RightBoundary;       //右边界 保存最后一次数据

  // fork   下面的变量用于岔路口的检测
} ImageDealDatatypedef;

typedef struct {

  /*以下关于全局图像正常参数*/

  //图像信息
  int TowPoint;            //初始比较点           给定前瞻
  int TowPointAdjust_v;    //初始比较点适应速度    没用
  int TowPoint_True;       //实际比较点           没用
  int TowPoint_Gain;       //增益系数             没用
  int TowPoint_Offset_Max; //最大偏置系数         没用
  int TowPoint_Offset_Min; //最小偏置系数         没用
  int Det_True;            //由GetDet()函数解算出来的平均图像偏差
  int Det_all;             //图像的近端到远端的总偏差
  float Det_all_k;         //斜率

  uint8 Threshold;         //二值化阈值
  uint32 Threshold_static; //二值化静态下限
  uint8 Threshold_detach;  //阳光算法分割阈值
  uint8 MiddleLine;        //屏幕中心
  int Foresight;           //偏差速度因子   远端的偏差  用于控速
  uint8 Left_Line;         //左边丢边数
  uint8 Right_Line;        //右边丢边数
  uint8 OFFLine;           /////图像顶边
  uint8 WhiteLine;         ////双边丢边数
  float ExpectCur;         /////图像期望曲率    没用
  float White_Ritio;       //有效行白点比例     没用
  int Black_Pro_ALL;       //整体黑点比例%      没用

  // PID
  float Piriod_P; //分段P   值     没用
  float MU_P;

  RoadType_e Road_type; //元素类型

  /****圆环***/
  uint8 IsCinqueOutIn; //进出圆环
  uint8 CirquePass;    //圆环中
  uint8 CirqueOut;     //出圆环
  uint8 CirqueOff;     //圆环结束

  //左右手法则扫线数据

  int16 WhiteLine_L;     //左边丢线数
  int16 WhiteLine_R;     //右边丢线数
  int16 OFFLineBoundary; //八领域截止行

  int Pass_Lenth;   //入环距离  用于防抖
                    /****圆环***/
  int Cirque1lenth; // tly1
  int Cirque2lenth; // tly2
  int Out_Lenth;    //
  int Fork_Out_Len; //出岔口防抖
  int Dowm_lenth;   //三叉减速距离  //g5.13
  int Cross_Lenth; // 270°转过之后一般是十字路口   作为三岔口的消抖
  int Cross_ture_lenth;
  int Sita; //据此判断在过去一段路程中转过多少角度
  int pansancha_Lenth;

  //车库
  int Barn_Flag;  //判断库的次数
  int Barn_Lenth; //入库停止距离
  int sanchaju;

  //保护
  int Stop_lenth; //出界保护放误判距离
  //坡道减速
  int Ramp_lenth;

  int variance;          //直道检测阈值方差
  int straight_acc_flag; //直线判断标志位

  int straight_acc; //直道加速标志位
  int variance_acc; //用于加速的阈值方差

  int ramptestlenth; //坡道检测间隔

  int rukuwait_lenth;
  int rukuwait_flag;
  // int Blue_lenth;
  int newblue_flag;

  int16 image_element_rings; /*0 :��Բ��          1 :��Բ��       2 :��Բ��*/
  int16 image_element_rings_flag; /*Բ������*/
  int16 ring_big_small;

} ImageStatustypedef;

typedef struct {
  float nowspeed;  // pulse表示nowspeed
  int expectspeed; // speed表示expectspeed
  int motor_duty;  //电机占空比
  float Length;    //走过路程
  int Circle_OUT_th;
  int MinSpeed;            //最低速度
  int MaxSpeed;            //最高速度
  float expect_True_speed; //实际期望速度
  int straight_speed;      //直道速度
} SpeedDatatypedef;

typedef struct {
  //    int16 Bend_Road;                            /*0 :无               1
  //    :右弯道     2 :左弯道*/
  int16 image_element_rings; /*0 :无圆环          1 :左圆环       2 :右圆环*/
  int16 ring_big_small; /*0:无                     1 :大圆环       2 :小圆环*/
  int16 image_element_rings_flag; /*圆环进程*/
  int16 straight_long;            /*长直道标志位*/
  //    int16 Garage_Location;                      /*0 :无车库          1
  //    :左车库       2 :右车库*/
  int16 Zebra_Flag; /*0 :无斑马线       1 左车库       2 :右车库*/
  //    int16 Ramp;                                  /*0 :无坡道 1：坡道*/ int16
  //    RoadBlock_Flag;                        /*0 :无路障            1 :路障*/
  //    int16 Out_Road;                               /*0 :无断路      1 :断路*/
  // 是否镜像
  bool is_flip;

  // 状态来源 0 无来源 1 传统方案 2 色块识别
  int stat_from;

  // 是否有红箱
  bool is_red;
} ImageFlagtypedef;

typedef struct {

  uint8 SteerOK;     //舵机启动标志
  uint8 CameraOK;    //摄像头启动显示标志
  uint8 OldCameraOK; //灰度传输标志
  uint8 MotorOK;     //电机开关
  uint8 Point;       //赛道实际中心
  uint8 UpdataOK;    //数据更新
  uint8 Stop;        //停止标志
  uint8 GO_OK;       //冲冲冲
  int Model;         //车辆模式

  //图像参数
  int OutCicle_line; //出环判断图像截止行  越大出环判断越严格越晚
  int L_T_R_W; // 出环左有边右无边数量  越大出环结束越严格
  int Circleoff_offline; //看到远处的距离判断出环结束  越小越严格
  int CircleP;           //环内P

  //防误判距离
  int fork_start_lenth; //岔路口检测起始
  int fork_off_lenth;   //岔路口检测截止

  int circle_off_lenth; //出环防误判距离

  //小圆环
  int circles_pl;
  int circles_pr;
  int circles_off_lenth; //小圆环出环防误判距离

  //中圆环
  int circlem_pl;
  int circlem_pr;
  int circlem_off_lenth; //小圆环出环防误判距离

  //大圆环
  int circlel_pl;
  int circlel_pr;
  int circlel_off_lenth;  //小圆环出环防误判距离
  int clrcle_priority[3]; //圆环类型
  int clrcle_num;         // 在第几个圆环

  int circle_kin;     //入环补线半径
  float circle_kout;  //出环补线斜率
  int circle_max_ang; //环内大叫限幅

  //直道
  float straight_p;       //直道P
  float straight_d;       //直道D
  int straighet_towpoint; //直道前瞻

  int debug_lenth; //调试距离

  //摄像头配置
  int exp_time;     //曝光时间
  int mtv_lroffset; //摄像头左右偏置
  int mtv_gain;     //摄像头增益

  int ramp_lenth_start; //坡道距离
  int fork_lenth_start; //三叉距离
  int barn_lenth;       //圆环距离

  int outbent_acc; //出弯加速

  int rounds;          // 圈数
  int speed_per_round; // 每圈减速多少

  SpeedDatatypedef SpeedData;
} SystemDatatypdef;

extern int Right_RingsFlag_Point1_Ysite;
extern int Right_RingsFlag_Point2_Ysite;
extern ImageStatustypedef ImageStatus; //图像的全局变量
extern ImageFlagtypedef ImageFlag;
extern SystemDatatypdef SystemData;
extern ImageParametertypedef ImageParameter;
extern float variance, variance_acc; //方差--直线检测用
#endif