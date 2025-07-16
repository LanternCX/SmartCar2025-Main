#include "image.hpp"
#include "Debug.h"
#include "image_cv.hpp"
#include "zf_common_headfile.h"
#include "zf_device_ips200_fb.h"
#include <math.h>
#include <string>

int img1[60][80];       //图像二值化之后的数组
int imgdisplay[60][80]; //打印到中端图像数组

uint8 Garage_Location_Flag = 0; //斑马线累积初始次数
uint8 Garage_num = 2;
int Zebra_num1 = 10000;
int Zebra_num2 = 1000;

static int IntervalLow = 0, IntervalHigh = 0;
static int *PicTemp;
static int Ysite = 0, Xsite = 0;
ImageDealDatatypedef ImageDeal[60]; //最终图像
uint8 ExtenLFlag = 0;               //是否左延长标志
uint8 ExtenRFlag = 0;               //是否右延长标志
int ImageScanInterval_Cross;        // 270°的弯道后十字的扫线范围
int ImageScanInterval; //扫边范围    上一行的边界+-ImageScanInterval
ImageStatustypedef ImageStatus; //图像的全局变量
SystemDatatypdef SystemData;
ImageParametertypedef ImageParameter;

static int TFSite = 0, FTSite = 0; //存放行
static float DetR = 0, DetL = 0;   //存放斜率
static int ytemp = 0;              //存放行

uint8 Circle[5];
uint8 TP = 25, TP_O1 = 25, TP_O2 = 25;
uint32 circle_count_flag = 0;

float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77,
                       0.71, 0.65, 0.59, 0.53, 0.47}; // 10行

float variance, variance_acc;

uint8 circle_num = 0;
int Left_RingsFlag_Point1_Ysite,
    Left_RingsFlag_Point2_Ysite; //左圆环判断的两点纵坐标
int Right_RingsFlag_Point1_Ysite,
    Right_RingsFlag_Point2_Ysite; //右圆环判断的两点纵坐标
uint8 Ring_Help_Flag = 0;         //进环辅助标志
ImageFlagtypedef ImageFlag;
int Point_Xsite, Point_Ysite;               //拐点横纵坐标
int Repair_Point_Xsite, Repair_Point_Ysite; //补线点横纵坐标
uint8 Half_Road_Wide[60] =                  //直到赛道半宽
    {
        7,  8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  9,  10, 10, 10,
        11, 12, 12, 13, 13, 17, 17, 17, 16, 18, 18, 19, 21, 21, 21,
        21, 21, 19, 19, 20, 20, 20, 21, 21, 22, 23, 23, 23, 23, 24,
        26, 26, 26, 28, 28, 28, 30, 30, 30, 32, 32, 32, 34, 34, 35,
};

void Data_Settings(void) //参数赋值
{

  //图像参数
  // adcsum = 0;
  ImageStatus.MiddleLine = 39; //中线  39
  ImageStatus.TowPoint_Gain = 0.2;
  ImageStatus.TowPoint_Offset_Max = 5;
  ImageStatus.TowPoint_Offset_Min = -2;
  ImageStatus.TowPointAdjust_v = 160;
  ImageStatus.Det_all_k = 0.7; //待定自动补线斜率
  ImageStatus.CirquePass = 'F';
  ImageStatus.IsCinqueOutIn = 'F';
  ImageStatus.CirqueOut = 'F';
  ImageStatus.CirqueOff = 'F';
  ImageStatus.Barn_Flag = 0;
  ImageStatus.straight_acc = 0;
  ImageStatus.Road_type = zero;

  ImageStatus.TowPoint = 18;
  ImageStatus.Threshold_static = 70;  //静态阈值  40-80
  ImageStatus.Threshold_detach = 180; //阳光算法  亮斑分离140-220
  ImageScanInterval = 2;
  ImageScanInterval_Cross = 5;   //十字扫线范围
  ImageStatus.variance = 100;    //直道方差阈值
  ImageStatus.variance_acc = 50; //直线加速检测
                                 //  SystemData.outbent_acc  =  5;
  // SystemData.clrcle_num=0;
  ImageStatus.newblue_flag = 0;
  // SystemData.Stop = 2;                 //启动标志位
  // BlueTooth_Flag=1;

  Circle[0] = TP_O1;
  Circle[1] = TP_O2;

  ImageFlag.is_flip = false;
  ImageFlag.stat_from = 0;
  ImageFlag.ramp_flag = 0;

  //   SteerPIDdata.Dl = 21.07;
  //   SteerPIDdata.Dh = 5.0;
  //   Left_Speed_Co_one_minus =0.06; Right_Speed_Co_one_minus=0.06 ;   //0.06

  /**位置式pid参数**/
  // 重中之重： 电池电压低于12V 去充电 否则参数都是不对的
  // 方向环可以先调P 发现P已经转弯接近内切的时候，再去加D
  //  SteerPIDdata.P = 1.5;  //16 原来2.2 5.4大了 3.2小了 4.2大了一点点 3.9
  //  原来2.9   100hz参数      200hz参数:9.0 9.5 10.5 11.5 12.0 13.0
  //  //  11.0  比较适合的值：10.5
  //  SteerPIDdata.I = 0.0;
  //  SteerPIDdata.D = 2.7;  //原来7.2                       9.0 10.0
  //  目前最好9.5                               原来6.8 测试200hz参数:29.0
  // 200hz D：32小了 35也有点小大 39大 37大了
}

static uint8 drawfirstline(void) {
  PicTemp = img1[59]; //单行图像PicTemp                //59
  if (*(PicTemp + ImageSensorMid) == 0) //如果底边图像中点为黑，异常情况
  {
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++) //找左右边线
    {
      if (*(PicTemp + ImageSensorMid - Xsite) !=
          0)   //一旦找到左或右赛道到中心距离，就break
        break; //并且记录Xsite
      if (*(PicTemp + ImageSensorMid + Xsite) != 0)
        break;
    }
    if (*(PicTemp + ImageSensorMid - Xsite) != 0) //赛道如果在左边的话
    {
      BottomBorderRight = ImageSensorMid - Xsite + 1; // 59行右边线有啦
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--) //开始找59行左边线
      {
        if (*(PicTemp + Xsite) == 0 &&
            *(PicTemp + Xsite - 1) == 0) //连续两个黑点，滤波
        {
          BottomBorderLeft = Xsite; //左边线找到
          break;
        } else if (Xsite == 1) {
          BottomBorderLeft = 0; //搜索到最后了，看不到左边线，左边线认为是0
          break;
        }
      }
    } else if (*(PicTemp + ImageSensorMid + Xsite) != 0) //赛道如果在右边的话
    {
      BottomBorderLeft = ImageSensorMid + Xsite - 1; // 59行右边线有啦
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++) //开始找59行右边线
      {
        if (*(PicTemp + Xsite) == 0 &&
            *(PicTemp + Xsite + 1) == 0) //连续两个黑点，滤波
        {
          BottomBorderRight = Xsite; //右边线找到
          break;
        } else if (Xsite == 78) {
          BottomBorderRight = 79; //搜索到最后了，看不到右边线，左边线认为是79
          break;
        }
      }
    }
  } else //左边线中点是白的，比较正常的情况
  {
    for (Xsite = 79; Xsite > ImageSensorMid; Xsite--) //一个点一个点地搜索右边线
    {
      if (*(PicTemp + Xsite) == 1 &&
          *(PicTemp + Xsite - 1) == 1) //连续两个白点，滤波
      {
        BottomBorderRight = Xsite; //找到就记录
        break;
      } else if (Xsite == 40) {
        BottomBorderRight = 39; //找不到认为79
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++) //一个点一个点地搜索左边线
    {
      if (*(PicTemp + Xsite) == 1 &&
          *(PicTemp + Xsite + 1) == 1) //连续两个黑点，滤波
      {
        BottomBorderLeft = Xsite; //找到就记录
        break;
      } else if (Xsite == 38) {
        BottomBorderLeft = 39; //找不到认为0
        break;
      }
    }
  }
  BottomCenter =
      (BottomBorderLeft + BottomBorderRight) / 2; // 59行中点直接取平均
  ImageDeal[59].LeftBorder =
      BottomBorderLeft; //在数组里面记录一下信息，第一行特殊一点而已
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;                       //确定最底边
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft; //存储宽度信息
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  for (Ysite = 58; Ysite > 54; Ysite--) //由两边向中间确定底边五行
  {
    PicTemp = img1[Ysite];
    for (Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center;
         Xsite--) //和前面一样的搜索
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1) {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center + 1)) {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center;
         Xsite++) //和前面一样的搜索
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1) {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center - 1)) {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T'; //这些信息存储到数组里
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /
        2; //存储中点
    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder; //存储宽度
  }
  return 'T';
}

void GetJumpPointFromDet(
    int *p, uint8 type, int L, int H,
    JumpPointtypedef *Q) //第一个参数是要查找的数组（80个点）
                         //第二个扫左边线还是扫右边线
{                        //三四是开始和结束点
  int i = 0;
  if (type == 'L') //扫描左边线
  {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1) //由黑变白
      {
        Q->point = i;  //记录左边线
        Q->type = 'T'; //正确跳变
        break;
      } else if (i == (L + 1)) //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0) //如果中间是白的
        {
          Q->point = (L + H) / 2; //认为左边线是中点
          Q->type = 'W'; //非正确跳变且中间为白，认为没有边
          break;
        }
      }
    }
  } else if (type == 'R') //扫描右边线
  {
    for (i = L; i <= H; i++) //从右往左扫
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1) //找由白到黑的跳变
      {
        Q->point = i; //记录
        Q->type = 'T';
        break;
      } else if (i == (H - 1)) //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0) //如果中间是白的
        {
          Q->point = (L + H) / 2; //右边线是中点
          Q->type = 'W';
          break;
        } else //如果中点是黑的
        {
          Q->point = L; //左边线直接最大值
          Q->type = 'H';
          break;
        }
      }
    }
  }
}

static void DrawLinesProcess(void) //////不用更改
{
  uint8 L_Found_T = 'F'; //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_L_line = 'F'; //找到这一帧图像的基准左斜率
  uint8 R_Found_T = 'F'; //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_R_line = 'F'; //找到这一帧图像的基准右斜率
  float D_L = 0;          //延长线左边线斜率
  float D_R = 0;          //延长线右边线斜率
  int ytemp_W_L;          //记住首次左丢边行
  int ytemp_W_R;          //记住首次右丢边行
  ExtenRFlag = 0;         //延长线标志位清0
  ExtenLFlag = 0;
  ImageStatus.Left_Line = 0;
  ImageStatus.WhiteLine = 0;
  ImageStatus.Right_Line = 0;
  uint16_t end_1 = 0;
  for (Ysite = 54; Ysite > ImageStatus.OFFLine;
       Ysite--) //前5行处理过了，下面从55行到（设定的不处理的行OFFLine）
  { //太远的图像不稳定，OFFLine以后的不处理
    PicTemp = img1[Ysite];
    JumpPointtypedef JumpPoint[2]; // 0左1右
    if (ImageStatus.Road_type != Cross_ture
        /* &&SystemData.SpeedData.Length*OX>500*/) {
      IntervalLow =
          ImageDeal[Ysite + 1].RightBorder -
          ImageScanInterval; //暂时为2
                             ////从上一行右边线-Interval的点开始（确定扫描开始点）
      IntervalHigh =
          ImageDeal[Ysite + 1].RightBorder +
          ImageScanInterval; //到上一行右边线+Interval的点结束（确定扫描结束点）
    } else {
      IntervalLow =
          ImageDeal[Ysite + 1].RightBorder -
          ImageScanInterval_Cross; //暂时为2
                                   ////从上一行右边线-Interval_Cross的点开始（确定扫描开始点）
      IntervalHigh =
          ImageDeal[Ysite + 1].RightBorder +
          ImageScanInterval_Cross; //到上一行右边线+Interval_Cross的点开始（确定扫描开始点）
    }

    LimitL(IntervalLow);  //确定左扫描区间并进行限制
    LimitH(IntervalHigh); //确定右扫描区间并进行限制
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh,
                        &JumpPoint[1]); //扫右边线

    IntervalLow =
        ImageDeal[Ysite + 1].LeftBorder -
        ImageScanInterval; //从上一行左边线-5的点开始（确定扫描开始点）
    IntervalHigh =
        ImageDeal[Ysite + 1].LeftBorder +
        ImageScanInterval; //到上一行左边线+5的点结束（确定扫描结束点）

    LimitL(IntervalLow);  //确定左扫描区间并进行限制
    LimitH(IntervalHigh); //确定右扫描区间并进行限制
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

    if (JumpPoint[0].type == 'W') //如果本行左边线不正常跳变，即这10个点都是白的
    {
      ImageDeal[Ysite].LeftBorder =
          ImageDeal[Ysite + 1].LeftBorder; //本行左边线用上一行的数值
    } else                                 //左边线正常
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point; //记录下来啦
    }

    if (JumpPoint[1].type == 'W') //如果本行右边线不正常跳变
    {
      ImageDeal[Ysite].RightBorder =
          ImageDeal[Ysite + 1].RightBorder; //本行右边线用上一行的数值
    } else                                  //右边线正常
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point; //记录下来啦
    }

    ImageDeal[Ysite].IsLeftFind =
        JumpPoint[0].type; //记录本行是否找到边线，即边线类型
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    // *************************添加右********************************
    if (ImageDeal[Ysite].IsRightFind == 'W') {
      for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
           Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--) {
        if ((*(PicTemp + Xsite) != 0) && (*(PicTemp + Xsite + 1) == 0)) {
          ImageDeal[Ysite].RightBorder = Xsite;
          ImageDeal[Ysite].IsRightFind = 'T';
          break;
        }
      }
    }

    // *************************添加左********************************
  if (ImageDeal[Ysite].IsLeftFind == 'W') {
     // 用上一行的 RightBorder 作为上限，避免被本行右角点影响
     int scanMax = ImageDeal[Ysite+1].RightBorder - 1;
     for (Xsite = ImageDeal[Ysite].LeftBorder + 1;
          Xsite <= scanMax; Xsite++) {
        if ((*(PicTemp + Xsite) != 0) && (*(PicTemp + Xsite - 1) == 0)) {
          ImageDeal[Ysite].LeftBorder =
              Xsite;
          ImageDeal[Ysite].IsLeftFind = 'T';
          break;
        }
      }
    }

    //重新确定那些大跳变的边缘
    if ((ImageDeal[Ysite].IsLeftFind == 'H' ||
         ImageDeal[Ysite].IsRightFind == 'H')) {
      if (ImageDeal[Ysite].IsLeftFind == 'H') //如果左边线大跳变
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
             Xsite <= (ImageDeal[Ysite].RightBorder - 1);
             Xsite++) //左右边线之间重新扫描
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder =
                Xsite; //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0) //一旦出现白点则直接跳出
            break;
          else if (Xsite == (ImageDeal[Ysite].RightBorder - 1)) {
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <=
          7) //图像宽度限定
      {
        ImageStatus.OFFLine = Ysite + 1; //如果这行比7小了后面直接不要了
        break;
      }
      if (ImageDeal[Ysite].IsRightFind == 'H')
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
             Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--) {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0)) {
            ImageDeal[Ysite].RightBorder =
                Xsite; //如果右边线的左边还有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1)) {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
    }

    /***********重新确定无边行************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;

    if (ImageStatus.Road_type != Ramp) {
      if (ImageDeal[Ysite].IsRightFind == 'W' && Ysite > 10 &&
          Ysite < 50) //最早出现的无边行
      {
        if (Get_R_line == 'F') //这一帧图像没有跑过这个找基准线的代码段才运行
        {
          Get_R_line = 'T'; //找了  一帧图像只跑一次 置为T
          ytemp_W_R = Ysite + 2;
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
            if (ImageDeal[ysite].IsRightFind ==
                'T') //往无边行下面搜索  一般都是有边的
              R_found_point++;
          }
          if (R_found_point >
              14) //找到基准斜率边  做延长线重新确定无边   当有边的点数大于8
          {
            D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder -
                           ImageDeal[Ysite + 3].RightBorder)) /
                  ((float)(R_found_point - 3));
            //求下面这些点连起来的斜率
            //好给无边行做延长线左个基准
            if (D_R > 0) {
              R_Found_T =
                  'T'; //如果斜率大于0  那么找到了这个基准行  因为梯形畸变
                       //所以一般情况都是斜率大于0  小于0的情况也不用延长 没必要
            } else {
              R_Found_T = 'F'; //没有找到这个基准行
              if (D_R < 0)
                ExtenRFlag = 'F'; //这个标志位用于十字角点补线  防止图像误补用的
            }
          }
        }
        if (R_Found_T == 'T')
          ImageDeal[Ysite].RightBorder =
              ImageDeal[ytemp_W_R].RightBorder -
              D_R * (ytemp_W_R - Ysite); //如果找到了 那么以基准行做延长线

        LimitL(ImageDeal[Ysite].RightBorder); //限幅
        LimitH(ImageDeal[Ysite].RightBorder); //限幅
      }

      if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 &&
          Ysite < 50) //下面同理  左边界
      {
        if (Get_L_line == 'F') {
          Get_L_line = 'T';
          ytemp_W_L = Ysite + 2;
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
            if (ImageDeal[ysite].IsLeftFind == 'T')
              L_found_point++;
          }
          if (L_found_point > 14) //找到基准斜率边  做延长线重新确定无边
          {
            D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -
                           ImageDeal[Ysite + L_found_point].LeftBorder)) /
                  ((float)(L_found_point - 3));
            if (D_L > 0) {
              L_Found_T = 'T';

            } else {
              L_Found_T = 'F';
              if (D_L < 0)
                ExtenLFlag = 'F';
            }
          }
        }

        if (L_Found_T == 'T')
          ImageDeal[Ysite].LeftBorder =
              ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);

        LimitL(ImageDeal[Ysite].LeftBorder); //限幅
        LimitH(ImageDeal[Ysite].LeftBorder); //限幅
        end_1 = 1;
      }
    }
    if (ImageDeal[Ysite].IsLeftFind == 'W' &&
        ImageDeal[Ysite].IsRightFind == 'W') {
      ImageStatus.WhiteLine++; //要是左右都无边，丢边数+1
    }
    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite < 55) {
      ImageStatus.Left_Line++;
    }
    if (ImageDeal[Ysite].IsRightFind == 'W' && Ysite < 55) {
      ImageStatus.Right_Line++;
    }

    //********************************************************************/

    LimitL(ImageDeal[Ysite].LeftBorder);  //限幅
    LimitH(ImageDeal[Ysite].LeftBorder);  //限幅
    LimitL(ImageDeal[Ysite].RightBorder); //限幅
    LimitH(ImageDeal[Ysite].RightBorder); //限幅

    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 14) //重新确定可视距离
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (ImageDeal[Ysite].RightBorder <= 10 ||
             ImageDeal[Ysite].LeftBorder >= 70) {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    } //当图像宽度小于0或者左右边达到一定的限制时，则终止巡边
  }

  return;
}

void Search_Bottom_Line_OTSU(int imageInput[CAMERA_H][CAMERA_W], uint8 Row,
                             uint8 Col, uint8 Bottonline) {

  //寻找左边边界
  for (int Xsite = Col / 2 - 2; Xsite > 1; Xsite--) {
    if (imageInput[Bottonline][Xsite] == 1 &&
        imageInput[Bottonline][Xsite - 1] == 0) {
      ImageDeal[Bottonline].LeftBoundary = Xsite; //获取底边左边线
      break;
    }
  }
  for (int Xsite = Col / 2 + 2; Xsite < CAMERA_W - 1; Xsite++) {
    if (imageInput[Bottonline][Xsite] == 1 &&
        imageInput[Bottonline][Xsite + 1] == 0) {
      ImageDeal[Bottonline].RightBoundary = Xsite; //获取底边右边线
      break;
    }
  }
}

void Search_Left_and_Right_Lines(int imageInput[CAMERA_H][CAMERA_W], int Row,
                                 int Col, int Bottonline) {

  /*寻左线坐标规则*/
  int Left_Rule[2][8] = {
      {0, -1, 1, 0, 0, 1, -1, 0},  //{0,-1},{1,0},{0,1},{-1,0},  (x,y )
      {-1, -1, 1, -1, 1, 1, -1, 1} //{-1,-1},{1,-1},{1,1},{-1,1}
  };
  /*寻右线坐标规则*/
  int Right_Rule[2][8] = {
      {0, -1, 1, 0, 0, 1, -1, 0},  //{0,-1},{1,0},{0,1},{-1,0},
      {1, -1, 1, 1, -1, 1, -1, -1} //{1,-1},{1,1},{-1,1},{-1,-1}
  };
  int num = 0;
  uint8 Left_Ysite = Bottonline;
  uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
  uint8 Left_Rirection = 0; //左边方向
  uint8 Pixel_Left_Ysite = Bottonline;
  uint8 Pixel_Left_Xsite = 0;

  uint8 Right_Ysite = Bottonline;
  uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
  uint8 Right_Rirection = 0; //右边方向
  uint8 Pixel_Right_Ysite = Bottonline;
  uint8 Pixel_Right_Xsite = 0;
  uint8 Ysite = Bottonline;
  ImageStatus.OFFLineBoundary = 5;
  while (1) {
    num++;
    if (num > 400) {
      ImageStatus.OFFLineBoundary = Ysite;
      break;
    }
    if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite) {
      if (Ysite < ImageStatus.OFFLineBoundary) {
        ImageStatus.OFFLineBoundary = Ysite;
        break;
      } else {
        Ysite--;
      }
    }
    /*********左边巡线*******/
    if ((Pixel_Left_Ysite > Ysite) ||
        Ysite == ImageStatus.OFFLineBoundary) //右边扫线
    {
      /*计算前方坐标*/
      Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
      Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

      if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0) //前方是黑色
      {
        //顺时针旋转90
        if (Left_Rirection == 3)
          Left_Rirection = 0;
        else
          Left_Rirection++;
      } else //前方是白色
      {
        /*计算左前方坐标*/
        Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
        Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

        if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0) //左前方为黑色
        {
          //方向不变  Left_Rirection
          Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
          Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
          if (ImageDeal[Left_Ysite].LeftBoundary_First == 0) {
            ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
            ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
          }
        } else //左前方为白色
        {
          // 方向发生改变 Left_Rirection  逆时针90度
          Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
          Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
          if (ImageDeal[Left_Ysite].LeftBoundary_First == 0)
            ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
          ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
          if (Left_Rirection == 0)
            Left_Rirection = 3;
          else
            Left_Rirection--;
        }
      }
    }
    /*********右边巡线*******/
    if ((Pixel_Right_Ysite > Ysite) ||
        Ysite == ImageStatus.OFFLineBoundary) //右边扫线
    {
      /*计算前方坐标*/
      Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
      Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

      if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0) //前方是黑色
      {
        //逆时针旋转90
        if (Right_Rirection == 0)
          Right_Rirection = 3;
        else
          Right_Rirection--;
      } else //前方是白色
      {
        /*计算右前方坐标*/
        Pixel_Right_Ysite =
            Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
        Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

        if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] ==
            0) //左前方为黑色
        {
          //方向不变  Right_Rirection
          Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
          Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
          if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
            ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
          ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
        } else //左前方为白色
        {
          // 方向发生改变 Right_Rirection  逆时针90度
          Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
          Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
          if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
            ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
          ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
          if (Right_Rirection == 3)
            Right_Rirection = 0;
          else
            Right_Rirection++;
        }
      }
    }

    if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) <
        3) // Ysite<80是为了放在底部是斑马线扫描结束  3 && Ysite < 30
    {
      ImageStatus.OFFLineBoundary = Ysite;
      break;
    }
  }
}

void Search_Border_OTSU(int imageInput[CAMERA_H][CAMERA_W], uint8 Row,
                        uint8 Col, uint8 Bottonline) {
  ImageStatus.WhiteLine_L = 0;
  ImageStatus.WhiteLine_R = 0;
  // ImageStatus.OFFLine = 1;
  /*封上下边界处理*/
  for (int Xsite = 0; Xsite < CAMERA_W; Xsite++) {
    imageInput[0][Xsite] = 0;
    imageInput[Bottonline + 1][Xsite] = 0;
  }
  /*封左右边界处理*/
  for (int Ysite = 0; Ysite < CAMERA_H; Ysite++) {
    ImageDeal[Ysite].LeftBoundary_First = 0;
    ImageDeal[Ysite].RightBoundary_First = 79;

    imageInput[Ysite][0] = 0;
    imageInput[Ysite][CAMERA_W - 1] = 0;
  }
  /********获取底部边线*********/
  Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
  /********获取左右边线*********/
  Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);

  for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1;
       Ysite--) {
    if (ImageDeal[Ysite].LeftBoundary < 3) {
      ImageStatus.WhiteLine_L++;
    }
    if (ImageDeal[Ysite].RightBoundary > CAMERA_W - 3) {
      ImageStatus.WhiteLine_R++;
    }
  }
}

static void DrawExtensionLine(void) //绘制延长线并重新确定中线 ，把补线补成斜线
{
  if ((ImageStatus.Road_type != Ramp)
      //        &&ImageStatus.pansancha_Lenth* OX==0
      && ImageStatus.Road_type != LeftCirque &&
      ImageStatus.Road_type != RightCirque) // g5.22  6.22调试注释  记得改回来
  {
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    //    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type ==
    //    LeftCirque)
    //      TFSite = 55;
    if (ExtenLFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--) //从第五行开始网上扫扫到顶边下面两行   多段补线
                    //不仅仅只有一段
      {
        // printf("延长%d\n",Ysite);
        PicTemp = img1[Ysite]; //存当前行
        if (ImageDeal[Ysite].IsLeftFind ==
            'W') //如果本行左边界没扫到但扫到的是白色，说明本行没有左边界点
        {
          // printf("补线左\n");
          //**************************************************//**************************************************
          if (ImageDeal[Ysite + 1].LeftBorder >= 70) //如果左边界实在是太右边
          {
            // printf("左边届太靠右了\n");
            ImageStatus.OFFLine = Ysite + 1;
            break; //直接跳出（极端情况）
          }
          //************************************************//*************************************************

          while (Ysite >= (ImageStatus.OFFLine + 4)) //此时还没扫到顶边
          {
            Ysite--; //继续往上扫
            if (ImageDeal[Ysite].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 1].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 2].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 2].LeftBorder > 0 &&
                ImageDeal[Ysite - 2].LeftBorder <
                    70) //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
            {
              // printf("找到斜\n");
              FTSite = Ysite - 2; //把本行上面的第二行存入FTsite
              break;
            }
          }

          DetL =
              ((float)(ImageDeal[FTSite].LeftBorder -
                       ImageDeal[TFSite].LeftBorder)) /
              ((float)(FTSite - TFSite)); //左边界的斜率：列的坐标差/行的坐标差
          if (FTSite > ImageStatus.OFFLine)
            for (
                ytemp = TFSite; ytemp >= FTSite;
                ytemp--) //从第一次扫到的左边界的下面第二行的坐标开始往上扫直到空白上方的左边界的行坐标值
            {
              ImageDeal[ytemp].LeftBorder =
                  (int)(DetL * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite]
                      .LeftBorder; //将这期间的空白处补线（补斜线），目的是方便图像处理
            }
        } else
          TFSite =
              Ysite + 2; //如果扫到了本行的左边界，该行存在这里面，（算斜率）
      }

    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    // g5.22
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55;
    if (ExtenRFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--) //从第五行开始网上扫扫到顶边下面两行
      {
        PicTemp = img1[Ysite]; //存当前行

        if (ImageDeal[Ysite].IsRightFind ==
            'W') //如果本行右边界没扫到但扫到的是白色，说明本行没有右边界点，但是处于赛道内的
        {
          // printf("补线右\n");
          if (ImageDeal[Ysite + 1].RightBorder <= 10) //如果右边界实在是太左边
          {
            ImageStatus.OFFLine =
                Ysite + 1; //直接跳出，说明这种情况赛道就尼玛离谱
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4)) //此时还没扫到顶边下面两行
          {
            Ysite--;
            if (ImageDeal[Ysite].IsRightFind == 'T' &&
                ImageDeal[Ysite - 1].IsRightFind == 'T' &&
                ImageDeal[Ysite - 2].IsRightFind == 'T' &&
                ImageDeal[Ysite - 2].RightBorder < 70 &&
                ImageDeal[Ysite - 2].RightBorder >
                    10) //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
            {
              FTSite = Ysite - 2; // 把本行上面的第二行存入FTsite
              break;
            }
          }

          DetR =
              ((float)(ImageDeal[FTSite].RightBorder -
                       ImageDeal[TFSite].RightBorder)) /
              ((float)(FTSite - TFSite)); //右边界的斜率：列的坐标差/行的坐标差
          if (FTSite > ImageStatus.OFFLine)
            for (
                ytemp = TFSite; ytemp >= FTSite;
                ytemp--) //从第一次扫到的右边界的下面第二行的坐标开始往上扫直到空白上方的右边界的行坐标值
            {
              ImageDeal[ytemp].RightBorder =
                  (int)(DetR * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite]
                      .RightBorder; //将这期间的空白处补线（补斜线），目的是方便图像处理
            }
        } else
          TFSite = Ysite +
                   2; //如果本行的右边界找到了，则把该行下面第二行坐标送个TFsite
      }
  }
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /
        2; //扫描结束，把这一块经优化之后的中间值存入
    ImageDeal[Ysite].Wide = -ImageDeal[Ysite].LeftBorder +
                            ImageDeal[Ysite].RightBorder; //把优化之后的宽度存入
  }
}

static void RouteFilter(void) {
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5); //原本58
       Ysite--) //从开始位到停止位
  {
    if (ImageDeal[Ysite].IsLeftFind == 'W' &&
        ImageDeal[Ysite].IsRightFind == 'W' && Ysite <= 45 &&
        ImageDeal[Ysite - 1].IsLeftFind == 'W' &&
        ImageDeal[Ysite - 1].IsRightFind ==
            'W') //当前行左右都无边，而且在前45行   滤波
    {
      ytemp = Ysite;
      while (ytemp >=
             (ImageStatus.OFFLine + 5)) // 改改试试，-6效果好一些   原本+5
      {
        ytemp--;
        if (ImageDeal[ytemp].IsLeftFind == 'T' &&
            ImageDeal[ytemp].IsRightFind ==
                'T') //寻找两边都正常的，找到离本行最近的就不找了
        {
          DetR = (float)(ImageDeal[ytemp - 1].Center -
                         ImageDeal[Ysite + 2].Center) /
                 (float)(ytemp - 1 - Ysite - 2); //算斜率
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) {
            ImageDeal[Ysite].Center =
                (int)(CenterTemp + DetR * (float)(Ysite - LineTemp)); //用斜率补
            Ysite--;
          }
          break;
        }
      }
    }
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) /
        3; //求平均，应该会比较滑  本来是上下两点平均
  }
}

void GetDet() {
  float DetTemp = 0;
  int TowPoint = 0;
  float UnitAll = 0;

  // Speed_Control_Factor();

  /*固定圆环前瞻*/
  if (ImageStatus.image_element_rings_flag != 0)
    TowPoint = Circle[circle_count_flag];
  //   else if(ImageStatus.Road_type ==Cross_ture)
  //   {
  //     TowPoint=29;
  //   }
  else
    TowPoint = ImageStatus.TowPoint; //初始前瞻
  if (TowPoint < ImageStatus.OFFLine)
    TowPoint = ImageStatus.OFFLine + 1; //前瞻限幅
  if (TowPoint >= 49)
    TowPoint = 49;

  if ((TowPoint - 5) >=
      ImageStatus.OFFLine) //前瞻取设定前瞻还是可视距离  需要分情况讨论
                           //正常前瞻（与截止行相差5行）
  {
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
      DetTemp =
          DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

  } else if (TowPoint > ImageStatus.OFFLine) //正常前瞻与截止行不相差5行
  {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
      DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  } else if (ImageStatus.OFFLine < 49) //前瞻等于截至行
  {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

  } else {
    DetTemp = ImageStatus.Det_True;
  }

  ImageStatus.Det_True = DetTemp;

  ImageStatus.TowPoint_True = TowPoint;
}


// void GetDet() {
//   float DetTemp = 0;
//   int TowPoint = 0;

//   if (ImageFlag.image_element_rings_flag) {
//     TowPoint = Circle[ImageFlag.image_element_rings];
//   } else {
//     TowPoint = ImageStatus.TowPoint;
//   }

//   if (ImageStatus.TowPoint < ImageStatus.OFFLine) {
//     TowPoint = ImageStatus.OFFLine;  
//   }

//   DetTemp = ImageDeal[TowPoint].Center;
  
//   ImageStatus.TowPoint_True = TowPoint;
//   ImageStatus.Det_True = DetTemp;
// }

void Element_Judgment_Left_Rings() {

  if (
    ImageStatus.Right_Line > 5
    || ImageStatus.Left_Line < 9 // 13
    || ImageStatus.OFFLine > 10 
    || ImageDeal[58].RightBorder > 75 
    || ImageDeal[57].RightBorder > 75 
    || ImageDeal[56].RightBorder > 75 
    || ImageDeal[55].RightBorder > 75 
    || ImageDeal[54].RightBorder > 75 
    || ImageDeal[52].IsLeftFind == 'W' 
    || ImageDeal[53].IsLeftFind == 'W' 
    || ImageDeal[54].IsLeftFind == 'W' 
    || ImageDeal[55].IsLeftFind == 'W' 
    || ImageDeal[56].IsLeftFind == 'W' 
    || ImageDeal[57].IsLeftFind == 'W' 
    || ImageDeal[58].IsLeftFind == 'W') {
    return;
  }
  int ring_ysite = 25; // 25
  Left_RingsFlag_Point1_Ysite = 0;
  Left_RingsFlag_Point2_Ysite = 0;
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite].LeftBoundary_First -
            ImageDeal[Ysite - 1].LeftBoundary_First > 6) {
      Left_RingsFlag_Point1_Ysite = Ysite;
      break;
    }
  }
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 6) {
      Left_RingsFlag_Point2_Ysite = Ysite;
      break;
    }
  }

  for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine;
       Ysite--) {
    if ((ImageDeal[Ysite + 6].LeftBorder) < ImageDeal[Ysite + 3].LeftBorder &&
        (ImageDeal[Ysite + 5].LeftBorder) < ImageDeal[Ysite + 3].LeftBorder &&
        ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder &&
        ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder) {
      Ring_Help_Flag = 1;
      break;
    }
  }

  if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 2 &&
      Ring_Help_Flag == 0 && ImageStatus.Left_Line > 7) {
    if (ImageStatus.Left_Line > 13) // 13
      Ring_Help_Flag = 1;
  }
  if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 2 &&
      Ring_Help_Flag == 1 && 
      ImageFlag.image_element_rings_flag == 0 &&
      ImageStatus.Left_Line > 12 && 
      Left_RingsFlag_Point1_Ysite != 0 &&
      Left_RingsFlag_Point2_Ysite < 35 &&
      Left_RingsFlag_Point2_Ysite - Left_RingsFlag_Point1_Ysite < 10) {

    ImageFlag.image_element_rings = 1;
    ImageFlag.image_element_rings_flag = 1;
    ImageFlag.ring_big_small = 1;
    ImageStatus.Road_type = LeftCirque;

    ImageFlag.stat_from = 1;
    // szr=1;
  }
  Ring_Help_Flag = 0;
}

void Element_Judgment_Right_Rings() {
  if (ImageStatus.Left_Line > 9 || 
      ImageStatus.Right_Line < 5 || 
      ImageStatus.OFFLine > 10 || 
      //|| Straight_Judge(1, 15, 45) > 30
      //||  variance_acc>50
      //|| ImageStatus.WhiteLine>4
      ImageDeal[58].LeftBorder < 6 || 
      ImageDeal[57].LeftBorder < 6 ||
      ImageDeal[56].LeftBorder < 6 || 
      ImageDeal[55].LeftBorder < 6 ||
      ImageDeal[54].LeftBorder < 6 || 
      ImageDeal[52].IsRightFind == 'W' ||
      ImageDeal[53].IsRightFind == 'W' || 
      ImageDeal[54].IsRightFind == 'W' ||
      ImageDeal[55].IsRightFind == 'W' ||
      ImageDeal[56].IsRightFind == 'W' ||
      ImageDeal[57].IsRightFind == 'W' || 
      ImageDeal[58].IsRightFind == 'W'
    ) {
    return;
  }
  int ring_ysite = 5; // 5
  Right_RingsFlag_Point1_Ysite = 0;
  Right_RingsFlag_Point2_Ysite = 0;
  for (int Ysite = 55; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite - 1].RightBoundary_First -
            ImageDeal[Ysite].RightBoundary_First >
        4) {
      Right_RingsFlag_Point1_Ysite = Ysite;
      break;
    }
  }
  for (int Ysite = 55; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary >
        4) {
      Right_RingsFlag_Point2_Ysite = Ysite;
      break;
    }
  }
  for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine;
       Ysite--) {
    if (ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
        ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
        ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder &&
        ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder) {
      Ring_Help_Flag = 1;
      break;
    }
  }
  if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 0 && ImageStatus.Right_Line > 10) {
    if (ImageStatus.Right_Line > 13)
      Ring_Help_Flag = 1;
  }

  if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 1 && 
      ImageFlag.image_element_rings_flag == 0 &&
      ImageStatus.Right_Line > 13 && 
      Right_RingsFlag_Point1_Ysite != 0 &&
      Right_RingsFlag_Point2_Ysite > 25

  ) {

    // 镜像之后直接当成左圆环处理
    ImageFlag.image_element_rings = 1;
    ImageFlag.image_element_rings_flag = 1;
    ImageFlag.ring_big_small = 1; //小环
    ImageStatus.Road_type = LeftCirque;

    ImageFlag.is_flip = true;
    ImageFlag.stat_from = 1;
  }
  Ring_Help_Flag = 0;
}

//左圆环判断
void Element_Handle_Left_Rings() {
  /***************************************判断**************************************/
  int num = 0;
  for (int Ysite = 55; Ysite > 40; Ysite--) {
    if (ImageDeal[Ysite].IsLeftFind == 'W')
      num++;
    if (ImageDeal[Ysite + 3].IsLeftFind == 'W' &&
        ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
        ImageDeal[Ysite + 1].IsLeftFind == 'W' &&
        ImageDeal[Ysite].IsLeftFind == 'T')
      break;
  }
  //    tft180_show_int(60,125,num,3);
  //    int ring_ysite = 30;
  //    for (int Ysite = 5; Ysite < ring_ysite; Ysite++)
  //    {
  //        if (ImageDeal[Ysite - 1].RightBoundary_First -
  //        ImageDeal[Ysite].RightBoundary_First > 4)
  //        {
  //            Right_RingsFlag_Point1_Ysite = Ysite;
  //            break;
  //        }
  //    }
  //    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
  //    {
  //        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite +
  //        1].RightBoundary > 4)
  //        {
  //            Right_RingsFlag_Point2_Ysite = Ysite;
  //            break;
  //        }
  //    }
  //准备进环
  if (ImageFlag.image_element_rings_flag == 1 &&
      num > 10) //&& ImageStatus.Left_Line >25
  {
    ImageFlag.image_element_rings_flag = 2;
    // wireless_uart_send_byte(2);
  }

  //    if( ImageFlag.image_element_r，rings_flag == 2 )
  //    {
  //        if( SaiDaoKuanDu() > 62)
  //        ImageFlag.image_element_rings_flag = 3;
  //
  //    }

  if (ImageFlag.image_element_rings_flag == 2 &&
      num < 9) //&& ImageStatus.Left_Line <15
  {
    ImageFlag.image_element_rings_flag = 5;
    // wireless_uart_send_byte(5);
  }
  //    if(ImageFlag.image_element_rings_flag == 3 && ImageStatus.Left_Line >20)
  //    {
  //        ImageFlag.image_element_rings_flag = 5;
  //    }

  //进环
  if (ImageFlag.image_element_rings_flag == 5 && 
    ((ImageStatus.Right_Line > 10 && ImageStatus.Left_Line > 10) || (ImageFlag.is_red)) ) {
    ImageFlag.image_element_rings_flag = 6;
    //   ImageStatus.Road_type = LeftCirque;
    // wireless_uart_send_byte(6);
  }
  //进环小圆环
  if (ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line < 3) {
    // Stop = 1;
    ImageFlag.image_element_rings_flag = 7;
    // wireless_uart_send_byte(8);
  }
  //环内 大圆环判断
  if (ImageFlag.image_element_rings_flag == 7) {
    Point_Ysite = 0;
    Point_Xsite = 0;
    for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--) {
      if (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder
          // ImageStatus.WhiteLine  > 8
          //                    && ImageDeal[Ysite].RightBorder <=
          //                    ImageDeal[Ysite + 4].RightBorder
          //                    && ImageDeal[Ysite].RightBorder <=
          //                    ImageDeal[Ysite - 4].RightBorder
      ) {
        Point_Xsite = ImageDeal[Ysite].RightBorder;
        Point_Ysite = Ysite;
        break;
      }
    }
    if (Point_Ysite > 15 && ImageStatus.Right_Line > 10) {
      ImageFlag.image_element_rings_flag = 8;
      // wireless_uart_send_byte(8);
      // Stop = 1;
    }
  }
  //        //环内 小圆环判断
  //    if (ImageFlag.image_element_rings_flag == 7 && ImageFlag.ring_big_small
  //    == 2)
  //    {
  //        Point_Ysite = 0;
  //        Point_Xsite = 0;
  //        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary + 3;
  //        Ysite--)
  //        {
  //            if (    ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite +
  //            2].RightBoundary
  //                 && ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite -
  //                 2].RightBoundary
  //               )
  //            {
  //                Point_Xsite = ImageDeal[Ysite].RightBoundary;
  //                Point_Ysite = Ysite;
  //                break;
  //            }
  //        }
  //        if (Point_Ysite > 20)
  //          ImageFlag.image_element_rings_flag = 8;
  //    }
  //出环后
  if (ImageFlag.image_element_rings_flag == 8) {
    if (
        // Straight_Judge(2, ImageStatus.OFFLine+15, 50) < 1
        (ImageStatus.Right_Line < 10 && 
        ImageStatus.OFFLine < 10) || 
        ImageFlag.is_red
      ) {
      //右边为直线且截止行（前瞻值）很小
      ImageFlag.image_element_rings_flag = 9;
      // wireless_uart_send_byte(9);
    }
  }

  //结束圆环进程
  if (ImageFlag.image_element_rings_flag == 9) {
    int num = 0;
    for (int Ysite = 40; Ysite > 10; Ysite--) {
      if (ImageDeal[Ysite].IsLeftFind == 'W')
        num++;
    }
    if (num < 5) {
      //                ImageStatus.Road_type = 0;   //出环处理完道路类型清0
      // ImageFlag.image_element_rings_flag = 0;
      // ImageFlag.image_element_rings = 0;
      // ImageFlag.ring_big_small = 0;
      // ImageStatus.Road_type = Normol;
      // ImageFlag.is_flip = false;

      ImageFlag.image_element_rings_flag = 10;
      // wireless_uart_send_byte(0);
      circle_num++;
    }
  }

  /***************************************处理**************************************/
  //准备进环  半宽处理
  if (ImageFlag.image_element_rings_flag == 1 ||
      ImageFlag.image_element_rings_flag == 2 ||
      ImageFlag.image_element_rings_flag == 3 ||
      ImageFlag.image_element_rings_flag == 4) {
    for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
    }
  }
  //进环  补线
  if (ImageFlag.image_element_rings_flag == 5 ||
      ImageFlag.image_element_rings_flag == 6) {
    int flag_Xsite_1 = 0;
    int flag_Ysite_1 = 0;
    float Slope_Rings = 0;
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) //下面弧点
    {
      for (Xsite = ImageDeal[Ysite].LeftBorder + 1;
           Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++) {
        if (img1[Ysite][Xsite] == 1 && img1[Ysite][Xsite + 1] == 0) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = Xsite;
          Slope_Rings = (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
      if (flag_Ysite_1 != 0) {
        break;
      }
    }
    if (flag_Ysite_1 == 0) {
      for (Ysite = ImageStatus.OFFLine + 1; Ysite < 30; Ysite++) {
        if (ImageDeal[Ysite].IsLeftFind == 'T' &&
            ImageDeal[Ysite + 1].IsLeftFind == 'T' &&
            ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
            abs(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 2].LeftBorder) >
                10) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = ImageDeal[flag_Ysite_1].LeftBorder;
          ImageStatus.OFFLine = Ysite;
          Slope_Rings = (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
    }
    //补线
    if (flag_Ysite_1 != 0) {
      for (Ysite = flag_Ysite_1; Ysite < 60; Ysite++) {
        ImageDeal[Ysite].RightBorder =
            flag_Xsite_1 + Slope_Rings * (Ysite - flag_Ysite_1);
        // if(ImageFlag.ring_big_small==1)//大圆环不减半宽
        ImageDeal[Ysite].Center =
            ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
        // else//小圆环减半宽
        //     ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder -
        //     Half_Bend_Wide[Ysite];
        if (ImageDeal[Ysite].Center < 4)
          ImageDeal[Ysite].Center = 4;
      }
      ImageDeal[flag_Ysite_1].RightBorder = flag_Xsite_1;
      for (Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--) // A点上方进行扫线
      {
        for (Xsite = ImageDeal[Ysite + 1].RightBorder - 10;
             Xsite < ImageDeal[Ysite + 1].RightBorder + 2; Xsite++) {
          if (img1[Ysite][Xsite] == 1 && img1[Ysite][Xsite + 1] == 0) {
            ImageDeal[Ysite].RightBorder = Xsite;
            // if(ImageFlag.ring_big_small==1)//大圆环不减半宽
            ImageDeal[Ysite].Center =
                ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /
                 2);
            // else//小圆环减半宽
            //     ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder -
            //     Half_Bend_Wide[Ysite];
            if (ImageDeal[Ysite].Center < 4)
              ImageDeal[Ysite].Center = 4;
            ImageDeal[Ysite].Wide =
                ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
            break;
          }
        }

        if (ImageDeal[Ysite].Wide > 8 &&
            ImageDeal[Ysite].RightBorder < ImageDeal[Ysite + 2].RightBorder) {
          continue;
        } else {
          ImageStatus.OFFLine = Ysite + 2;
          break;
        }
      }
    }
  }
  //环内 小环弯道减半宽 大环不减
  if (ImageFlag.image_element_rings_flag == 7) {
  }
  //大圆环出环 补线
  if (ImageFlag.image_element_rings_flag == 8) //大圆环
  {
    Repair_Point_Xsite = 30;
    Repair_Point_Ysite = 7;
    for (int Ysite = 40; Ysite > 5; Ysite--) {
      if (img1[Ysite][15] == 1 && img1[Ysite - 1][15] == 0) // 28
      {
        Repair_Point_Xsite = 15;
        Repair_Point_Ysite = Ysite - 1;
        ImageStatus.OFFLine = Ysite + 1; //截止行重新规划
        break;
      }
    }

    for (int Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--) //补线
    {
      ImageDeal[Ysite].RightBorder =
          (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) /
              (58 - Repair_Point_Ysite) +
          ImageDeal[58].RightBorder;
      ImageDeal[Ysite].Center =
          (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
    }
    //        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--) //补线
    //        {
    ////            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder -
    /// Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  +
    /// ImageDeal[58].RightBorder; /            ImageDeal[Ysite].Center =
    ///((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
    //            ImageDeal[Ysite].RightBorder
    //            =ImageDeal[Ysite].LeftBorder+Half_Road_Wide[Ysite]+8;
    //            if(ImageDeal[Ysite].RightBorder>77)
    //            ImageDeal[Ysite].RightBorder=77; ImageDeal[Ysite].Center =
    //            ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)
    //            / 2);
    //        }
  }
  //        //小圆环出环 补线
  //    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small
  //    == 2)    //小圆环
  //    {
  //        Repair_Point_Xsite = 0;
  //        Repair_Point_Ysite = 0;
  //        for (int Ysite = 55; Ysite > 5; Ysite--)
  //        {
  //            if (Pixle[Ysite][15] == 1 && Pixle[Ysite-1][15] == 0)
  //            {
  //                Repair_Point_Xsite = 15;
  //                Repair_Point_Ysite = Ysite-1;
  //                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
  //                break;
  //            }
  //        }
  //        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--) //补线
  //        {
  //            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder -
  //            Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)
  //            + ImageDeal[58].RightBorder; ImageDeal[Ysite].Center =
  //            (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /
  //            2;
  //        }
  //    }
  //已出环 半宽处理
  if (ImageFlag.image_element_rings_flag == 9 ||
      ImageFlag.image_element_rings_flag == 10) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
    }
  }
}
//--------------------------------------------------------------
//  @name           Element_Handle_Right_Rings()
//  @brief          整个图像处理的子函数，用来处理右圆环类型.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Right_Rings();
//-------------------------------------------------------------
void Element_Handle_Right_Rings() {
  /****************判断*****************/
  int num = 0;
  for (int Ysite = 55; Ysite > 20; Ysite--) {
    if (ImageDeal[Ysite].IsRightFind == 'W') {
      num++;
      // szr=num;
    }
    if (ImageDeal[Ysite + 3].IsRightFind == 'W' &&
        ImageDeal[Ysite + 2].IsRightFind == 'W' &&
        ImageDeal[Ysite + 1].IsRightFind == 'W' &&
        ImageDeal[Ysite].IsRightFind == 'T')
      break;
  }
  //准备进环
  if (ImageFlag.image_element_rings_flag == 1 &&
      num > 10) //&& ImageStatus.Right_Line >25
  {
    ImageFlag.image_element_rings_flag = 2;
  }

  //    if( ImageFlag.image_element_rings_flag == 2 )
  //    {
  //        if( SaiDaoKuanDu() > 62)
  //        ImageFlag.image_element_rings_flag = 3;
  //
  //    }

  if (ImageFlag.image_element_rings_flag == 2 &&
      num < 9) //&& ImageStatus.Right_Line<15
  {
    ImageFlag.image_element_rings_flag = 5;
  }
  //    if(ImageFlag.image_element_rings_flag = 3 &&
  //    ImageStatus.Right_Line>25)//
  //    {
  //        ImageFlag.image_element_rings_flag = 5;
  //    }
  //进环
  if (ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line >= 20) {
    ImageFlag.image_element_rings_flag = 6;
    // ImageStatus.Road_type = RightCirque;
  }
  //进环小圆环
  if (ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line < 10) {
    ImageFlag.image_element_rings_flag = 7;
    // Stop=1;
  }
  if (ImageFlag.image_element_rings_flag == 7) {
    Point_Xsite = 0;
    Point_Ysite = 0;
    for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--) {
      if ( // ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
           //&& ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder
          // && ImageStatus.WhiteLine  > 8

          //   && ImageDeal[Ysite].LeftBorder > ImageDeal[Ysite - 4].LeftBorder
          //                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite +
          //                 4].LeftBorder
          //                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite -
          //                 4].LeftBorder
      ) {
        Point_Xsite = ImageDeal[Ysite].LeftBorder;
        Point_Ysite = Ysite;
        break;
      }
    }
    if (Point_Ysite > 35 && ImageStatus.Left_Line > 20) {
      ImageFlag.image_element_rings_flag = 8;
      //            gpio_set_level(P33_10, 1);
    }
  }
  if (ImageFlag.image_element_rings_flag == 8) {
    if ( // Straight_Judge(1, ImageStatus.OFFLine+10, 45) < 1
        ImageStatus.Left_Line < 20 && ImageStatus.OFFLine < 25

        ) //右边为直线且截止行（前瞻值）很小
    {
      ImageFlag.image_element_rings_flag = 9;
    }
  }
  if (ImageFlag.image_element_rings_flag == 9) {
    int num = 0;
    for (int Ysite = 40; Ysite > 10; Ysite--) {
      if (ImageDeal[Ysite].IsRightFind == 'W') {
        num++;
      }
    }
    // szr=num;
    if (num < 10) {
      // ImageStatus.Road_type = 0;   //出环处理完道路类型清0
      
      // ImageFlag.image_element_rings_flag = 0;
      // ImageFlag.image_element_rings = 0;
      // ImageFlag.ring_big_small = 0;
      // ImageStatus.Road_type = Normol;

      ImageFlag.image_element_rings_flag = 10;
      //            Front_Ring_Continue_Count++;
      circle_num++;
    }
  }
  /***************************************处理**************************************/
  //准备进环  半宽处理
  if (ImageFlag.image_element_rings_flag == 1 ||
      ImageFlag.image_element_rings_flag == 2 ||
      ImageFlag.image_element_rings_flag == 3 ||
      ImageFlag.image_element_rings_flag == 4) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
    }
  }

  //进环  补线
  if (ImageFlag.image_element_rings_flag == 5 ||
      ImageFlag.image_element_rings_flag == 6) {
    int flag_Xsite_1 = 0;
    int flag_Ysite_1 = 0;
    float Slope_Right_Rings = 0;
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      for (Xsite = ImageDeal[Ysite].LeftBorder + 1;
           Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++) {
        if (img1[Ysite][Xsite] == 1 && img1[Ysite][Xsite + 1] == 0) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = Xsite;
          Slope_Right_Rings =
              (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
      if (flag_Ysite_1 != 0) {
        break;
      }
    }
    if (flag_Ysite_1 == 0) {
      for (Ysite = ImageStatus.OFFLine + 5; Ysite < 30; Ysite++) {
        if (ImageDeal[Ysite].IsRightFind == 'T' &&
            ImageDeal[Ysite + 1].IsRightFind == 'T' &&
            ImageDeal[Ysite + 2].IsRightFind == 'W' &&
            abs(ImageDeal[Ysite].RightBorder -
                ImageDeal[Ysite + 2].RightBorder) > 10) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = ImageDeal[flag_Ysite_1].RightBorder;
          ImageStatus.OFFLine = Ysite;
          Slope_Right_Rings =
              (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
    }
    //补线
    if (flag_Ysite_1 != 0) {
      for (Ysite = flag_Ysite_1; Ysite < 58; Ysite++) {
        ImageDeal[Ysite].LeftBorder =
            flag_Xsite_1 + Slope_Right_Rings * (Ysite - flag_Ysite_1);
        //                if(ImageFlag.ring_big_small==2)//小圆环加半宽
        //                    ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
        //                else//大圆环不加半宽
        ImageDeal[Ysite].Center =
            (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /
            2; //板块
        if (ImageDeal[Ysite].Center > 79)
          ImageDeal[Ysite].Center = 79;
      }
      ImageDeal[flag_Ysite_1].LeftBorder = flag_Xsite_1;
      for (Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--) // A点上方进行扫线
      {
        for (Xsite = ImageDeal[Ysite + 1].LeftBorder + 8;
             Xsite > ImageDeal[Ysite + 1].LeftBorder - 4; Xsite--) {
          if (img1[Ysite][Xsite] == 1 && img1[Ysite][Xsite - 1] == 0) {
            ImageDeal[Ysite].LeftBorder = Xsite;
            ImageDeal[Ysite].Wide =
                ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
            //                     if(ImageFlag.ring_big_small==2)//小圆环加半宽
            //                         ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
            //                     else//大圆环不加半宽
            ImageDeal[Ysite].Center =
                (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /
                2; //板块
            if (ImageDeal[Ysite].Center > 79)
              ImageDeal[Ysite].Center = 79;
            if (ImageDeal[Ysite].Center < 5)
              ImageDeal[Ysite].Center = 5;
            break;
          }
        }
        if (ImageDeal[Ysite].Wide > 8 &&
            ImageDeal[Ysite].LeftBorder > ImageDeal[Ysite + 2].LeftBorder) {
          continue;
        } else {
          ImageStatus.OFFLine = Ysite + 2;
          break;
        }
      }
    }
  }
  //环内不处理
  if (ImageFlag.image_element_rings_flag == 7) {
  }
  //大圆环出环 补线
  if (ImageFlag.image_element_rings_flag == 8) //大圆环
  {
    Repair_Point_Xsite = 30;
    Repair_Point_Ysite = 7;
    for (int Ysite = 40; Ysite > 8; Ysite--) {
      if (img1[Ysite][65] == 1 && img1[Ysite - 1][65] == 0) {
        Repair_Point_Xsite = 65;
        Repair_Point_Ysite = Ysite - 1;
        ImageStatus.OFFLine = Ysite + 1; //截止行重新规划
        break;
      }
    }

    for (int Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--) //补线
    {
      ImageDeal[Ysite].LeftBorder =
          (Repair_Point_Xsite - ImageDeal[58].LeftBorder) * (58 - Ysite) /
              (58 - Repair_Point_Ysite) +
          ImageDeal[58].LeftBorder;
      ImageDeal[Ysite].Center =
          (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
    }
    if (ImageFlag.image_element_rings_flag == 9) {
      for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
        ImageDeal[Ysite].Center =
            ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
      }
    }
  }
}

void Element_Judgment_Zebra() //斑马线判断
{
  if (ImageFlag.Zebra_Flag)
    return;
  int net = 0;

  for (int Ysite = 40; Ysite < 50; Ysite++) {
    net = 0;
    for (int Xsite = 0; Xsite < 79; Xsite++) {
      if (img1[Ysite][Xsite] != img1[Ysite][Xsite + 1] &&
          ImageStatus.OFFLine < 10) {
        net++;
      }
    }
    if (net > 8 && ImageFlag.Zebra_Flag == 0) {
      ImageFlag.Zebra_Flag = 1;
      Garage_Location_Flag++;
      break;
    }
  }
}
void Element_Handle_Zebra() //斑马线处理
{
  if (Garage_Location_Flag < Garage_num)

  {
    if (ImageStatus.Barn_Lenth >
        Zebra_num1) //第一次斑马线发车，当识别到编码器记录的路程大于设定的路程时，把车库标志位和编码器清零
    {
      ImageFlag.Zebra_Flag = 0;
      ImageStatus.Barn_Lenth = 0;
    }
  } else if (Garage_Location_Flag >= Garage_num) {
    if (ImageStatus.Barn_Lenth > Zebra_num2) {
      ImageStatus.Road_type = Barn_in;
      ImageFlag.Zebra_Flag = 0;
      ImageStatus.Barn_Lenth = 0;
    }
  }
}

//停车
void Stop_Test() {
  if (ImageStatus.Stop_lenth * OX > 50 && ImageFlag.image_element_rings == 0) {

    //      if (ImageStatus.OFFLine >= 55)
    //      //如果电磁很小并且可视距离基本没有就表示出界
    //           SystemData.Stop = 1;
    // else SystemData.Stop = 0;
    if (ImageStatus.OFFLine >= 55) //如果电磁很小并且可视距离基本没有就表示出界
      SystemData.Stop = 1;
  }
}

//停车弱保护
void Stop_Test2() { //弱保护
  if (ImageStatus.OFFLine >= 55 && SystemData.Stop == 0 &&
      SystemData.SpeedData.Length * OX > 150)
    SystemData.Stop = 2;

  if (SystemData.Stop == 2 && ImageStatus.Stop_lenth * OX > 80 &&
      ImageStatus.OFFLine >= 55)
    SystemData.Stop = 1;
  else if (SystemData.Stop == 2 && ImageStatus.Stop_lenth * OX > 80 &&
           ImageStatus.OFFLine < 55)
    SystemData.Stop = 0;
}

void Element_Handle() {
  if (ImageFlag.image_element_rings == 1 && ImageStatus.Road_type == LeftCirque) {
    Element_Handle_Left_Rings();
  } else if (ImageFlag.image_element_rings == 2 && ImageStatus.Road_type == RightCirque) {
    std::cout << "error";
  }

  Element_Handle_Zebra(); //斑马线停车处理
}

//用于加速的直道检测

void Straightacc_Test(void) {
  int sum = 0;
  for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--) {
    sum += (ImageDeal[Ysite].Center - ImageStatus.MiddleLine) *
           (ImageDeal[Ysite].Center - ImageStatus.MiddleLine);
  }
  variance_acc = (float)sum / (54 - ImageStatus.OFFLine);
  //直线加速
  if (variance_acc < ImageStatus.variance_acc && ImageStatus.OFFLine <= 15 &&
      ImageStatus.Left_Line < 10 && ImageStatus.Right_Line < 10) {
    ImageStatus.straight_acc = 1;
  } else
    ImageStatus.straight_acc = 0;

  //判断直线
  if (variance_acc < ImageStatus.variance && ImageStatus.OFFLine <= 15 &&
      ImageStatus.Left_Line < 10 && ImageStatus.Right_Line < 10) {
    ImageStatus.straight_acc_flag = 1;
  } else
    ImageStatus.straight_acc_flag = 0;
  // printf("off_line:%d    ",ImageStatus.OFFLine);
  // printf("_acc:%.2f     ",variance_acc);
  // printf("l:%d     ",ImageStatus.Left_Line);
  // printf("r:%d     \n",ImageStatus.Right_Line);
}

//用于变参数的直道检测。短直线有BUG而且没必要
void Straight_Test_2(void) {
  float midd_k, sum;
  midd_k = (ImageDeal[55].Center - ImageDeal[ImageStatus.OFFLine + 1].Center) /
           (float)(55 - ImageStatus.OFFLine - 1);
  for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--) {
    ImageDeal[Ysite].mid_temp =
        ImageDeal[55].Center - midd_k * (55 - Ysite) + 0.5;
    sum += pow(ImageDeal[Ysite].Center - ImageDeal[Ysite].mid_temp, 2);
  }

  variance = sum / (54 - ImageStatus.OFFLine);
  if (variance < ImageStatus.variance && ImageStatus.OFFLine <= 9) {
    ImageStatus.Road_type = Straight;
  }
}

void Element_Test() {
  if (ImageFlag.image_element_rings == 0) {
    Element_Judgment_Zebra();
  }

  if (ImageStatus.Road_type !=
      Ramp) /*&& circle_num < 2 */ //圆环检测//&& ImageStatus.Road_type != Ramp
  {
    Element_Judgment_Left_Rings();  //左圆环检测
    Element_Judgment_Right_Rings(); //右圆环检测
  }
}

//要打印的数据
static void get_imgdisplay() {
  for (Ysite = 59; Ysite >= 0; Ysite--) {
    // for(int x=0;x<=79;x++)
    // {
    //   imgdisplay[Ysite][x]=0;
    // }
    img3[Ysite][ImageDeal[Ysite].Center] = 6;
    img3[Ysite][ImageDeal[Ysite].LeftBorder] = 7;
    img3[Ysite][ImageDeal[Ysite].RightBorder] = 8;
  }
  for (int i = 0; i < 80; i++) {
    // img3[ImageStatus.TowPoint][i] = 9;
    img3[ImageStatus.TowPoint_True][i] = 9;
  }
}

int imageprocess(void) {
  ImageStatus.OFFLine = 2; //这个值根据真实距离得到，必须进行限制//图像顶边
  ImageStatus.WhiteLine = 0; //双边丢边数
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].IsLeftFind = 'F';  //左边有边
    ImageDeal[Ysite].IsRightFind = 'F'; //右边有边
    ImageDeal[Ysite].LeftBorder = 0;    //边界
    ImageDeal[Ysite].RightBorder = 79;
    ImageDeal[Ysite].LeftTemp = 0; //临时边界
    ImageDeal[Ysite].RightTemp = 79;
    ImageDeal[Ysite].close_LeftBorder = 0; //靠边边界
    ImageDeal[Ysite].close_RightBorder = 79;
  }
  drawfirstline(); //绘制底边
  DrawLinesProcess();
  Search_Border_OTSU(img1, CAMERA_H, CAMERA_W,
                     CAMERA_H - 2); // 58行位底行//八邻域爬边
  // Straightacc_Test();//直线检测
  //  if(ImageStatus.Road_type != LeftCirque&&ImageStatus.Road_type !=
  //  RightCirque&&ImageStatus.Road_type != Ramp) {Element_Test();}//元素检测
  Element_Test();
  if (ImageFlag.image_element_rings == 0) {
    DrawExtensionLine();
  }

  RouteFilter();      //中线平滑滤波
  Straightacc_Test(); //直线加速
  Element_Handle();
  Stop_Test();
  GetDet();
  Stop_Test2();
  get_imgdisplay(); //获取要打印的数据
  // printf("stop:%.2f\n",SystemData.SpeedData.Length);
  // printf("stop_BZW:%d\n",SystemData.Stop)
  // debug(ImageStatus.TowPoint_True);
  // debug()
  // debug(ImageStatus.TowPoint_True);
  // debug(circle_count_flag);
  if (ImageFlag.image_element_rings_flag) {
  }
  // if (ImageFlag.image_element_rings_flag == 0) {
  //   return ImageDeal[ImageStatus.TowPoint_True].Center;
  // } else if (ImageFlag.image_element_rings_flag > 0 && ImageFlag.image_element_rings_flag <= 2){
  //   return ImageDeal[ImageStatus.TowPoint + 10].Center + 10;
  // } else {
  //   return ImageDeal[ImageStatus.TowPoint_Gain].Center;
  // }
  if (ImageFlag.is_flip) {
    ImageStatus.Det_True = 2 * ImageStatus.MiddleLine - ImageStatus.Det_True;
  }
  // debug(ImageStatus.TowPoint_True);
  // debug(ImageStatus.OFFLine);
  return ImageStatus.Det_True;
}