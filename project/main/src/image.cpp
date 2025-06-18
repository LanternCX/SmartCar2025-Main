#include"image.hpp"
#include"imgdeal.hpp"
#include"encoder.hpp"
#include"motor.hpp"
#include"steer.hpp"
#include "base.hpp"
int img1[CAMERA_H][CAMERA_W];//ͼ������
int imgdisplay[CAMERA_H][CAMERA_W];//ͼ������

double fps;

std::string ImageStatusToString(RoadType_e Road_type) 
{

        switch (Road_type) 
        {
            case zero:
                return "zero";
            case Normol:
                return "Normol";
            case Straight:
                return "Straight";
            case Cross:
                return "Cross";
            case Ramp:
                return "Ramp";
            case LeftCirque:
                return "LeftCirque";
            case RightCirque:
                return "RightCirque";
            case Cross_ture:
                return "Cross_ture";
            default:
                return "Unknown";
        }
    
}

Mat erzhihua(Mat img)
{
    Mat img_gaussian, img_gray, img_threshold;
    cvtColor(img, img_gray, COLOR_BGR2GRAY);
    GaussianBlur(img_gray, img_gaussian, Size(5, 5), 0);
    threshold(img_gaussian, img_threshold, 0, 255, THRESH_BINARY + THRESH_OTSU);
        resize(img_threshold, img_threshold, Size(80, 60));
        for (int y = 0; y < CAMERA_H; y++) 
        {
           for (int x = 0; x < CAMERA_W; x++) 
           {
               img1[y][x] = img_threshold.at<uchar>(y, x);
               if (img1[y][x] == 0)
               {
                   img1[y][x] = 0;
               }
               else
               {
                   img1[y][x] = 1;
               }
           }
        }
    return img_threshold;
}


void printfshuzu() 
{
    ImageStatus.offline=ImageStatus.OFFLine;
    cout << endl << endl;
    cout << "fps:" << fps << "    ";
    // cout << "����:" << ImageStatus.offline << endl;
    // cout << "ǰհ:" << ImageStatus.TowPoint_True << "    ";
    // cout << "�е�:" << ImageStatus.Det_True << "    ";

    // cout << "ImageStatus.OFFLine:"<< static_cast<int>(ImageStatus.OFFLine);
    // cout << "ImageStatus.Left_Line:" << static_cast<int>(ImageStatus.Left_Line) << "    " << "ImageStatus.Right_Line:" << static_cast<int>(ImageStatus.Right_Line) << endl;
    // cout << "Right_RingsFlag_Point1_Ysite:" << Right_RingsFlag_Point1_Ysite << "    " << "Right_RingsFlag_Point2_Ysite:" << Right_RingsFlag_Point2_Ysite << endl;
    // cout << "Բ������:" << ImageFlag.image_element_rings_flag <<"    "<< "���" << szr;

    std::cout << "The current road type is: " << ImageStatusToString(ImageStatus.Road_type) << std::endl;
    cout<<"�������"<<speed_l<<"     "<<"�ұ�����"<<speed_r<<"     "<<"Speed_Goal";
    cout<<"���ٶ�"<<Speed_Goal_r<<"     "<<"���ٶ�"<<Speed_Goal_r<<"     "<<"Speed_Goal"<<Speed_Goal<<endl;
    cout<<"��PWM"<<Speed_PID_OUT_l<<"     "<<"��PWM"<<Speed_PID_OUT_r<<endl;
    cout<<"ƫ��"<<ImageStatus.Det_True-39<<"ǰհ"<<ImageStatus.TowPoint<<endl;
    cout << endl << endl;
    for (int y = 0; y < CAMERA_H; y+=2)
    {
        stringstream ss;
        for (int x = 0; x < CAMERA_W; x++)
        {
            ss << img1[y][x];
        }
        string result = ss.str();
        cout << result << endl;
    }
}
