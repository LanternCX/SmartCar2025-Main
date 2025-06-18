#ifndef IMAGE_H
#define IMAGE_H

#include"base.hpp"

extern int img1[CAMERA_H][CAMERA_W];//图像数组
extern int imgdisplay[CAMERA_H][CAMERA_W];//图像数组

extern double fps;

extern Mat erzhihua(Mat img);
extern void printfshuzu(void) ;



#endif