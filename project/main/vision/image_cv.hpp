#ifndef __IMAGE_CV_H__
#define __IMAGE_CV_H__

#include <opencv2/opencv.hpp>

extern char img[60][80];
extern char img3[60][80];
extern char img4[60][160];
extern char img5[120][160];

char image_cv_Init(void);
char image_cv_zip(cv::Mat &src);
cv::VideoCapture & getCap();

#endif // !__