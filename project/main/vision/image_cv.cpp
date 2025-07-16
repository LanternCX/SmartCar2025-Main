#include "image_cv.hpp"
#include "image.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/videoio.hpp>
#include <unistd.h>
#include "Debug.h"

using namespace cv;
using namespace std;

// 创建一个摄像头
VideoCapture cap;
// 获取视频流
Mat frame;

char img[60][80];
char img3[60][80]; //显示图像的数组
char img4[60][160];
char img5[120][160];

//摄像头初始化，成功返回1，失败返回-1；
char image_cv_Init(void) {
  cap = VideoCapture(0);
  //打开摄像头
  if (!cap.isOpened()) {
    cerr << "Error open video stream" << endl;
    return -1;
  }
  // 设置视频流编码器
  cap.set(cv::CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
  // 设置摄像头图像宽高和帧率
  cap.set(CAP_PROP_FRAME_WIDTH, 320);
  cap.set(CAP_PROP_FRAME_HEIGHT, 240);
  cap.set(CAP_PROP_FPS, 160);

  //设置摄像头的曝光
  // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);   //非完全关闭自动曝光
  // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);   //完全关闭自动曝光
  // cap.set(cv::CAP_PROP_EXPOSURE, 300);      //设置曝光

  // 获取摄像头图像宽高和帧率
  int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  double frame_fps = cap.get(cv::CAP_PROP_FPS);
  printf("frame:%d*%d, fps:%3f", frame_width, frame_height, frame_fps);
  sleep(1);
  return 1;
}

char image_cv_zip(cv::Mat &src) {
  // 读取摄像头一帧图像
  // cap.read(frame);    // cap >> frame;
  // cap >> frame;
  src.copyTo(frame);
  if (frame.empty()) {
    cerr << "Error read frame" << endl;
    return -1;
  }
  int height = frame.rows;
  int width = frame.cols;

  // 裁切区域从顶部开始，高度为 3/4
  cv::Rect roi(0, height * 1 / 3, width, height - height * 1 / 3);
  // debug(width, height);
  frame = frame(roi).clone();

  frame.copyTo(src);

  // 转化为灰度
  Mat gray;
  // frame为彩色输入图像，gray为灰度输出图像
  cvtColor(frame, gray, COLOR_BGR2GRAY);
  // 二值化处理
  Mat binary;
  // gray为输入的灰度图像，binary为二值化后输出的图像
  threshold(gray, binary, 0, 1, THRESH_BINARY + THRESH_OTSU);
  //第一个binary为要压缩的输入图像，第二个binary为压缩后的输出图像，第三个是压缩为80*60的大小
  resize(binary, binary, Size(160, 120));

    // 形态学滤波
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

  if (ImageFlag.is_flip) {
    flip(binary, binary, 1);
  }
  // auto start2 = std::chrono::high_resolution_clock::now();
  // cv::threshold(gray, binary1, 0, 1, THRESH_BINARY + THRESH_OTSU);
  // cv::Mat resized2;
  // resize(binary1, binary1, Size(80, 60));
  // auto end2 = std::chrono::high_resolution_clock::now();
  // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2
  // - start2).count(); std::cout << "先二值化再压缩耗时: " << duration2 << "
  // 毫秒" << std::endl;

  for (int i = 0, p = 0; i < 120; i += 2, p++) {
    for (int j = 0, k = 0; j < 160; j += 2, k++) {
      img1[p][k] = binary.at<unsigned char>(i, j);
      img3[p][k] = binary.at<unsigned char>(i, j);
      // printf("%d",binary.at<unsigned char>(i, j));
    }
    // printf("\n");
  }
  // printf("\n\n\n");

  return 1;
}

VideoCapture & getCap() {
  return cap;
}