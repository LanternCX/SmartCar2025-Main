#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <time.h>
#include <chrono> // 用于高精度时间测量
#include "zf_common_headfile.h"

using namespace cv;
using namespace std;
using namespace std::chrono; // 使用chrono命名空间

#define CAMERA_H  60
#define CAMERA_W  80

// typedef unsigned char uint8;
// typedef unsigned short uint16;
// typedef unsigned long uint32;
// typedef signed short int int16; 
// typedef long int32;
// typedef signed char int8;

extern long szr;

#define uchar unsigned char
#define uint unsigned int

