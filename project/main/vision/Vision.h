#ifndef _VISON_H_
#define _VISON_H_
#include "Bin.h"
#include "Line.h"
#include "Perspective.h"

#define VISION_DEBUG 1
enum Type {
    LINE,
    CURVE,
};

typedef struct{
    int center;
    Type type;
}vision_result;

vision_result process_img(cv::Mat frame);

#endif