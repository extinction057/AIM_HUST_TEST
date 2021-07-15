#ifndef __TOOL_H
#define __TOOL_H

#include "tool_time.h"
#include "tool_config.h"
#include <opencv2/opencv.hpp>

namespace ly{
    // 接收数据格式
    class receiveData{
    public:
        unsigned char flag;
        int yaw;
        short pitch;
        clock_t Data_Time;
    };

    // 发送数据格式
    class sendData{
    public:
        unsigned char shootStatus;
        int yaw;
        short pitch;
        unsigned char distance;
    };

    typedef struct{
        cv::Mat mat;
        receiveData receiveData_;
        clock_t time;
    } Mat;
}

#endif //__TOOL_H

