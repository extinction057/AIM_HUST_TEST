//
// Created by angrypsyduck on 2021/9/12.
//

#ifndef ARMORDETECTOR_TOOL_CVEX_H
#define ARMORDETECTOR_TOOL_CVEX_H
#include<iostream>
#include<opencv2/opencv.hpp>
#include<cmath>
namespace cvex
{
    void subtractBGR(cv::Mat &src,cv::Mat &dst,int color,float threshold);
    void inRangeHSV(cv::Mat &src,cv::Mat &dst,int colour);
}
#endif //ARMORDETECTOR_TOOL_CVEX_H
