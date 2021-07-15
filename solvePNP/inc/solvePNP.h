#ifndef __SOLVE_PNP_H
#define __SOLVE_PNP_H

#include<opencv2/opencv.hpp>
#include <vector>
#include "tool_config.h"
#include "armor.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "transform.h"
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"


namespace ly
{
    class solvePNP
    {
    public:
        solvePNP();
        solvePNP(cam_param config,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite);
        solvePNP(double fx, double fy, double u0, double v0, double k_1, double  k_2, double  p_1, double  p_2, double k_3);
        ~solvePNP();
        void solve(ly::armorNode armor,int type,receiveData receiveData_);
        void solve_BUFF(receiveData receiveData_,std::vector<cv::Point2f> Points2D_,double angle);
        Sophus::SE3<double> solve_BUFF_3D(double x,double y,double r,receiveData receiveData_,std::vector<cv::Point2f> Points2D_);
        Sophus::SE3<double> solve_BUFF_Circle(receiveData receiveData_,std::vector<cv::Point2f> Points2D_);
        cv::Mat solve_BUFF_A(receiveData receiveData_,Sophus::SE3<double> tf);
        void setCameraMatrix(double fx, double fy, double u0, double v0);
        void setDistortionCoefficients(double k_1, double  k_2, double  p_1, double  p_2, double k_3);
        float get_pitch();
        float get_yaw();
        void setTgc(float pitch, float yaw, float roll);
        Sophus::SE3<double> TF;
        cv::Mat TF_Mat;
        cv::Mat rvec;//解出来的旋转向量
        cv::Mat tvec;//解出来的平移向量
        cv::Mat Tgc;
        time test;
        double times;
    private:
        serialPortReadThread *serialPortRead_;
        serialPortWriteThread *serialPortWrite_;
        std::vector<cv::Point3f> Points3D;//存储四个点的世界坐标
        std::vector<cv::Point2f> Points2D;//存储四个点的图像坐标

        cv::Mat camera_matrix;//内参数矩阵
        cv::Mat distortion_coefficients;//畸变系数
        transform *transform_;
        std::FILE *file;
    };
}






#endif //__SOLVE_PNP_H
