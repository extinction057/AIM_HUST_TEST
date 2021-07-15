#ifndef _RM_BUFF_
#define _RM_BUFF_
#include <iostream>
#include <opencv2/opencv.hpp>
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/eigen.hpp"
#include "tools.h"
#include "solvePNP.h"

namespace ly
{
    struct RM_tar_Armor
    {
        cv::RotatedRect self_Elli;//????
        cv::RotatedRect farther_Elli;//??????????
        cv::RotatedRect self_Rect;
        cv::RotatedRect farther_Rect;
        cv::RotatedRect self_Fit_rect;
        cv::Point2f offset_Pt;//offset point for self_Fit_rect
        float cross_Ang;//???????????????????????????????????????????90????
        float self_Ang;//????????????0~360??
        std::vector<cv::Point2f> armor_Pts;//???��???????????   ???????��??
        double Time;
        cv::Point2f center;
        std::vector<cv::Point2f> self_Fit_poly;
        double rotAng;
        double rotAng_pre;
        double rotSpdA;
        double angle;
    };

    struct predict_
    {
        cv::Point2f Point_now;
        cv::Point2f Point_pre;
        double angle_now;
        double angle_pre;
        double pre_r;
        int quadrant;
    };

    struct circle_3D
    {
        //ax+by+cz-1=0原平面
        //C(x,y,z)为圆心坐标
        double Cx;
        double Cy;
        double Cz;
        double a;
        double b;
        double c;
        double radius;
    };

    struct Target_3D
    {
        Eigen::Vector3d Point;
        double angle;
        double speed;
        double Time;

    };

    struct Table
    {
        double yaw;
        double pitch;
        double angle;

    };

    class RM_BUFF_Dec
    {
    public:
        RM_BUFF_Dec(std::string config_Path,int color_Type,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,cam_param cam_config);
        bool process(Mat src);
        cv::Mat show_Res(cv::Mat &src_Input);
        void cal_Ang(receiveData receiveData_);
        void reset_All();
        std::vector<double> x_Data,y_Data;
        bool ispredict;
    private:
        void init_Img();
        void sortpoint(std::vector<cv::Point2f> Point);
        bool find_Armor();//find
        bool search_Pts(RM_tar_Armor &cur_Armor);
        bool fit_Traj();
        bool fit_Traj_world();
        bool fit_Traj_3D();
        bool fit_SpdSin();
        bool fit_SpdC();
        bool predict_SpdC();
        bool fit_SpdC_3D();
        void update_Order();
        void setimu(float pitch, float yaw, float roll);
        void setCameraMatrix(double fx, double fy, double u0, double v0);
        void setDistortionCoefficients(double k_1, double  k_2, double  p_1, double  p_2, double k_3);


        Sophus::SE3<double> world_;
        Sophus::SE3<double> imu_;
        Sophus::SE3<double> camera_;
        Sophus::SE3<double> gimbal_;
        Sophus::SE3<double> circle_;
        Sophus::SE3<double> gimbal_to_cam_;
        cv::Point2f circle_center;//Բ��
        std::vector<cv::Point3f> Points3D;//�洢�ĸ������������
        std::vector<cv::Point2f> Points2D;//�洢�ĸ����ͼ������
        cv::Mat rvec;//���������ת����
        cv::Mat tvec;//�������ƽ������
        cv::Mat camera_matrix;//�ڲ�������
        cv::Mat distortion_coefficients;//����ϵ��
        Sophus::SE3<double> TF;
        Sophus::SE3<double> TF_3D;
        Sophus::SE3<double> TF_circle;
        cv::Mat TF_3D_Mat;
        cv::Mat A;
        cv::Mat A_ORI;
        solvePNP *pnp_;
        Table table;
        bool isfitspd;
        double averSpd;

        bool isfirsttfA;
        bool isinitialize;
        int m_Color;
        int color_Thresh_blue,color_Thresh_red;
        int armor_Pts_min,armor_Pts_max,far_Armor_min,far_Armor_max;
        int ele_Size;
        int fitMode=1;
        cv::Mat element,ele4blue,ele4red;
        cv::Mat src_Img,cur_Img,real_Src,src_temp;
        cv::Mat buff_show;
        std::vector<RM_tar_Armor> targets;
        RM_tar_Armor preTarget;

        std::vector<double>time_Data,spd_Data;
        bool fitSpdBeginFlag=true;
        std::vector<std::vector<cv::Point2f> > long_Set;
        std::vector<Eigen::Vector3d> long_Set_3D;
        cv::Point2f  predictCenter;
        double param_r,param_x0,param_y0;//r^2=(x-x0)^2+(y-y0)^2
        bool isfind_Armor,isfind_Traj;
        double cam_U0,cam_V0,cam_F;
        double firstpitch,firstyaw;

        cv::Rect armor_Roi;
        bool isget_Roi,isuse_Roi;
        cv::Point2f Roi_Offset;
        float lenth = 0.7f;
        serialPortReadThread *serialPortRead_;
        serialPortWriteThread *serialPortWrite_;
        predict_ predict[4];
        cv::Point2f pre_Point[4];
        receiveData receiveData_now;
        circle_3D circle_3D_;
        Target_3D Target_3D_;
        Target_3D Target_3D_pre;
        Target_3D Target_3D_predict;
        transform *transform_;
        bool isprespd;
        double speed;
    };

    struct CICLE_FITTING_COST
    {
        CICLE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
        // �в�ļ���
        template <typename T>
        bool operator() (
                const T* const x0y0r,     // ģ�Ͳ�������3ά
                T* residual ) const     // �в�
        {
            //(x-x0)^2+(y-y0)^2-r^2
            residual[0]=x0y0r[2]-(T ( _x )-x0y0r[0])*(T ( _x )-x0y0r[0])-(T ( _y )-x0y0r[1])*(T ( _y )-x0y0r[1]);
            return true;
        }
        const double _x, _y;    // x,y����
    };

    struct CURVE_FITTING_COST
    {
        float spd_A,spd_W;
        CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
        template <typename T>
        bool operator() (
                const T* const abc,     // parameter to estimate
                T* residual) const     // �в�
        {
            residual[0] = T(_y) - (0.785 * sin(1.884 * T(_x) + abc[0]) + abc[1]);//spd_A*sin(spd_W*t+est_C1)+spd_C
            return true;
        }
        const double _x, _y;    //x->time y->spd
    };


}
#endif