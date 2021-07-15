/**
 * @file transform.cpp
 * @brief solve transform of robot and world
 *
 *        camera --------------
 *          |                  \
 *          |                   \
 *        gimbal -- shooter    armor
 *          |
 *        world
 *
 * @note
 *         ^ z
 *         |
 *         |
 * <-------/
 * x      /
 *       / y
 *  y -> positive forward,unit m
 *  pangolin： x:red,y:green,z:blue
 * @author lyc
 */

#include "eigen3/Eigen/Dense"
#include <opencv2/core/eigen.hpp>
#include "transform.h"
#include "table.h"
#include "../../thread/inc/serialPortWriteThread.h"
#include "../../thread/inc/serialPortReadThread.h"
#include <lcm/lcm-cpp.hpp>

#define BIAS_PITCH 0
#define BIAS_YAW 0.2
#define KF_REBOOT_THRESH 0.5
#define KF_STATE_X KF_.statePost.at<float>(0)
#define KF_STATE_Y KF_.statePost.at<float>(1)
#define KF_STATE_VX KF_.statePost.at<float>(2)
#define KF_STATE_VY KF_.statePost.at<float>(3)

cv::Point2d measurePoint;
cv::Point2d statePoint;
cv::Point2f pointBuff[6];
float zBuff[9] = {0};
float filterCOE[6]={0.0264077249,0.1405313627,0.3330609123,0.3330609123,0.1405313627,0.0264077249};
float zfilterCOE[9]={0.01720767,0.0475656475,0.1221292039,0.19813310412,0.22992874818,0.19813310412,0.1221292039,0.0475656475,0.01720767};
namespace ly
{
    extern const double table_angle[2937];
    extern const double table_pitch[2937];
    extern const double table_yaw[2937];
    transform::transform(serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite):
            serialPortRead_(serialPortRead),serialPortWrite_(serialPortWrite)
    {
        world_ = Sophus::SE3<double>(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0));
        imu_ = Sophus::SE3<double>(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0));
        //辅瞄暂时不使用IMU
        // 如果要用陀螺仪 gimbal_ = imu_;
        gimbal_ = imu_;
        //@TODO 改成用配置文件
        //Eigen::Vector3d shooter_trans(0,0.1,0.05);
        Eigen::Vector3d camera_trans(0,0.1085,0.05941);

        //shooter_ = Sophus::SE3<double>(Eigen::Matrix3d::Identity(),shooter_trans);
        gimbal_to_cam_ = Sophus::SE3<double>(Eigen::Matrix3d::Identity(),camera_trans);

        armor_ = world_;
        measurement_ = cv::Mat::zeros(4, 1, CV_32F);
        KF_ = cv::KalmanFilter(4,4,0,CV_32F);
        state_ = cv::Mat::zeros(4,1,CV_32F);
        processNoise_ = cv::Mat::zeros(4,1,CV_32F);

        KF_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
                                                      1, 0, 1, 0,//A 状态转移矩阵
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1);
        setIdentity(KF_.measurementMatrix);
        setIdentity(KF_.processNoiseCov, cv::Scalar::all(1e-5));
        setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(1e-1));
        setIdentity(KF_.errorCovPost, cv::Scalar::all(1));
        randn(KF_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
        sendData_.pitch = 0;
        sendData_.yaw = 0;
        sendData_.distance = 1;
//#ifdef DEBUG_TRANS //visual debug
//        pthread_t tids;
//        int ret = pthread_create(&tids, NULL, visual, (void*)this);
//#endif
    }
    void transform::setimu(float pitch, float yaw, float roll)
    {
        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
        gimbal_ = Sophus::SE3<double>(rotation_matrix3,Eigen::Vector3d(0,0,0));
    }
    void transform::setArmor2Cam(Sophus::SE3<double> armor2cam,receiveData receiveData_)
    {
        float ppp = receiveData_.pitch/100.0f*3.14159f/180.0f;
        float yyy = receiveData_.yaw/100.0f*3.14159f/180.0f;
        setimu(ppp,yyy,0);
        camera_ = gimbal_*gimbal_to_cam_;
        armor_ = camera_*armor2cam;


        if (sqrt(pow(last_armor_.translation()[0] - armor_.translation()[0], 2) + pow(last_armor_.translation()[1] - armor_.translation()[1], 2) + pow(last_armor_.translation()[2] - armor_.translation()[2], 2)) > KF_REBOOT_THRESH)
        {
//std::cout<<"--------------------------KF Reboot------------"<<std::endl;

            KF_.statePre.at<float>(0) = armor_.translation()[0];
            KF_.statePre.at<float>(1) = armor_.translation()[1];
            KF_.statePre.at<float>(2) = 0;
            KF_.statePre.at<float>(3) = 0;
            KF_.statePost.at<float>(0) = armor_.translation()[0];
            KF_.statePost.at<float>(1) = armor_.translation()[1];
            KF_.statePost.at<float>(2) = 0;
            KF_.statePost.at<float>(3) = 0;
            setIdentity(KF_.errorCovPost, cv::Scalar::all(1));
            setIdentity(KF_.errorCovPre, cv::Scalar::all(1));
//std::cout<<"--------------------------KF Reboot------------"<<std::endl;
        }


        last_armor_.translation()[0] = armor_.translation()[0];
        last_armor_.translation()[1] = armor_.translation()[1];
        last_armor_.translation()[2] = armor_.translation()[2];

        KF_prediction = KF_.predict();

/**************ADD FIR************************/
        for(int i=0;i<5;i++)
        {
            pointBuff[i] = pointBuff[i+1];
        }
        for(int i=0;i<8;i++)
        {
            zBuff[i] = zBuff[i+1];
        }
        pointBuff[5] = cv::Point2f(armor_.translation()[0],armor_.translation()[1]);
        zBuff[8] = armor_.translation()[2];
//printf("znew   %f3\n",armor_.translation()[2]);
        measurement_.at<float>(0)=0;
        measurement_.at<float>(1)=0;
        float z = 0;

        for(int i=0;i<6;i++)
        {
            measurement_.at<float>(0) += filterCOE[i]*(pointBuff[i].x);
            measurement_.at<float>(1) += filterCOE[i]*(pointBuff[i].y);
        }
        for(int i=0;i<9;i++)
        {
            z += zfilterCOE[i]*(zBuff[i]);

        }

/**********************************************/


        measurePoint.x = measurement_.at<float>(0)*100+512;
        measurePoint.y = -measurement_.at<float>(1)*100+384;
        //printf("measX	%f3	measY	%f3\n",armor_.translation()[0],	armor_.translation()[1]);
        //printf("filterX	%f3	filterY	%f3\n",measurePoint.x,measurePoint.y);
        //printf("%f3	%f3\n",measurePoint.x,measurePoint.y);
        KF_.correct(measurement_);

//        float distance = sqrt(pow(KF_prediction.at<float>(0),2) + pow(KF_prediction.at<float>(1),2) + pow(KF_prediction.at<float>(2),2));
        float distance = sqrt(pow(armor_.translation()[0],2) + pow(armor_.translation()[1],2) + pow(armor_.translation()[2],2));
        delta_t = distance / 18.5;
        float predictX = KF_STATE_X ;//+ KF_STATE_VX*580*delta_t;
        float predictY = KF_STATE_Y ;//+ KF_STATE_VY*580*delta_t;

        distance = sqrt(pow(predictX,2) + pow(predictY,2));// + pow(armor_.translation()[2],2));

//       std::cout<<delta_t<<std::endl;

        statePoint = cv::Point2d(KF_.statePost.at<float>(0) * 100 + 512, -KF_.statePost.at<float>(1) * 100 + 384);

        float dy = distance;
        float dz = 0.02646*dy*dy-0.03556*dy-0.0009681;
        float lifted = z + dz;

        pitch = atan2(lifted, distance)*180.0f/3.14159f+BIAS_PITCH;

        yaw = -atan2(predictX, predictY)*180.0f/3.14159f+BIAS_YAW;

        sendData_.pitch = pitch*100;
        sendData_.yaw = yaw*100;
        sendData_.distance = distance*10;

        if(debug_ <5)
        {
            debug_++;
        }
        else
        {
            if(sendData_.yaw>-100000&&sendData_.yaw<100000&&sendData_.pitch<100000&&sendData_.pitch>-100000)
            {
/*	std::cout<<"send.y:"<<armor_.translation()[1]<<"send.pitch:"<<sendData_.pitch<<"send.distance:"<<sendData_.distance <<"send.z:"<<armor_.translation()[2]<<"send.x:"<<armor_.translation()[0]<<std::endl;
*/

                //if(coutCnt%2 == 0)
                {

//		printf("STM Pitch	%f3	STM Yaw	%f3\n",ppp*180.0f/3.14159f,yyy*180.0f/3.14159f);
//		printf("send Pitch	%f3	sendYaw	%f3\n",pitch,yaw);
//		printf("predictX	%f3	predictY	%f3\n",predictX,predictY);
//		printf("distance	%f3	z	%f3\n",distance,z);

                }
                serialPortWrite_->setSendMsg(sendData_);
            }
        }

    }





    void transform::trans_BUFF(Sophus::SE3<double> armor2cam,receiveData receiveData_,double angle)
    {
        //std::cout<<"11pitch:"<<receiveData_.pitch/100.0f<<std::endl;
        float ppp = (receiveData_.pitch/100.0f)*3.14159f/180.0f;
        float yyy = receiveData_.yaw/100.0f*3.14159f/180.0f;
        //std::cout<<"receive_pitch"<<receiveData_.pitch/100.0f<<std::endl;
        //std::cout<<"receive_yaw"<<receiveData_.yaw/100.0f<<std::endl;
        setimu(0,0,0);
        camera_ = gimbal_*gimbal_to_cam_;
        armor_ = camera_*armor2cam;
        //publish();
        ballistic_equation(ppp);

        static float pitch_ori = receiveData_.pitch;
        static float yaw_ori = receiveData_.yaw;

        setimu(ppp,yyy,0);
        camera_ = gimbal_*gimbal_to_cam_;
        armor_ = camera_*armor2cam;

        distance = sqrt(pow(armor_.translation()[0],2) + pow(armor_.translation()[1],2) + pow(armor_.translation()[2],2));
        //pitch = atan2(armor_.translation()[2], distance);
        yaw = -atan2(armor_.translation()[0], armor_.translation()[1]);

        float delta_angle_min = 100;
        int delta_angle_minnum = 0;
        for(int i=0;i<2937;i++)
        {
            if(abs(table_angle[i]-angle)<delta_angle_min)
            {
                delta_angle_min = abs(table_angle[i]-angle);
                delta_angle_minnum = i;
            }
            if(delta_angle_min<0.3)
            {
                break;
            }
        }
        sendData_.pitch = table_pitch[delta_angle_minnum];
        sendData_.yaw = table_yaw[delta_angle_minnum];
        cv::Mat shoot = cv::Mat::zeros(100,100,CV_8UC3);
        cv::imshow("shooting",shoot);
        char c =cv::waitKey(20);
        if(c == 'q')
        {
            sendData_.shootStatus = 1;
        }
        else
        {
            sendData_.shootStatus =0;
        }
        if(sendData_.yaw>-100000&&sendData_.yaw<100000&&sendData_.pitch<100000&&sendData_.pitch>-100000)
        {
            serialPortWrite_->setSendMsg(sendData_);
        }


    }

    void transform::trans_BUFF_amend(double x,double y,double r,Sophus::SE3<double> armor2cam,double yaw,receiveData receiveData_)
    {
        //std::cout<<"11pitch:"<<receiveData_.pitch/100.0f<<std::endl;
        float ppp = (receiveData_.pitch/100.0f)*3.14159f/180.0f;
        float yyy = receiveData_.yaw/100.0f*3.14159f/180.0f;
//        std::cout<<"receive_pitch"<<receiveData_.pitch/100.0f<<std::endl;
//        std::cout<<"receive_yaw"<<receiveData_.yaw/100.0f<<std::endl;
        setimu(ppp,yyy,0);
        camera_ = gimbal_*gimbal_to_cam_;
        armor_ = camera_*armor2cam;


        cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) <<
                                                          armor_.rotationMatrix()(0,0), armor_.rotationMatrix()(0,1), armor_.rotationMatrix()(0,2),
                armor_.rotationMatrix()(1,0), armor_.rotationMatrix()(1,1), armor_.rotationMatrix()(1,2),
                armor_.rotationMatrix()(2,0), armor_.rotationMatrix()(2,1), armor_.rotationMatrix()(2,2)
        );
        cv::Mat R;
        cv::Rodrigues(rotation_matrix,R);
        yaw = R.at<double>(1);
        //std::cout<<"yaw"<<yaw<<std::endl;
        //std::cout<<"send.y:"<<armor_.translation()[1] <<"send.z:"<<armor_.translation()[2]<<"send.x:"<<armor_.translation()[0]<<std::endl;

        static int bias_y = 50;
        static int bias_x = 50;
        static int bias_z = 50;

        cv::Mat buff_show =  cv::Mat::zeros(800,800,CV_8UC3);
        float buff_f = 0.79f + (float)(bias_y-50)/100;
        double temp_y = 1264*buff_f/sqrt(r);
        double temp_x,temp_z;
        temp_x = temp_y*(x-640)/1264  + (float)(bias_x-50)/100 + 0.13;
        temp_z = temp_y*(-y+512)/1264 + (float)(bias_z-50)/100 - 0.07;


        double V = 300.f/buff_f *(armor_.translation()[2] - temp_z);
        double U = 300.f/buff_f *(armor_.translation()[0] - temp_x);

        // draw circle
//        cv::circle(buff_show,cv::Point(400,400),300,cv::Scalar(255,255,255));
//        cv::circle(buff_show,cv::Point(400,400),10,cv::Scalar(255,255,255));
//        cv::circle(buff_show,cv::Point(U+400,-V+400),10,cv::Scalar(255,255,255));
//        cv::createTrackbar("r","buff",&bias_y,100);
//        cv::createTrackbar("x","buff",&bias_x,100);
//        cv::createTrackbar("z","buff",&bias_z,100);
//        cv::imshow("buff",buff_show);


        //std::cout<<"shoot:"<<(int)sendData_.shootStatus<<std::endl;
        sendData_.pitch = receiveData_.pitch;
        sendData_.yaw = yaw*100*180.0f/3.14159f;
        //std::cout<<"send.y:"<<receiveData_.pitch<<"send.p:"<<sendData_.pitch<<std::endl;
//        serialPortWrite_->setSendMsg(sendData_);


    }


    // 根据物理方程来计算设定pitch和yaw
    Eigen::Vector2f transform::ballistic_equation(float gim_pitch)
    {
        Eigen::Vector2f result;
        // 先计算yaw轴角度
        yaw = atan(armor_.translation()[0] /armor_.translation()[1]);
        // armor 的位置进行了一定的旋转
        armor_.translation() << 0,sqrt(armor_.translation()[0] * armor_.translation()[0] + armor_.translation()[1] * armor_.translation()[1]), armor_.translation()[2];
        // 旋转到世界坐标系
        armor_.translation() << 0,armor_.translation()[1]*cos(gim_pitch)-armor_.translation()[2]*sin(gim_pitch),armor_.translation()[1]*sin(gim_pitch)+armor_.translation()[2]*cos(gim_pitch);
        //std::cout<<"armor[2] "<<armor_.translation()[2]<<"armor[1]"<<armor_.translation()[1]<<std::endl;
        //std::cout<<"receive_pitch"<<gim_pitch/3.1415926*180<<std::endl;
        // 计算pitch轴的初始角度
        pitch = atan2(armor_.translation()[2], armor_.translation()[1]);
        //std::cout<<"armor[2] "<<armor_.translation()[2]<<"armor[1]"<<armor_.translation()[1]<<std::endl;
        //std::cout<<"pitch"<<pitch/3.1415926*180<<std::endl;
        float err = 100;
        float delta=0;
        // 迭代计算pitch轴角度
        for(int i = 0;i < 5;i ++)
        {// 多次迭代计算得到目标位置
            // 前向弹道方程
            auto target = forward_ballistic_equation(pitch,armor_.translation()[1]);
            err = armor_.translation()[2] - target;
            if(fabs(err) < 0.001)
            {
                break;
            }
            // 计算导数
            float J = derivation(pitch,armor_.translation()[1]);
            float d_theta = - err / J;
            // 更新设定角度
            pitch += d_theta;
            delta+=d_theta;
        }
        result << pitch/3.14159*180,yaw/3.14159*180;
//        td::cout<<"ss"<<delta<<std::endl;
        return result;
    }

// 高中物理公式
    float transform::forward_ballistic_equation(float angle,float x)
    {
        float time = x / (m_set_speed * cos(angle));
        return m_set_speed * sin(angle) * time - 0.5 * 9.8 * time * time;
    }


    float transform::derivation(float angle,float x)
    {
        return -x / pow(cos(angle),2) +1.5*x*x/(m_set_speed*m_set_speed)*1/pow(cos(angle),3);
    }

#ifdef DEBUG_TRANS
    void* transform::visual(void *__this)
    {

        transform * _this =(transform *)__this;
        pangolin::CreateWindowAndBind("Main");
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
                pangolin::ModelViewLookAt(-2,-2,-2, 0,0,0, pangolin::AxisZ)
        );
        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                .SetHandler(&handler);
        while(true)
        {
            try
            {
                // Clear screen and activate view to render into
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);
                //draw
                _this->glDrawColouredAxis(_this->world_,0.08);
                _this->glDrawColouredAxis(_this->camera_,0.08);
                _this->glDrawColouredAxis(_this->gimbal_,0.5);
                //_this->glDrawColouredAxis(_this->imu_,0.5);
                //_this->glDrawColouredAxis(_this->shooter_,0.08);
                _this->glDrawColouredCuboid(_this->armor_,0.08);
                // Swap frames and Process Events
                pangolin::FinishFrame();
                usleep(20000);//50hz
            }
            catch (...)
            {
                break;
            }
        }
    }
    void transform::glDrawColouredAxis(const Sophus::SE3<double> T, float length = 0.8)
    {
        std::vector<Eigen::Vector3d> axis;
        axis.emplace_back(Eigen::Vector3d(0.0f,0.0f,0.0f));
        axis.emplace_back(Eigen::Vector3d(length,0.0f,0.0f));
        axis.emplace_back(Eigen::Vector3d(0.0,length,0.0f));
        axis.emplace_back(Eigen::Vector3d(0.0,0.0f,length));
        std::vector<Eigen::Vector3d> trans_axis(axis);
        for(int i = 0;i<4;i++)
        {
            trans_axis.at(i) = T*axis.at(i);
        }
        glLineWidth(3);
        glColor3f ( 0.8f,0.f,0.f );
        glBegin ( GL_LINES );
        glVertex3d(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3d(trans_axis.at(1)[0],trans_axis.at(1)[1],trans_axis.at(1)[2]);
        glColor3f( 0.f,0.8f,0.f);

        glVertex3d(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3d(trans_axis.at(2)[0],trans_axis.at(2)[1],trans_axis.at(2)[2]);
        glColor3f( 0.2f,0.2f,1.f);

        glVertex3d(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3d(trans_axis.at(3)[0],trans_axis.at(3)[1],trans_axis.at(3)[2]);
        glEnd();
    }
    void transform::glDrawColouredCuboid(const Sophus::SE3<double> T, GLfloat a,GLfloat b,GLfloat c)
    {
        std::vector<Eigen::Vector3d> points={Eigen::Vector3d(a,b,-c),Eigen::Vector3d(-a,b,-c),
                                             Eigen::Vector3d(a,b,c),Eigen::Vector3d(-a,b,c),
                                             Eigen::Vector3d(a,-b,c),Eigen::Vector3d(-a,-b,c),
                                             Eigen::Vector3d(a,-b,-c),Eigen::Vector3d(-a,-b,-c)};
        std::vector<Eigen::Vector3d> trans_points(points);
        for(int i=0;i<points.size();i++)
        {
            trans_points.at(i) = T*points.at(i);
        }
        GLfloat  verts[] =
                {
                        (float)trans_points.at(0)[0],(float)trans_points.at(0)[1],(float)trans_points.at(0)[2],
                        (float)trans_points.at(1)[0],(float)trans_points.at(1)[1],(float)trans_points.at(1)[2],
                        (float)trans_points.at(2)[0],(float)trans_points.at(2)[1],(float)trans_points.at(2)[2],
                        (float)trans_points.at(3)[0],(float)trans_points.at(3)[1],(float)trans_points.at(3)[2],
                        // FRONT
                        (float)trans_points.at(6)[0],(float)trans_points.at(6)[1],(float)trans_points.at(6)[2],
                        (float)trans_points.at(7)[0],(float)trans_points.at(7)[1],(float)trans_points.at(7)[2],
                        (float)trans_points.at(4)[0],(float)trans_points.at(4)[1],(float)trans_points.at(4)[2],
                        (float)trans_points.at(5)[0],(float)trans_points.at(5)[1],(float)trans_points.at(5)[2],
                        // BACK
                        (float)trans_points.at(0)[0],(float)trans_points.at(0)[1],(float)trans_points.at(0)[2],
                        (float)trans_points.at(2)[0],(float)trans_points.at(2)[1],(float)trans_points.at(2)[2],
                        (float)trans_points.at(6)[0],(float)trans_points.at(6)[1],(float)trans_points.at(6)[2],
                        (float)trans_points.at(4)[0],(float)trans_points.at(4)[1],(float)trans_points.at(4)[2],
                        // LEFT
                        (float)trans_points.at(7)[0],(float)trans_points.at(7)[1],(float)trans_points.at(7)[2],
                        (float)trans_points.at(5)[0],(float)trans_points.at(5)[1],(float)trans_points.at(5)[2],
                        (float)trans_points.at(1)[0],(float)trans_points.at(1)[1],(float)trans_points.at(1)[2],
                        (float)trans_points.at(3)[0],(float)trans_points.at(3)[1],(float)trans_points.at(3)[2],
                        // RIGHT
                        (float)trans_points.at(2)[0],(float)trans_points.at(2)[1],(float)trans_points.at(2)[2],
                        (float)trans_points.at(3)[0],(float)trans_points.at(3)[1],(float)trans_points.at(3)[2],
                        (float)trans_points.at(4)[0],(float)trans_points.at(4)[1],(float)trans_points.at(4)[2],
                        (float)trans_points.at(5)[0],(float)trans_points.at(5)[1],(float)trans_points.at(5)[2],
                        // TOP
                        (float)trans_points.at(0)[0],(float)trans_points.at(0)[1],(float)trans_points.at(0)[2],
                        (float)trans_points.at(6)[0],(float)trans_points.at(6)[1],(float)trans_points.at(6)[2],
                        (float)trans_points.at(1)[0],(float)trans_points.at(1)[1],(float)trans_points.at(1)[2],
                        (float)trans_points.at(7)[0],(float)trans_points.at(7)[1],(float)trans_points.at(7)[2],
                        // BOTTOM
                };

        glVertexPointer(3, GL_FLOAT, 0, verts);//verts中每次取三个数据（一个点的坐标）
        glEnableClientState(GL_VERTEX_ARRAY);

        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);

        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);

        glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);

        glDisableClientState(GL_VERTEX_ARRAY);

        // draw the original axis
        std::vector<Eigen::Vector3d> axis;
        axis.push_back(Eigen::Vector3d(0.0f,0.0f,0.0f));
        axis.push_back(Eigen::Vector3d(0.8,0.0f,0.0f));
        axis.push_back(Eigen::Vector3d(0.0,0.8f,0.0f));
        axis.push_back(Eigen::Vector3d(0.0,0.0f,0.8f));
        std::vector<Eigen::Vector3d> trans_axis(axis);
        for(int i = 0;i<4;i++)
        {
            trans_axis.at(i) = T*axis.at(i);
        }
        glLineWidth(3);
        glColor3f ( 0.8f,0.f,0.f );
        glBegin ( GL_LINES );
        glVertex3f(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3f(trans_axis.at(1)[0],trans_axis.at(1)[1],trans_axis.at(1)[2]);
        glColor3f( 0.f,0.8f,0.f);

        glVertex3f(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3f(trans_axis.at(2)[0],trans_axis.at(2)[1],trans_axis.at(2)[2]);
        glColor3f( 0.2f,0.2f,1.f);

        glVertex3f(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3f(trans_axis.at(3)[0],trans_axis.at(3)[1],trans_axis.at(3)[2]);
        glEnd();
    }


#endif
}
