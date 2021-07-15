#include "camera.h"
#include <iostream>
#include "tools.h"
#include<opencv2/opencv.hpp>
#include <utility>

namespace ly
{
    long long int camera::frame_id_ = 0;

    // cv::Mat DAH_Camera::getFrame()
    // {
    //     return cv::Mat();
    // }
    /**
     * @brief ���λ�ȡͼƬ
     * @return ͼƬ
     */

    Mat picture::getFrame()
    {
        setUpdateTime(frame_counter_.getTimeMs());
        std::stringstream ss;
//        ss<<pic_path_<<"/"<<getFrameId()<<".jpg"; // ƴ���ַ���
        ss<<pic_path_; // ƴ���ַ���
        auto pic = cv::imread(ss.str());
        updateId();
        frame_counter_.countBegin();
        return Mat();
    }
    picture::picture(std::string pic_path)
    {
        pic_path_ = std::move(pic_path);
    }

    /**
     * @brief ��ȡ��Ƶ�е�һ֡
     * @return ͼƬ
     */
    Mat video::getFrame()
    {
        setUpdateTime(frame_counter_.getTimeMs());
        cv::Mat temp;
        cap_>>frame_.mat;
        updateId();
        frame_counter_.countBegin();
        return frame_;
    }
    /**
     * @brief ���캯��
     * @param video_path ��Ƶ·��
     */
    video::video(const std::string& video_path)
    {
        cap_.open(video_path);
        if(!cap_.isOpened())//�����Ƶ�����������򷵻�
        {
            std::cerr<<"error !!! can not open the video!"<<std::endl;
            return ;
        }
        else
        {
            std::cout<<"open video success !"<<std::endl<<"path:"<<video_path<<std::endl;
        }
    }
}





