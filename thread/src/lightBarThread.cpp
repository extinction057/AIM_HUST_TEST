#include <tools.h>
#include "preProcessThread.h"
#include "lightBarThread.h"

namespace ly
{
    lightBarThread::lightBarThread(cam_param cam_config,lightBar_param config,cameraThread *camera,preProcessThread *preProcess,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,int enemy) :
    lightBar(config),camera_(camera),preProcess_(preProcess),serialPortRead_(serialPortRead),serialPortWrite_(serialPortWrite),colour(enemy)
    {
        gamma = config.gamma;
        //buff_Tool=new BUFF_Detector("/home/zjl/code/RM2021/AIM_HUST-BUFF/config/new_Buff_cfg.yml",serialPortRead_,serialPortWrite_,cam_config);
        init(std::move(config));
        start(1000);
    }
    void lightBarThread::process() {
        //时间同步
        pic_ = camera_->getFrame();
        if(pic_.mat.empty())
        {return;}
//        if(serialPortRead_->getReceiveMsg().empty())
  //      {return;}
//        receive_Data = serialPortRead_->getReceiveMsg();
//        time_t end = receive_Data.at(149).Data_Time;
//
//        double t_ms = double(end-pic_.time)/CLOCKS_PER_SEC*1000;
//        t_ms = fmax(0,fmin(t_ms,149));
//        pic_.receiveData_ = receive_Data.at(149-(int)t_ms);



        //大符模式 @TODO 大小符模式切换
//        if(receive_Data.end()->flag==0x07||receive_Data.end()->flag==0x08)
//        {
//
//        cv::Point3f est_Pt;
//        if(pic_.mat.empty())
//        {return;}
//        buff_Tool->process(pic_,est_Pt);
//        }
//
//
//

          //辅瞄模式
//        else if(receive_Data.end()->flag==0x05||receive_Data.end()->flag==0x06)
//        {
//            if(receive_Data.end()->flag==0x05)
//            {
//                colour = 0;//red
//            }
//            else if(receive_Data.end()->flag==0x06)
//            {
//                colour = 1;//blue
//            }


//            cv::Mat temp = pic_.mat.clone();
//            float fGamma = 1/2.2;
//            getGammaCorrection(temp,pic_.mat, fGamma);
            if (pic_.mat.empty())
                return;
           show_ = pic_.mat.clone();
            getPreQue();                            //获取预处理得到的前一帧的roi
            detect(pic_, preQue_,colour);        //在roi中检测lightBar
            updatePreQue();                         //用检测到的lightBar更新roi
            mutex_lightBar_.lock();
            copy_lightBarsQue_ = lightBarsQue_;     //更新lightBar
            mutex_lightBar_.unlock();
            update_ = true;
            debug_show();
//        }

    }
    void lightBarThread::getPreQue()
    {
        if(preProcess_->checkUpdate())
        {
            preQue_ = preProcess_->getLightBarQue();        //这里实际上返回的是roi
        }
    }
    void lightBarThread::updatePreQue()
    {
        preQue_ = preProcess_->ROI(lightBarsQue_);          //这里实际上返回的是roi
    }
    void lightBarThread::debug_show()
    {
        if(debug_show_light || debug_show_armor || debug_show_ROI)
        {
            //show_ =show_temp.clone();
            if(debug_show_light)                                //显示LightBar
            {
                std::priority_queue<lightBarNode> show_lightBar = lightBarsQue_;
                int times = show_lightBar.size() > 5 ? 5 : (int) show_lightBar.size();
                for (int i = 0; i < times; ++i)
                {
                    if (show_lightBar.empty())
                        continue;
                    cv::Point2f rect_points[4];
                    for (int j = 0; j < 4; j++)
                    {
                        line(show_, show_lightBar.top().point[j], show_lightBar.top().point[(j + 1) % 4], 255, 2, 8);
                    }
                    std::stringstream ss;
                    ss << show_lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(show_, num, show_lightBar.top().point[0], cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0,255,0));
                    show_lightBar.pop();
                }
            }
            if (!armor_.empty() && debug_show_armor)            //显示armor
            {
                std::stringstream ss;
                ss << armor_.top().lightBarRatio;
                std::string num = ss.str();
//                if (armor_.top().sum > 5) {
                    cv::putText(show_,num,armor_.top().center,cv::FONT_HERSHEY_COMPLEX,0.8,255);
                    cv::putText(show_, "0", armor_.top().corner[0], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                    cv::putText(show_, "1", armor_.top().corner[1], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                    cv::putText(show_, "2", armor_.top().corner[2], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                    cv::putText(show_, "3", armor_.top().corner[3], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);

                    for (int j = 0; j < 4; j++) {
                        line(show_, armor_.top().corner[j], armor_.top().corner[(j + 1) % 4], 255, 2, 8);
//                    }
                }
            }
            if(debug_show_ROI)                                  //显示ROI
            {
                std::priority_queue<lightBarNode> show_lightBar = preQue_;

                int times = show_lightBar.size()>5?5:(int)show_lightBar.size();
                for (int i=0; i<times; ++i)
                {
                    if(show_lightBar.empty())
                        continue;
                    cv::Point2f rect_points[4];
                    for( int j = 0; j < 4; j++ )
                    {
                        line(show_, show_lightBar.top().point[j], show_lightBar.top().point[(j+1)%4],255, 2, 8 );
                    }
                    std::stringstream ss;
                    ss<<i;//lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(show_,num,show_lightBar.top().point[0],cv::FONT_HERSHEY_COMPLEX,0.8,255);
                    show_lightBar.pop();
                }
            }
            cv::namedWindow("lightBar debug",1);
            cv::imshow("lightBar debug", show_);
            cv::waitKey(10);
        }
    }
    std::priority_queue<lightBarNode> lightBarThread::getLightBarQue()
    {
        if(update_ && !copy_lightBarsQue_.empty())
        {
            update_ = false;
            mutex_lightBar_.lock();
            auto temp = copy_lightBarsQue_;
            mutex_lightBar_.unlock();
            return temp;
        }
        return std::priority_queue<lightBarNode>();
    }
    void lightBarThread::update_lightBar(std::priority_queue<lightBarNode> node)
    {
        mutex_lightBar_.lock();
        lightBarsQue_ = node;
        mutex_lightBar_.unlock();
    }
    void lightBarThread::setArmorQue(const std::priority_queue<armorNode> armor)
    {
        armor_ = armor;
    }
    void lightBarThread::getGammaCorrection(cv::Mat& src, cv::Mat& dst, const float fGamma)
    {
        unsigned char bin[256];
        for (int i = 0; i < 256; ++i)
        {
            bin[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
        }
        dst = src.clone();
        const int channels = dst.channels();
        switch (channels)
        {
            case 1:
            {
                cv::MatIterator_<uchar> it, end;
                for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
                    *it = bin[(*it)];
                break;
            }
            case 3:
            {
                cv::MatIterator_<cv::Vec3b> it, end;
                for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++)
                {
                    (*it)[0] = bin[((*it)[0])];
                    (*it)[1] = bin[((*it)[1])];
                    (*it)[2] = bin[((*it)[2])];
                }
                break;
            }
        }
    }
}


