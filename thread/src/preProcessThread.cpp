#include <tools.h>
#include "preProcessThread.h"

namespace ly
{
    preProcessThread::preProcessThread(lightBar_param config,cameraThread *camera,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,int enemy):
            camera_(camera),serialPortRead_(serialPortRead),serialPortWrite_(serialPortWrite),colour(enemy)
    {
        init(std::move(config));                //继承自lightar
        start(10000);//100hz                    //开始线程
    }

    void preProcessThread::process()
    {
        pic_ = camera_->getFrame();
        if(serialPortRead_->getReceiveMsg().empty())
        {return;}
        if(pic_.mat.empty())
        {return;}
        receive_Data = serialPortRead_->getReceiveMsg();
        time_t end = receive_Data.at(149).Data_Time;

        double t_ms = double(end-pic_.time)/CLOCKS_PER_SEC*1000;
        //std::cout<<"time: "<<t_ms<<std::endl;
        t_ms = fmax(0,fmin(t_ms,149));
        pic_.receiveData_ = receive_Data.at(149-(int)t_ms);
//        if(receive_Data.end()->flag==0x05)
//        {
//            colour = 0;//red
//        }
//        else if(receive_Data.end()->flag==0x06)
//        {
//            colour = 1;//blue
//        }
//        colour = 1;//blue
        detect(pic_,colour);                           //全图检测灯条
        std::priority_queue<lightBarNode> roi_ = ROI(temp_LBQ_);
        mutex_ROI_.lock();
        ROI_ = roi_;                  //根据检测到的灯条计算roi
        mutex_ROI_.unlock();
        update_ = true;
    }
    std::priority_queue<lightBarNode> preProcessThread::getLightBarQue()
    {
        mutex_ROI_.lock();
        std::priority_queue<lightBarNode> roi = ROI_;
        mutex_ROI_.unlock();
        return roi;
    }
    bool preProcessThread::checkUpdate()
    {
        if(update_)
        {
            update_ = false;
            return true;
        }
        else
        {
            return false;
        }
    }
    std::priority_queue<lightBarNode> preProcessThread::ROI(std::priority_queue<lightBarNode> lightBar)
    {
        std::priority_queue<lightBarNode> ROI;
        auto lightBar_copy = lightBar;
        while (!lightBar_copy.empty())
        {
            lightBarNode bestLightBar = lightBar_copy.top();
            // 计算roi宽度
            bestLightBar.width = bestLightBar.width*width_ceo;
            bestLightBar.length = bestLightBar.length*length_ceo;
            for(auto & i : bestLightBar.point)
            {
                // 计算roi的四个点（可以考虑改进成一个点存储）
                i.x = fmin(fmax(width_ceo*(i.x - bestLightBar.center.x) + bestLightBar.center.x,0),pic_.mat.rows);
                i.y = fmin(fmax(length_ceo*(i.y - bestLightBar.center.y) + bestLightBar.center.y,0),pic_.mat.cols);
            }
            ROI.push(bestLightBar);
            lightBar_copy.pop();
        }
        return ROI;
    }

}

