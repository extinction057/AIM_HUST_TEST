#ifndef __PRE_PROCESS_THREAD_H
#define __PRE_PROCESS_THREAD_H

#include "lightBar.h"
#include "thread.h"
#include "camera.h"
#include "cameraThread.h"
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"

namespace ly
{

    class preProcessThread:public lightBar,public thread
    {
    public:
        /*!
         * @brief 构造函数
         * @param config 从配置文件中读取的配置
         * @param camera
         * @return none
         */
        explicit preProcessThread(lightBar_param config,cameraThread *camera,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,int enemy);
        /*!
         * @name getLightBarQue
         * @brief 获取roi区域
         * @param none
         * @return roi区域
         */
        std::priority_queue<lightBarNode> getLightBarQue() override ;       // override:对lightBarThread中函数的重写
        /*!
         * @name checkupdate
         * @brief 检查update_的状态，获取是否需要更新的信息
         * @param none
         * @return none
         */
        bool checkUpdate();
        /*!
         * @name ROI
         * @brief 取出bestLightBar，根据width_ceo和height_ceo计算ROI
         * @param lightBar 存储检测到的lightBar的优先队列
         * @return ROI区域
         */
        std::priority_queue<lightBarNode> ROI(std::priority_queue<lightBarNode> lightBar);
    private:
        cameraThread *camera_ = nullptr;
        serialPortReadThread* serialPortRead_ = nullptr;
        serialPortWriteThread* serialPortWrite_ = nullptr;
        /*!
         * @name process
         * @brief 检测灯条，计算roi
         * @param none
         * @return none
         */
        void process();
        Mat pic_;
        std::priority_queue<lightBarNode> ROI_;
        float width_ceo = 3;                        //全图检测时roi宽度的放大倍数
        float length_ceo = 1.5;                       //全图检测时roi长度的放大倍数
        bool update_ = false;
        time counter_;
        std::vector<receiveData> receive_Data ;
        int colour;
    };
}

#endif //__PRE_PROCESS_THREAD_H
