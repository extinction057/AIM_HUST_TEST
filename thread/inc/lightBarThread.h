#ifndef __LIGHT_BAR_THREAD_H
#define __LIGHT_BAR_THREAD_H
#include "lightBar.h"
#include "thread.h"
#include "camera.h"
#include "preProcessThread.h"
#include "armor.h"
#include "cameraThread.h"
#include "buff.h"
namespace ly
{
    class lightBarThread:public lightBar,public thread
    {
    public:
        explicit lightBarThread(cam_param cam_config,lightBar_param config,cameraThread *camera,preProcessThread *preProcess,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,int enemy);
        lightBarThread() = default;
        Mat pic_;
        /*!
         * @name getLightBarQue
         * @brief 如果需要更新且当前lightbar队列不为空，就更新；否则就返回空队列
         * @param none
         * @return lightbar队列
         */
        std::priority_queue<lightBarNode> getLightBarQue() override;
        /*!
         * @name setArmorQue
         * @brief 将armor储存到armor_中
         * @param armor
         * @return none
         */
        void setArmorQue(const std::priority_queue<armorNode> armor);
        cv::Mat show_;
    private:
        cv::Mat show_temp;
        cameraThread *camera_ = nullptr;
        preProcessThread *preProcess_ = nullptr;
        serialPortWriteThread* serialPortWrite_ = nullptr;
        serialPortReadThread* serialPortRead_ = nullptr;
        std::priority_queue<armorNode> armor_;
        void getGammaCorrection(cv::Mat& src, cv::Mat& dst, const float fGamma);
        void process() override;
        void getPreQue();
        void updatePreQue();
        void debug_show();
        void update_lightBar(std::priority_queue<lightBarNode> node);
        float gamma;
        time counter_;                                                  //计时器
        bool debug_show_ROI = false;                                     //可视化ROI
        bool debug_show_light = false;                                  //可视化lightbar
        bool debug_show_armor = true;                                   //可视化armor
        std::priority_queue<lightBarNode> preQue_;                      //预处理的结果队列
        bool update_ = false;                                           //检查更新标志
        BUFF_Detector *buff_Tool;
        std::vector<receiveData> receive_Data ;
        int colour;
    };
}
#endif //__LIGHT_BAR_THREAD_H
