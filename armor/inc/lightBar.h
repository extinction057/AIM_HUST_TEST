#ifndef __LIGHT_BAR_
#define __LIGHT_BAR_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "tools.h"
#include "tool_cvex.h"
#include <queue>
#include <vector>
#include "score.h"
#include <ctime>

#define DEBUG_INFO(STR)\
std::cout<<STR<<std::endl;\

namespace ly
{
    typedef struct lightBarNode
    {
        /**
         * 评价灯条的优先级的指标
         * ---面积，在一定范围内面积越大得分越高，过大则为零
         * ---离光心的马氏距离，从0到最大值，得分从1到0线性分布
         * ---装甲的角度，从0到90，得分从1到0线性分布
         * ---长宽比
         */
        float lengthWidthRatio = 0;                                         //长宽比,1:1--->0;6:1--->1
        float angle = 0;                                                    //角度
        float area = 0;                                                     //面积 200
        float distance = 0;                                                 //离光心的像素平面上的欧式距离
        float LWR_Ceo = 0;
        float angle_Ceo = 0;
        float area_Ceo = 0;
        float distance_Ceo = 0;
        float length = 0;
        float width = 0;
        int disable = 0;
        cv::Point2f point[4] = {};
        cv::Point2f center{};
        float sum = 0;
        receiveData receiveData_;
        clock_t time{};
        friend bool operator < (lightBarNode a, lightBarNode b)         //重载小于号
        {
            return a.sum < b.sum;
        }
        lightBarNode(){}
    }lightBarNode;
    class lightBar
    {
    public:
        lightBar() = default;
        lightBar(lightBar_param config);
        ~lightBar();
        void init(lightBar_param config);
        virtual std::priority_queue<lightBarNode> getLightBarQue()=0;
        std::vector<cv::RotatedRect> getLightBar(){ return lightBars_;}

        lightBarNode detect(Mat &rawPic,int colour,cv::Point2f center=cv::Point2f(0,0),int length=0,int width=0,float useROI=0);
        void detect(Mat &rawPic,std::priority_queue<lightBarNode> preLBQ,int colour);

        std::priority_queue<lightBarNode>lightBarsQue_;
        std::priority_queue<lightBarNode>copy_lightBarsQue_;
    protected:
        score *LWR_Ceo_;
        score *angle_Ceo_;
        score *area_Ceo_;
        score *distance_Ceo_;
        lightBar_param config_;
        std::vector<cv::RotatedRect> lightBars_;
        std::priority_queue<lightBarNode> temp_LBQ_;
        int thresh_;
        cv::Point2f ROI_Pt_;
        /*!
         * @brief 更新lightBar
         * @param node src
         * @return none
         */
        void update_lightBar(std::priority_queue<lightBarNode> node);
        bool NOLightBar=false;
        cv::Mat pryDownNoG(cv::Mat rawPic,float rateX,float rateY);
        float pryRateX=3;
        float pryRateY=3;
        int downPicCols=300;
        float downPicXYRate=1.2;
    };
}






#endif //__LIGHT_BAR_
