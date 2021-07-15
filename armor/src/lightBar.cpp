/**
 * @file lightBar.cpp
 * @brief
 *
 */

#include "lightBar.h"


namespace ly
{
    lightBar::lightBar(lightBar_param config)
    {
        init(config);
    }
    void lightBar::init(lightBar_param config)
    {
        config_ = std::move(config);
        LWR_Ceo_ = new score(config_.lengthWidthRadio);
        angle_Ceo_ = new score(config_.angle);
        area_Ceo_ = new score(config_.area);
        distance_Ceo_ = new score(config_.distance);
        thresh_ = config_.thresh;
        ROI_Pt_ = cv::Point2f(0,0);
    }
    lightBar::~lightBar()
    {
        delete(LWR_Ceo_);
        delete(angle_Ceo_);
        delete(area_Ceo_);
        delete(distance_Ceo_);
    }
    void lightBar::detect(Mat &rawPic, std::priority_queue<ly::lightBarNode> preLBQ,int colour)
    {
        if(rawPic.mat.empty())                              //原始图像为空就返回
            return;
        std::priority_queue<lightBarNode> temp_lightBars;
        ly::lightBarNode last_lightBar;                     //
        last_lightBar.center = cv::Point2f(0,0);
//        DEBUG_INFO(preLBQ.top().center)
        if(preLBQ.empty()||preLBQ.top().sum<1.5)            //没有前一帧的检测数据或前一帧的检测数据很差
        {
//            pryRateX=(float)rawPic.mat.cols/downPicCols;
//            pryRateY=pryRateX*downPicXYRate;
//            cv::Mat downPic;
//            downPic=pryDownNoG(rawPic.mat,pryRateX,pryRateY);
//            rawPic.mat=downPic.clone();
//            NOLightBar=true;
            detect(rawPic,colour);                                 //全图检测
            temp_lightBars = temp_LBQ_;
            update_lightBar(temp_lightBars);
            //std::cout<<"no ROI"<<std::endl;
            return;
        }
        else{                                               //先前帧有合适的roi
            int times = preLBQ.size()>5?5:(int)preLBQ.size();
            for(int i=0;i<times;i++)
            {
                auto length = (int)(preLBQ.top().length);
                auto width = (int)(preLBQ.top().width);
                auto useROI = (int)2;
                float distance = (preLBQ.top().center.x - last_lightBar.center.x)*(preLBQ.top().center.x - last_lightBar.center.x)+
                                 (preLBQ.top().center.y - last_lightBar.center.y)*(preLBQ.top().center.y - last_lightBar.center.y);
                if(distance<10)                             //两帧检测到的灯条位置接近，看成一个灯条
                {
                    preLBQ.pop();
                }
                last_lightBar = preLBQ.top();
                temp_lightBars.push(detect(rawPic,colour,preLBQ.top().center,width,length,useROI));
                preLBQ.pop();
            }
            update_lightBar(temp_lightBars);
        }
    }
    void lightBar::update_lightBar(const std::priority_queue<lightBarNode> node)
    {
        lightBarsQue_ = node;
    }

    lightBarNode lightBar::detect(Mat &rawPic,int colour,cv::Point2f center, int length, int width, float useROI)
    {
        cv::Mat ROI;

        if (rawPic.mat.empty())                                 //原图为空
        {
            std::cerr << "detect rawPic.empty()" << std::endl;
            return temp_LBQ_.top();
        }


        if(useROI > 1 && width > 0 && length > 0)               //放大倍数大于1就计算ROI
        {
            // 计算出roi的左上角坐标，放入ROI_PT_中
            float roi_left_x = (center.x-length/2.0f*useROI)<1?1:(center.x-length/2.0f*useROI);
            float roi_left_y = (center.y-width/2.0f*useROI)<1?1:(center.y-width/2.0f*useROI);
            // ROI_Pt 左上角点坐标
            ROI_Pt_ = cv::Point2f(roi_left_x,roi_left_y);
            // 计算ROI的宽高，并作边界检查
            length = (int)(length*useROI+ROI_Pt_.x)>rawPic.mat.cols?rawPic.mat.cols:(int)(length*useROI+ROI_Pt_.x);
            width = (int)(width*useROI+ROI_Pt_.y)>rawPic.mat.rows?rawPic.mat.rows:(int)(width*useROI+ROI_Pt_.y);
            // 将ROI取出放入rawPic.mat
            ROI = rawPic.mat(cv::Range((int)ROI_Pt_.y,width),cv::Range((int)ROI_Pt_.x,length));
        }
        else                                                 //不使用ROI，全图作为roi
        {
            ROI_Pt_ = cv::Point2f(0,0);
            pryRateX=(float)rawPic.mat.cols/downPicCols;
            pryRateY=pryRateX*downPicXYRate;
            cv::Mat downPic;
            downPic=pryDownNoG(rawPic.mat,pryRateX,pryRateY);
   //         rawPic.mat=downPic.clone();
            ROI = downPic;
            NOLightBar=true;
        }
        cv::Mat subtract_dst;                           //红蓝通道相减得到的图
        cv::Mat thresh=cv::Mat::zeros(ROI.rows,ROI.cols,CV_8UC1);                                 //二值化后的单通道图像
        cv::Mat RoiHSV=cv::Mat::zeros(ROI.rows,ROI.cols,CV_32FC3);
        std::vector<cv::Mat> rgb_vec;                   //三通道的颜色向量
        std::vector<std::vector<cv::Point> > contours;  //轮廓点集
        time lightBar_time;                             //计时器

        lightBar_time.countBegin();
//#define RGB
#define HSV
#ifdef HSV
            //色相
            int hmin = 0;//R0 B0-100
            int hmin_Max = 180;
            int hmax = 10;//R0-110 B120-180
            int hmax_Max = 180;
            //饱和度
            int smin = 43;//R0-212 B0-240
            int smin_Max = 255;
            int smax = 255;//R255 B255
            int smax_Max = 255;
            //亮度
            int vmin = 46;//R40 B46
            int vmin_Max = 255;
            int vmax = 255;//R150-255 B230-255
            int vmax_Max = 255;
            if(colour==0) {
                //色相
                hmin = 0;//R0 B0-100 MAX180
                hmax = 10;//R0-110 B120-180 MAX180
                //饱和度
                smin = 43;//R0-212 B0-240 MAX255
                smax = 255;//R255 B255 MAX255
                //亮度
                vmin = 46;//R40 B46 MAX255
                vmax = 255;//R150-255 B230-255 MAX255
            }
            else {
                //色相
                hmin = 100;//R0 B0-100 MAX180
                hmax = 124;//R0-110 B120-180 MAX180
                //饱和度
                smin = 43;//R0-212 B0-240 MAX255
                smax = 255;//R255 B255 MAX255
                //亮度
                vmin = 46;//R40 B46 MAX255
                vmax = 255;//R150-255 B230-255 MAX255
            }
            cv::cvtColor(ROI,ROI,CV_BGR2HSV);
            cv::inRange(ROI, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax),thresh);
//            cvex::inRangeHSV(ROI,thresh,colour);
//        cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3));
//        cv::dilate(thresh,thresh,kernel,cv::Point(0,-1));
#endif

#ifdef RGB
        cvex::subtractBGR(ROI,thresh,colour,thresh_);
//        if(colour == 1)                     //需要检测蓝色装甲
//        {
//            cv::split(ROI,rgb_vec);
//            cv::subtract(rgb_vec[0], rgb_vec[2], subtract_dst);
//            cv::threshold(subtract_dst, thresh, thresh_, 255, cv::THRESH_BINARY );      //thresh_在config里面配置
//        }
//        else if (colour == 0)                //需要检测红色装甲
//        {
//            cv::split(ROI,rgb_vec);
//            cv::subtract(rgb_vec[2], rgb_vec[0], subtract_dst);
//            cv::threshold(subtract_dst, thresh, thresh_, 255, cv::THRESH_BINARY );
//        }
#endif
        cv::imshow("hsv",thresh);
        lightBar_time.countEnd();
        std::cout<<"light "<<lightBar_time.getTimeMs()<<std::endl;
//        cv::imshow("thresh", thresh);
//        static int i = 1;
//        if(i < 1000){
//            i++;
//        }
//        else{
//            cv::imwrite("../thresh.jpg", thresh);
//        }
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //查找图像轮廓

        std::vector<cv::RotatedRect> possibleLightBars(contours.size());

        // 清空lightBar_和temp_LBQ
        lightBars_.clear();
        while(!temp_LBQ_.empty())
        {
            temp_LBQ_.pop();
        }

        if(contours.size()<2 && useROI<=1)                                  // 没有识别到灯条的情况，返回一个空的lightBarNode
        {
            return lightBarNode();
        }

        for (unsigned int j = 0; j < contours.size(); ++j)                  // 遍历每一个轮廓，找到最小矩形框
        {
            lightBarNode node;
            cv::Point2f point[4];
            possibleLightBars.at(j) = cv::minAreaRect(contours[j]);

            //------------test----------------
            bool is_get_angle = false;
            cv::RotatedRect tmp_Elli;
            if(contours[j].size()>6)                                        // fitEllipse要求点集最少有6个点
            {
                tmp_Elli=cv::fitEllipse(contours[j]);                       // 返回拟合后的椭圆
//                | -90
//                | 0
//                | 90
                if(tmp_Elli.angle<90)
                {
                    tmp_Elli.angle = -tmp_Elli.angle;
                }
                else
                {
                    tmp_Elli.angle = -tmp_Elli.angle + 180;
                }
                is_get_angle = true;
            }

            if(possibleLightBars.at(j).size.height > possibleLightBars.at(j).size.width)
            {
                if(is_get_angle)
                {
                    node.angle = tmp_Elli.angle;
                }
                else
                {
                    node.angle = -possibleLightBars.at(j).angle;
                }
                if(!NOLightBar)
                {
                    node.length = possibleLightBars.at(j).size.height;
                    node.width = possibleLightBars.at(j).size.width;
                }
                else
                {
                    node.length = possibleLightBars.at(j).size.height*pryRateX;
                    node.width = possibleLightBars.at(j).size.width*pryRateY;
                }
                possibleLightBars.at(j).points(point);
                if(!NOLightBar)
                {
                    node.point[0]=point[0] + ROI_Pt_;
                    node.point[1]=point[3] + ROI_Pt_;//1->3
                    node.point[2]=point[2] + ROI_Pt_;
                    node.point[3]=point[1] + ROI_Pt_;//3->1
                }
                else
                {
                    node.point[0].x=(point[0].x + ROI_Pt_.x)*pryRateX;
                    node.point[1].x=(point[3].x + ROI_Pt_.x)*pryRateX;
                    node.point[2].x=(point[2].x + ROI_Pt_.x)*pryRateX;
                    node.point[3].x=(point[1].x + ROI_Pt_.x)*pryRateX;
                    node.point[0].y=(point[0].y + ROI_Pt_.y)*pryRateY;
                    node.point[1].y=(point[3].y + ROI_Pt_.y)*pryRateY;
                    node.point[2].y=(point[2].y + ROI_Pt_.y)*pryRateY;
                    node.point[3].y=(point[1].y + ROI_Pt_.y)*pryRateY;
                }
            }
            else
            {
                if(is_get_angle)
                {
                    node.angle = tmp_Elli.angle;
                }
                else
                {
                    node.angle = -possibleLightBars.at(j).angle-90;
                }
                if(!NOLightBar)
                {
                    node.length = possibleLightBars.at(j).size.width;
                    node.width = possibleLightBars.at(j).size.height;
                }
                else
                {
                    node.length = possibleLightBars.at(j).size.width*sqrt(pryRateX*pryRateX+pryRateY*pryRateY);
                    node.width = possibleLightBars.at(j).size.height*sqrt(pryRateX*pryRateX+pryRateY*pryRateY);
                }
                possibleLightBars.at(j).points(node.point);
                for(int k=0;k<4;k++)
                {
                    if(!NOLightBar)
                        node.point[k]=node.point[k]+ROI_Pt_;
                    else
                    {
                        node.point[k].x=(node.point[k].x+ROI_Pt_.x)*pryRateX;
                        node.point[k].y=(node.point[k].y+ROI_Pt_.y)*pryRateY;
                    }
                }
            }
            if(!NOLightBar)
            {
                possibleLightBars.at(j).center += ROI_Pt_;
            }
            else
            {
                possibleLightBars.at(j).center.x = (possibleLightBars.at(j).center.x+ROI_Pt_.x)*pryRateX;
                possibleLightBars.at(j).center.y = (possibleLightBars.at(j).center.y+ROI_Pt_.y)*pryRateY;
            }

            //计算lightbar到图像中心的距离和lightbar的面积
            node.distance = std::sqrt((possibleLightBars.at(j).center.x-rawPic.mat.rows/2)*(possibleLightBars.at(j).center.x-rawPic.mat.rows/2)
                                      + (possibleLightBars.at(j).center.y-rawPic.mat.cols/2)*(possibleLightBars.at(j).center.y-rawPic.mat.cols/2));
            node.area = possibleLightBars.at(j).size.area();

            //计算长宽比
            if(node.width == 0)
            {
                node.lengthWidthRatio = 0;
            }
            else
            {
                node.lengthWidthRatio = node.length/node.width;
            }
            node.center = possibleLightBars.at(j).center;
            node.angle_Ceo = (float)angle_Ceo_->getScore(node.angle);
            node.area_Ceo = (float)area_Ceo_->getScore(node.area);
            node.LWR_Ceo = (float)LWR_Ceo_->getScore(node.lengthWidthRatio);
            node.receiveData_ = rawPic.receiveData_;
            node.distance_Ceo = (float)distance_Ceo_->getScore((double)node.distance);

            node.sum = node.angle_Ceo + node.LWR_Ceo + node.area_Ceo + node.distance_Ceo;       //计算node的分数
            if(node.disable == 0)
            {
                temp_LBQ_.push(node);
            }
        }
        if(NOLightBar) NOLightBar=false;
        return temp_LBQ_.empty() ? lightBarNode() : temp_LBQ_.top();
    }

    cv::Mat lightBar::pryDownNoG(cv::Mat rawPic,float rateX,float rateY)
    {
        cv::Mat downPic=cv::Mat::zeros(floor(rawPic.rows/rateY),floor(rawPic.cols/rateX),CV_8UC3);
        for(int i=0;i<downPic.rows;i++)
            for(int j=0;j<downPic.cols;j++)
            {
                downPic.at<cv::Vec3b>(i,j)=rawPic.at<cv::Vec3b>(i*rateY+floor((rateY-1)/2),j*rateX+floor((rateX-1)/2));
            }
        return downPic;
    }
}


