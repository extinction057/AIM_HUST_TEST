#include "armor.h"

namespace ly
{
    armor::armor(armor_param config, armor_param large)
    {
//        std::ofstream fileWriter(path, std::ios::out);
//        fileWriter.close();
        init(config, large);
    }
    void armor::init(armor_param config, armor_param large){
        config_ = std::move(config);
        large_ = std::move(large);

        LWR_ceo_ = new score(config.lengthWidthRadio);
        LBR_ceo_ = new score(config.lightBarRadio);
        angle_ceo_ = new score(config.angle);
        area_ceo_ = new score(config.area);
        parallelism_ceo_ = new score(config.parallelism);
        lastCenter_ceo_ = new score(config.lastCenter);
        horizontalParallelism_ceo_ = new score(config.hParallelism);

        large_LWR_ceo_ = new score(large.lengthWidthRadio);
        large_LBR_ceo_ = new score(large.lightBarRadio);
        large_angle_ceo_ = new score(large.angle);
        large_area_ceo_ = new score(large.area);
        large_parallelism_ceo_ = new score(large.parallelism);
        large_lastCenter_ceo_ = new score(large.lastCenter);
        large_horizontalParallelism_ceo_ = new score(large.hParallelism);
    }
    armor::~armor()
    {
        delete(LWR_ceo_);
        delete(LBR_ceo_);
        delete(angle_ceo_);
        delete(area_ceo_);
        delete(parallelism_ceo_);
        delete(lastCenter_ceo_);

        delete(large_LWR_ceo_);
        delete(large_LBR_ceo_);
        delete(large_angle_ceo_);
        delete(large_area_ceo_);
        delete(large_parallelism_ceo_);
        delete(large_lastCenter_ceo_);
    }
    void armor::matchLightBar(std::priority_queue<lightBarNode> lightBars)
    {

        if(lightBars.empty() || lightBars.top().sum<1.5){                 //没有检测到lightBar，或者效果太差
            //std::cout << "no lightBar" << std::endl;
            return ;
        }

        while(!armor_.empty())  armor_.pop();                           //清空armor_，用lightBar检测新的armor_

        int size1 = fmin(4, lightBars.size());
        while(size1>=0 && !lightBars.empty()){
            lightBarNode lightBar1 = lightBars.top();
            lightBars.pop();
            std::priority_queue<lightBarNode> lightBars_bak(lightBars);

            int size2 = fmin(3, lightBars.size());
            while(size2>=0 && !lightBars_bak.empty()){
                lightBarNode lightBar2 = lightBars_bak.top();
                lightBars_bak.pop();
                judgeArmor(lightBar1, lightBar2);
                size2--;
            }

            size1--;
        }

        if(!armor_.empty())
        {
            last_center = armor_.top().center;
        }


    }
    /**
     * @brief 两个灯条节点是否能构成一个装甲
     * @param lightBar_1
     * @param lightBar_2
     */
    void armor::judgeArmor(lightBarNode lightBar_1, lightBarNode lightBar_2)
    {
        armorNode armor;
        armor.center = (lightBar_1.center+lightBar_2.center)/2;
        cv::Point2f point[4];
        point[0] = (lightBar_1.point[0]+lightBar_1.point[1])/2;
        point[1] = (lightBar_1.point[2]+lightBar_1.point[3])/2;
        point[2] = (lightBar_2.point[0]+lightBar_2.point[1])/2;
        point[3] = (lightBar_2.point[2]+lightBar_2.point[3])/2;
        if(point[0].x>point[2].x)
        {
            if(point[0].y<point[1].y)       //point[0]是右上角点,point[1]是右下角点
            {
                armor.corner[1] = point[0];
                armor.corner[2] = point[1];
            }
            else
            {
                armor.corner[1] = point[1];
                armor.corner[2] = point[0];
            }
            if(point[2].y<point[3].y)
            {
                armor.corner[0] = point[2];
                armor.corner[3] = point[3];
            }
            else
            {
                armor.corner[0] = point[3];
                armor.corner[3] = point[2];
            }
        }
        else
        {
            if(point[0].y<point[1].y)
            {
                armor.corner[0] = point[0];
                armor.corner[3] = point[1];
            }
            else
            {
                armor.corner[0] = point[1];
                armor.corner[3] = point[0];
            }
            if(point[2].y<point[3].y)
            {
                armor.corner[1] = point[2];
                armor.corner[2] = point[3];
            }
            else
            {
                armor.corner[1] = point[3];
                armor.corner[2] = point[2];
            }
        }

        armor.angle = (lightBar_1.angle+lightBar_2.angle)/2;
        armor.length = (lightBar_1.length+lightBar_2.length)/2;
        float x = (lightBar_1.center.x-lightBar_2.center.x)*(lightBar_1.center.x-lightBar_2.center.x);
        float y = (lightBar_1.center.y-lightBar_2.center.y)*(lightBar_1.center.y-lightBar_2.center.y);
        armor.width = sqrt(x+y);
        armor.lengthWidthRatio = armor.width/armor.length;
        armor.area = armor.length*armor.width;
        armor.receiveData_ = lightBar_1.receiveData_;
        armor.distance = armor.center.x+armor.center.y;
        armor.parallelism = fabs(lightBar_1.angle-lightBar_2.angle);
        armor.horizontalParallelism = (lightBar_1.center.y-lightBar_2.center.y)/armor.width;

        if(lightBar_1.length>lightBar_2.length)
        {
            armor.lightBarRatio = lightBar_2.length/lightBar_1.length;
        }
        else
        {
            armor.lightBarRatio = lightBar_1.length/lightBar_2.length;
        }

        armor.deltaCenter = sqrt((armor.center.x - last_center.x)*(armor.center.x - last_center.x)
                                 + (armor.center.y - last_center.y)*(armor.center.y - last_center.y));

        // 开始计算得分

        float lightBar_ceo1 = (lightBar_1.distance_Ceo + lightBar_1.area_Ceo + lightBar_1.LWR_Ceo + lightBar_1.angle_Ceo)/4;
        float lightBar_ceo2 = (lightBar_2.distance_Ceo + lightBar_2.area_Ceo + lightBar_2.LWR_Ceo + lightBar_2.angle_Ceo)/4;

        armor.lightBar_ceo = (lightBar_ceo1 + lightBar_ceo2)/2;
        armor.distance_ceo = (lightBar_1.distance_Ceo+lightBar_2.distance_Ceo)/2;

        armor.LBR_ceo = (float)(LBR_ceo_->getScore(armor.lightBarRatio)*config_.LBR_w);
        armor.LWR_ceo = (float)(LWR_ceo_->getScore(armor.lengthWidthRatio)*config_.LWR_w);
        armor.angle_ceo = (float)(angle_ceo_->getScore(armor.angle)*config_.angle_w);
        armor.area_ceo = (float)(area_ceo_->getScore(armor.area)*config_.area_w);
        armor.parallelism_ceo = (float)(parallelism_ceo_->getScore(armor.parallelism)*config_.parallelism_w);
        armor.lastCenter_ceo = (float)(lastCenter_ceo_->getScore(armor.deltaCenter)*config_.lastCenter_w);
        armor.hp_ceo = (float)(horizontalParallelism_ceo_->getScore(armor.horizontalParallelism)*config_.hp_w);

        armor.sum = armor.LWR_ceo + armor.angle_ceo + armor.area_ceo
                    + armor.distance_ceo + armor.parallelism_ceo + armor.lightBar_ceo
                    + armor.LBR_ceo + armor.lastCenter_ceo + armor.hp_ceo;

        armor.large_LBR_ceo = (float)(large_LBR_ceo_->getScore(armor.lightBarRatio)*large_.LBR_w);
        armor.large_LWR_ceo = (float)(large_LWR_ceo_->getScore(armor.lengthWidthRatio)*large_.LWR_w);
        armor.large_angle_ceo = (float)(large_angle_ceo_->getScore(armor.angle)*large_.angle_w);
        armor.large_area_ceo = (float)(large_area_ceo_->getScore(armor.area)*large_.area_w);
        armor.large_parallelism_ceo = (float)(large_parallelism_ceo_->getScore(armor.parallelism)*large_.parallelism_w);
        armor.large_lastCenter_ceo = (float)(large_lastCenter_ceo_->getScore(armor.deltaCenter)*large_.lastCenter_w);
        armor.large_hp_ceo = (float)(large_horizontalParallelism_ceo_->getScore(armor.horizontalParallelism)*large_.hp_w);

        armor.sum_large = armor.large_LWR_ceo + armor.large_angle_ceo + armor.large_area_ceo
                    + armor.distance_ceo + armor.large_parallelism_ceo + armor.lightBar_ceo
                    + armor.large_LBR_ceo + armor.large_lastCenter_ceo + armor.large_hp_ceo;

//        std::cout << "sum: " << armor.sum << " sumL: " << armor.sum_large << std::endl;

        armor.isLarge = armor.sum <= armor.sum_large;

        if(armor.lengthWidthRatio > 6 || armor.lengthWidthRatio < 1.5 ||
            armor.area < 100 ||
            armor.angle < -20 || armor.angle > 20 ||
            armor.parallelism < -10 || armor.parallelism > 10 ||
            armor.horizontalParallelism > 0.35
            ){
            armor.disable = 1;
        }

        if(armor.disable == 0)
        {
            armor_.push(armor);
        }

#if OUTPUT_FEATURE
        if(armor_.empty()){
            std::cout << "empty armor queue!" << std::endl;
            return;
        }
        std::ofstream fileWriter(path, std::ios::app|std::ios::out);
        fileWriter << armor.lengthWidthRatio << " " << armor.lightBarRatio << " " <<
                   armor.angle << " " << armor.area << " " <<
                   armor.distance << " " << armor.parallelism << " " <<
                   armor.deltaCenter << " " <<
//                   ((armor_.top().sum > 4.5 && armor_.top().disable == 0) ? 1 : 0) << std::endl;
                    0 << std::endl;
        fileWriter.close();
#endif
        return;
    }
}

