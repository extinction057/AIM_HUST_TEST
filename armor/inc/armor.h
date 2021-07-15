#ifndef __ARMOR_
#define __ARMOR_

#include <opencv2/opencv.hpp>
#include "lightBar.h"
#include "score.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

#define OUTPUT_FEATURE 0

namespace ly
{
    typedef struct armorNode
    {
        /**
         * 评价装甲的优先级的指标
         * ---面积，在一定范围内面积越大得分越高，过大则为零
         * ---平行度，两个灯条的平行度，角度相减为零为1
         * ---离光心的马氏距离，从0到最大值，得分从1到0线性分布
         * ---装甲的角度，从0到60，得分从1到0线性分布
         * ---长宽比
         */
        float lengthWidthRatio;//长宽比
        float lightBarRatio;
        float angle;//角度
        float area;//面积
        float distance;//离光心的像素平面上的欧式距离
        float parallelism;//平行度
        float horizontalParallelism;

        float LWR_ceo;
        float LBR_ceo;
        float angle_ceo;
        float area_ceo;
        float distance_ceo;
        float parallelism_ceo;
        float lightBar_ceo;
        float lastCenter_ceo;
        float hp_ceo;

        float large_LWR_ceo;
        float large_LBR_ceo;
        float large_angle_ceo;
        float large_area_ceo;
        float large_parallelism_ceo;
        float large_lastCenter_ceo;
        float large_hp_ceo;

        float length;
        float width;
        float deltaCenter;
        int disable = 0;
        cv::Point2f corner[4];      //armor的四个角，0-3分别为左上，右上，右下，左下
        cv::Point2f center;
        float sum;
        float sum_large;
        bool isLarge;
        clock_t time;
        receiveData receiveData_;
        friend bool operator < (armorNode a, armorNode b)
        {
            return a.sum < b.sum;
        }
    }armorNode;

    class armor
    {
    public:
        armor() = default;
        armor(armor_param config, armor_param large);
        ~armor();
        void init(armor_param config, armor_param large);
        void matchLightBar(std::priority_queue<lightBarNode> lightBars);
        std::priority_queue<armorNode> getArmor(){ return armor_;}
        int verifyArmorLight(cv::Mat armor_pic,Eigen::Matrix2d A,Eigen::Vector2d t);

    protected:
        void judgeArmor(lightBarNode lightBar_1,lightBarNode lightBar_2);

        std::priority_queue<armorNode> armor_;
        armorNode bestArmor_;
        cv::Point2f last_center;

        armor_param config_;
        armor_param large_;

        score *LWR_ceo_;
        score *LBR_ceo_;
        score *angle_ceo_;
        score *area_ceo_;
        score *parallelism_ceo_;
        score *lastCenter_ceo_;
        score *horizontalParallelism_ceo_;

        score *large_LWR_ceo_;
        score *large_LBR_ceo_;
        score *large_angle_ceo_;
        score *large_area_ceo_;
        score *large_parallelism_ceo_;
        score *large_lastCenter_ceo_;
        score *large_horizontalParallelism_ceo_;

        std::string path = "../data/armor_feature.txt";
    };
}

#endif //__ARMOR_
