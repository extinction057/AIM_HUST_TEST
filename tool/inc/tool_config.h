#ifndef __TOOL_CONFIG_H
#define __TOOL_CONFIG_H

#include "jsoncpp/json/json.h"
#include <string>
#include <memory>
namespace ly{
    typedef struct cam_device
    {
        int deviceType;
        std::string sn;
        std::string videoPath;
        std::string picPath;
        int picNum;
    }cam_device;
    typedef struct serialPort_dev
    {
        bool enable;
        std::string deviceName;
    }serialPort_dev;
    typedef struct cam_param
    {
        double fx;
        double fy;
        double u0;
        double v0;
        double k_1;
        double k_2;
        double p_1;
        double p_2;
        double k_3;
        double ExposureTime;
        double Gain;
    }cam_param;

    typedef struct robot_param
    {
        double gimbal_cam[3];
    }robot_param;
    typedef struct param
    {
        bool enable;
        double best;
        double min;
        double max;
        double left_ceo;
        double right_ceo;
    }param;
    typedef struct lightBar_param
    {
        int thresh;
        float gamma;
        std::string color;
        param lengthWidthRadio;
        param angle;
        param area;
        param distance;
    }lightBar_param;

    typedef struct armor_param
    {
        param lengthWidthRadio;
        param lightBarRadio;
        param angle;
        param area;
        param parallelism;
        param lastCenter;
        param hParallelism;
        double LWR_w;
        double angle_w;
        double area_w;
        double distance_w;
        double parallelism_w;
        double LBR_w;
        double lastCenter_w;
        double hp_w;
    }armor_param;

//    typedef struct weights_param
//    {
//        double LWR_ceo;
//        double angle_ceo;
//        double area_ceo;
//        double distance_ceo;
//        double parallelism_ceo;
//        double lightBar_ceo;
//        double LBR_ceo;
//        double lastCenter_ceo;
//    } weights_param;

    typedef struct configData
    {
        cam_device camDevice;
        serialPort_dev serialPortDev;
        cam_param camParam;
        robot_param robotParam;
        lightBar_param lightBarParam;
        armor_param armorParam;
        armor_param largeArmorParam;
        int enemyType;
    }configData;

    class config
    {
    public:
        config();
        param getParam(const std::string& paramName1,const std::string& paramName2);
        configData getConfigData(){ return configDataSt_;}
    private:
        configData configDataSt_;
        Json::CharReaderBuilder builder;
        Json::Value root;
        std::string path_ = "../config/armorDetector.json";
    };
}
#endif //__TOOL_CONFIG_H