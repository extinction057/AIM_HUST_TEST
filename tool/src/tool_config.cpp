/**
 * @file tool_config.cpp
 * @brief use json file to get config data, https://www.cnblogs.com/fnlingnzb-learner/p/6743367.html
 * @author lyc
 */
#include "tool_config.h"
#include <fstream>
#include <iostream>
namespace ly{
    config::config()
    {//read all config data
        std::fstream file(path_,std::ios::in);
        if(!file.is_open())
        {
            std::cerr<<"can not open "<<"path:"<<path_<<std::endl;
        }
        JSONCPP_STRING errs;
        bool status = Json::parseFromStream(builder, file, &root, &errs);
        if(!status){
            std::cerr<<"parse error !!!"<<std::endl;
        }
        else {
            /***********cam_device************/
            configDataSt_.enemyType = root["enemy_type"].asInt();
            /***********cam_device************/
            configDataSt_.camDevice.deviceType = root["cam_device"]["deviceType"].asInt();
            configDataSt_.camDevice.sn = root["cam_device"]["sn"].asString();
            configDataSt_.camDevice.videoPath = root["cam_device"]["videoPath"].asString();
            configDataSt_.camDevice.picPath = root["cam_device"]["picturePath"].asString();
            configDataSt_.camDevice.picNum = root["cam_device"]["picNum"].asInt();
            /***********serialPort_dev************/
            configDataSt_.serialPortDev.enable = root["serialPort_dev"]["enable"].asBool();
            configDataSt_.serialPortDev.deviceName = root["serialPort_dev"]["deviceName"].asString();
            /***********cam_param************/
            configDataSt_.camParam.fx = root["cam_param"]["fx"].asDouble();
            configDataSt_.camParam.fy = root["cam_param"]["fy"].asDouble();
            configDataSt_.camParam.u0 = root["cam_param"]["u0"].asDouble();
            configDataSt_.camParam.v0 = root["cam_param"]["v0"].asDouble();
            configDataSt_.camParam.k_1 = root["cam_param"]["k1"].asDouble();
            configDataSt_.camParam.k_2 = root["cam_param"]["k2"].asDouble();
            configDataSt_.camParam.k_3 = root["cam_param"]["k3"].asDouble();
            configDataSt_.camParam.p_1 = root["cam_param"]["p1"].asDouble();
            configDataSt_.camParam.p_2 = root["cam_param"]["p2"].asDouble();
            configDataSt_.camParam.ExposureTime = root["cam_param"]["ExposureTime"].asDouble();
            configDataSt_.camParam.Gain = root["cam_param"]["Gain"].asDouble();
            /***********robot_param************/
            configDataSt_.robotParam.gimbal_cam[0] = root["robot_param"]["gimbal_cam"]["dx"].asDouble();
            configDataSt_.robotParam.gimbal_cam[1] = root["robot_param"]["gimbal_cam"]["dy"].asDouble();
            configDataSt_.robotParam.gimbal_cam[2] = root["robot_param"]["gimbal_cam"]["dz"].asDouble();
            /***********lightBar_param************/
            configDataSt_.lightBarParam.thresh = root["lightBar_param"]["thresh"].asInt();
            configDataSt_.lightBarParam.gamma = root["lightBar_param"]["gamma"].asFloat();
            configDataSt_.lightBarParam.color = root["lightBar_param"]["color"].asString();
            configDataSt_.lightBarParam.lengthWidthRadio = getParam("lightBar_param", "lengthWidthRadio");
            configDataSt_.lightBarParam.angle = getParam("lightBar_param", "angle");
            configDataSt_.lightBarParam.area = getParam("lightBar_param", "area");
            configDataSt_.lightBarParam.distance = getParam("lightBar_param", "distance");
            /***********armor_param************/
            configDataSt_.armorParam.lengthWidthRadio = getParam("armor_param", "lengthWidthRadio");
            configDataSt_.armorParam.lightBarRadio = getParam("armor_param", "lightBarRadio");
            configDataSt_.armorParam.angle = getParam("armor_param", "angle");
            configDataSt_.armorParam.area = getParam("armor_param", "area");
            configDataSt_.armorParam.parallelism = getParam("armor_param", "parallelism");
            configDataSt_.armorParam.lastCenter = getParam("armor_param", "lastCenter");
            configDataSt_.armorParam.hParallelism = getParam("armor_param", "hParallelism");
            configDataSt_.armorParam.LWR_w = root["armor_param"]["LWR_w"].asDouble();
            configDataSt_.armorParam.angle_w = root["armor_param"]["angle_w"].asDouble();
            configDataSt_.armorParam.area_w = root["armor_param"]["area_w"].asDouble();
            configDataSt_.armorParam.distance_w = root["armor_param"]["distance_w"].asDouble();
            configDataSt_.armorParam.parallelism_w = root["armor_param"]["parallelism_w"].asDouble();
            configDataSt_.armorParam.LBR_w = root["armor_param"]["LBR_w"].asDouble();
            configDataSt_.armorParam.lastCenter_w = root["armor_param"]["lastCenter_w"].asDouble();
            configDataSt_.armorParam.hp_w = root["armor_param"]["hp_w"].asDouble();
            /***********large_armor_param******/
            configDataSt_.largeArmorParam.lengthWidthRadio = getParam("large_armor_param", "lengthWidthRadio");
            configDataSt_.largeArmorParam.lightBarRadio = getParam("large_armor_param", "lightBarRadio");
            configDataSt_.largeArmorParam.angle = getParam("large_armor_param", "angle");
            configDataSt_.largeArmorParam.area = getParam("large_armor_param", "area");
            configDataSt_.largeArmorParam.parallelism = getParam("large_armor_param", "parallelism");
            configDataSt_.largeArmorParam.lastCenter = getParam("large_armor_param", "lastCenter");
            configDataSt_.largeArmorParam.hParallelism = getParam("large_armor_param", "hParallelism");
            configDataSt_.largeArmorParam.LWR_w = root["large_armor_param"]["LWR_w"].asDouble();
            configDataSt_.largeArmorParam.angle_w = root["large_armor_param"]["angle_w"].asDouble();
            configDataSt_.largeArmorParam.area_w = root["large_armor_param"]["area_w"].asDouble();
            configDataSt_.largeArmorParam.distance_w = root["large_armor_param"]["distance_w"].asDouble();
            configDataSt_.largeArmorParam.parallelism_w = root["large_armor_param"]["parallelism_w"].asDouble();
            configDataSt_.largeArmorParam.LBR_w = root["large_armor_param"]["LBR_w"].asDouble();
            configDataSt_.largeArmorParam.lastCenter_w = root["large_armor_param"]["lastCenter_w"].asDouble();
            configDataSt_.largeArmorParam.hp_w = root["large_armor_param"]["hp_w"].asDouble();
//            /***********weights****************/
//            configDataSt_.weightsParam.LWR_ceo = root["weights_param"]["LWR_ceo"].asDouble();
//            configDataSt_.weightsParam.angle_ceo = root["weights_param"]["angle_ceo"].asDouble();
//            configDataSt_.weightsParam.area_ceo = root["weights_param"]["area_ceo"].asDouble();
//            configDataSt_.weightsParam.distance_ceo = root["weights_param"]["distance_ceo"].asDouble();
//            configDataSt_.weightsParam.parallelism_ceo = root["weights_param"]["parallelism_ceo"].asDouble();
//            configDataSt_.weightsParam.lightBar_ceo = root["weights_param"]["lightBar_ceo"].asDouble();
//            configDataSt_.weightsParam.LBR_ceo = root["weights_param"]["LBR_ceo"].asDouble();
//            configDataSt_.weightsParam.lastCenter_ceo = root["weights_param"]["lastCenter_ceo"].asDouble();
        }
        file.close();
    }
    param config::getParam(const std::string& paramName1,const std::string& paramName2)
    {
        param temp;
        temp.enable = root[paramName1][paramName2]["enable"].asBool();;
        temp.best = root[paramName1][paramName2]["best"].asDouble();;
        temp.min = root[paramName1][paramName2]["min"].asDouble();;
        temp.max = root[paramName1][paramName2]["max"].asDouble();;
        temp.left_ceo = root[paramName1][paramName2]["left_ceo"].asDouble();;
        temp.right_ceo = root[paramName1][paramName2]["right_ceo"].asDouble();;
        return temp;
    }
}








