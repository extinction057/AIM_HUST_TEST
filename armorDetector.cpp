#include "armorDetector.h"
#include<cmath>

namespace ly
{
    armorDetector::armorDetector()
    {
        config_ = new config();
        camera_ = new cameraThread(config_->getConfigData().camDevice, config_->getConfigData().camParam);
        serialPortRead_ = new serialPortReadThread(config_->getConfigData().serialPortDev);
        serialPortWrite_ = new serialPortWriteThread(config_->getConfigData().serialPortDev);
        preProcess_ = new preProcessThread(config_->getConfigData().lightBarParam,camera_, serialPortRead_, serialPortWrite_,config_->getConfigData().enemyType);
        light_ = new lightBarThread(config_->getConfigData().camParam,config_->getConfigData().lightBarParam,camera_,preProcess_,serialPortRead_,serialPortWrite_,config_->getConfigData().enemyType);
        armor_ = new armorThread(config_->getConfigData().armorParam, config_->getConfigData().largeArmorParam, config_->getConfigData().camParam, light_, serialPortRead_, serialPortWrite_, camera_);
#if testSerialPort
        serialPortRead_ = new serialPortReadThread(config_->getConfigData().serialPortDev);
        serialPortWrite_ = new serialPortWriteThread(config_->getConfigData().serialPortDev);
        int i = 0;
        while(1){
            receiveData re_ = serialPortRead_->getReceiveMsg();
//            std::cout <<  "p: " <<  re_.pitch <<
//                      "y: " <<  re_.yaw   <<
//                      std::endl;
            sendData se_;
            se_.yaw = i;
            se_.pitch = i-2000;
            se_.distance = 0;
//            std::cout <<  "p: " <<  se_.pitch <<
//                      "y: " <<  se_.yaw   <<
//                                          std::endl;
            i = i>10000?0:i+1;
            serialPortWrite_->setSendMsg(se_);
            usleep(1);
        }
#endif

    }
    armorDetector::~armorDetector()
    {
        delete config_;
        delete armor_;
        delete camera_;
        delete light_;
        delete preProcess_;
//	    delete serialPortRead_;
//        delete serialPortWrite_;
	    //delete transform_;
    }
}
