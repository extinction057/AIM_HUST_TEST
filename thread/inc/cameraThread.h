#ifndef __CAMERA_THREAD_H
#define __CAMERA_THREAD_H

#include "camera.h"
//#include "HIK_camera.h"
#include "DAHEN_camera.h"
#include <mutex>
#include <thread>
#include "tools.h"
#include "thread.h"
#include "GxCamera.h"
#include <cstring>


namespace ly{

    extern Mat val_out;

    class cameraThread:public camera,public thread
    {
    public:
        cameraThread() = default;
        explicit cameraThread(const cam_device& config, cam_param camera);
        Mat getFrame() override;
        Mat getFrame2() override;
        int getType() override{ return type_;}
        inline double getUpdateTime() override{ return cam_->getUpdateTime();}
        inline int getFrameId() override{ return cam_->getFrameId();}
        double timeSum;
        std::string saveImagePath = "../data/pic";
        int imageId;
    private:
        int type_;
        bool isOpen = false;
        camera * cam_{};
        GxCamera* cam_new{};
        cam_device device;
        cam_param cameraParam;
        void process();
        time counter_;
    };
}


#endif //__CAMERA_THREAD_H