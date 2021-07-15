#include "cameraThread.h"

namespace ly{

    /**
     * @brief 根据配置文件初始化不同的类
     * @param config 设备配置参数
     */
    cameraThread::cameraThread(const cam_device& config, cam_param camera)
    {
        device = config;
        cameraParam = camera;

        type_ = config.deviceType;
        std::cout<<type_<<std::endl;

        switch (type_)
        {
            case HIK_CAM:
//                cam_ = new HIK_camera();
//                start(1000);//1KHZ
                break;
            case DAH_CAM:
//                cam_ = new DAH_Camera();
                cam_new = new GxCamera();
                start(5000);
                break;
            case PIC:
                cam_ = new picture(config.picPath);
                start(10000);//100帧相机
                break;
            case VIDEO:
                cam_ = new video(config.videoPath);
                start(10000);//100帧相机
                break;
            default:
                break;
        }
    }

    /**
     * @brief 多线程获取相机信息
     */
    void cameraThread::process()
    {
//        if(!isOpen){
//            isOpen = true;
//        }
//        else{
//            return;
//        }
        //init camrea lib
        cam_new->initLib();
        //   open device      SN号
        cam_new->openDevice("KE0200060393");
        //Attention:   (Width-64)%2=0; (Height-64)%2=0; X%16=0; Y%2=0;
        int64_t width = 1280;
        int64_t height = 1024;
        int64_t OffsetX = 0;
        int64_t OffsetY = 0;
        //   ROI             Width  Height        X       Y
        cam_new->setRoiParam(width, height, OffsetX, OffsetY);
        //   ExposureGain          autoExposure  autoGain  ExposureTime  AutoExposureMin  AutoExposureMax  Gain(<=16)  AutoGainMin  AutoGainMax  GrayValue
        cam_new->setExposureGainParam( false, false, 5000, 1000, 3000, 3, 5, 16, 127);
        //   WhiteBalance             Applied?       light source type
        cam_new->setWhiteBalanceParam(    false,    GX_AWB_LAMP_HOUSE_ADAPTIVE);
        //   Acquisition Start!
        cam_new->acquisitionStart(&frame_);

         //复制两次，分别用于灯条识别和装甲id识别
         //若测试图片或视频要下面的部分
//        Mat temp = cam_->getFrame();
//        mutex_pic_.lock();
//        frame_.mat = temp.mat.clone();
//        mutex_pic_.unlock();
//        mutex_pic2_.lock();
//        frame2_.mat = frame_.mat.clone();
//        mutex_pic2_.unlock();
//
//        mutex_pic_.lock();
//        frame_ = val_out;
//        mutex_pic_.unlock();
//        mutex_pic2_.lock();
//        frame2_ = frame_;
//        mutex_pic2_.unlock();
    }
    /**
     * @brief 返回图片信息，相机和非相机两种情况
     * @return 图片信息
     */
    Mat cameraThread::getFrame()
    {
        Mat temp1;
        mutex_pic_.lock();
        temp1.mat = frame_.mat.clone();
        temp1.time = frame_.time;
        mutex_pic_.unlock();
        return temp1;
    }

    Mat cameraThread::getFrame2()
    {
        Mat temp2;
        mutex_pic2_.lock();
        temp2.mat = frame2_.mat.clone();
        temp2.time = frame2_.time;
        mutex_pic2_.unlock();
        return temp2;
    }
}