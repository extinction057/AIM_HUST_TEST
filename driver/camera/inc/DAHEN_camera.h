#ifndef __DAHENG_CAMERA_H
#define __DAHENG_CAMERA_H

#include "camera.h"
#include"DxImageProc.h"
#include"GxIAPI.h"

namespace ly{
class DAH_Camera:public camera
{
public:
    DAH_Camera();
    ~DAH_Camera();
    int Init();
    int SetPixelFormat(int64_t pixelformat = GX_PIXEL_FORMAT_BAYER_BG10);
    int SetResolution(unsigned int width, unsigned int height);
    int SetTriggerMode(unsigned int mode);
    int SetFPS(double fps);
    int SetExposureTime(float time);
    int SetGain();
    int SetBalanceWhite(double ratioBlue,double ratioRed,double ratioGreen);
    int SetDeviceName(char *name);
    int StartStream();
    int operator >>(cv::Mat &val_out);
    Mat getFrame() ;
    int getType() { return DAH_CAM;}
    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
    void SetGamma(double gamma_);
private:
    char m_DeviceID[128];
    GX_DEV_HANDLE m_CamHandle = NULL;
//    int64_t g_nPayloadSize = 2621440;
    static unsigned char* g_pRaw8Image ;           ///< Memory for RAW8toRGB24
    static unsigned char* g_pRGBImageBuf;     ///< Payload size
    int m_ImageWidth;
    int m_ImageHeight;
    double m_fps;                       //OK
    int m_TriggerMode;          //OK
    int64_t m_PixelFormat;//OK  相机仅支持像素格式为BayerRG8和BayerRG10
    float m_ExpTime;   //OK
    float m_Gain;               //OK
    double m_Gamma;         //OK   相机不支持gamma的设置
    int m_GrabCnt;
    double BalanceWhite_Blue;
    double BalanceWhite_Red;
    double BalanceWhite_Green;

};


#endif // __DAHENG_CAMERA_H
}
