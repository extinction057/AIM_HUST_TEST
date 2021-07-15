#include"GxIAPI.h"
#include"DxImageProc.h"
#include "DAHEN_camera.h"
#include <cstring>
#include <cstdio>
#include <unistd.h>


namespace ly{
    Mat val_out;
    time counter_;
    int getFramestatus =0;
    void *pGammaLut;
    DAH_Camera::DAH_Camera()
    {


        m_ImageWidth=1280;
        m_ImageHeight=1024;
        m_fps=220;                       //OK
        m_TriggerMode=0;          //OK
        m_PixelFormat=GX_PIXEL_FORMAT_BAYER_RG8;//OK
        m_ExpTime=4500.0;   //OK
        m_Gamma=1;         //OK
        m_Gain = 1;
        // 白 平 衡
        BalanceWhite_Blue= 1.1062;
        BalanceWhite_Red= 1.7305;
        BalanceWhite_Green= 1.0;


        Init();

    }


    DAH_Camera::~DAH_Camera()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        //发 送 停 采 命 令
        status = GXStreamOff(m_CamHandle);
        if(status!=GX_STATUS_SUCCESS){
            std::cout<<"GXStreamOff Failed!\n";
        }
        status = GXCloseDevice(m_CamHandle);
        if(status!=GX_STATUS_SUCCESS){
            std::cout<<"GXCloseDevice Failed!\n";
        }
        status = GXCloseLib();
        if(status!=GX_STATUS_SUCCESS){
            std::cout<<"GXCloseLib Failed!\n";
        }
        // delete m_pFrameBuffer;
    }

    unsigned char* DAH_Camera::g_pRaw8Image = new unsigned char[2621440];
    unsigned char* DAH_Camera::g_pRGBImageBuf = new unsigned char[2621440 * 3];

    int DAH_Camera::Init()
    {
        val_out.mat= cv::Mat::zeros(1024,1280,CV_8UC3);
        GX_STATUS status = GX_STATUS_SUCCESS;
        //获取库信息
        const char *pLibVersion = GXGetLibVersion();
        std::cout<<pLibVersion<<"\n";
        // delete []pLibVersion;
        //初 始 化 库
        //如果找不到TL库，是libgxiapi.so文件连接出错，CmakeLists里面改为/usr/lib/libgxiapi.so
        status = GXInitLib();
        if (status != GX_STATUS_SUCCESS)
        {
            std::cout<<"InitLib Failed!\n";
            return 0;
        }
        // 枚举设备
        // enum device
        uint32_t nDeviceNum = 0;
        status = GXUpdateDeviceList(&nDeviceNum, 1000);
        if (status == GX_STATUS_SUCCESS&&nDeviceNum> 0)
        {
            GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
            long unsigned int nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
            //获 取 所 有 设 备 的 基 础 信 息
            status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
            std::cout<<pBaseinfo->szVendorName<<"\n";
            std::cout<<pBaseinfo->szModelName<<"\n";
            std::cout <<pBaseinfo->szSN<<"\n";
            std::cout<<pBaseinfo->szDisplayName<<"\n";
            // delete []pBaseinfo;
        }

        //打 开 设 备
        status = GXOpenDeviceByIndex(1, &m_CamHandle);
        //status = GXSetAcqusitionBufferNumber(m_CamHandle,20);

        if (status == GX_STATUS_SUCCESS)
        {

            //设 置 采 集 模 式 。 一 般 相 机 的 默 认 采 集 模 式 为 连 续 模 式 。
            //int64_t nAcqMode = GX_ACQ_MODE_CONTINUOUS;
            //status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, nAcqMode);
            SetGamma(m_Gamma);
            //注 册 图 像 处 理 回 调 函 数
            status = GXRegisterCaptureCallback(m_CamHandle, NULL, OnFrameCallbackFun);
            //发 送 开 采 命 令
            status = GXSendCommand(m_CamHandle, GX_COMMAND_ACQUISITION_START);
            status = GXSendCommand(m_CamHandle, GX_COMMAND_TRIGGER_SOFTWARE);


        }
        else std::cout<<"OpenDevice Failed!\n";
        return (SetBalanceWhite(BalanceWhite_Blue,BalanceWhite_Red,BalanceWhite_Green)|SetExposureTime(m_ExpTime)|SetFPS(m_fps)|SetGain()|SetTriggerMode(m_TriggerMode));
        //SetPixelFormat(m_PixelFormat)|SetResolution(m_ImageWidth, m_ImageHeight)|
    }

    void GX_STDC DAH_Camera::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
    {
        getFramestatus =0;
        GX_STATUS status = GX_STATUS_SUCCESS;
        int m_GrabCnt=0;
        if (pFrame->status == 0)
        {
            //输出图像格式转换
            // status = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
            // RAW2RGB_NEIGHBOUR, BAYERRG, true);
            switch (pFrame->nPixelFormat)
            {
                case GX_PIXEL_FORMAT_BAYER_GR8:
                case GX_PIXEL_FORMAT_BAYER_RG8:
                case GX_PIXEL_FORMAT_BAYER_GB8:
                case GX_PIXEL_FORMAT_BAYER_BG8:
                {
                    // Convert to the RGB image
                    status = DxRaw8toRGB24((unsigned char*)pFrame->pImgBuf, g_pRGBImageBuf, pFrame->nWidth, pFrame->nHeight,
                                           RAW2RGB_NEIGHBOUR, BAYERRG, false);
                    if (status != DX_OK)
                    {
                        printf("DxRaw8toRGB24 Failed, Error Code: %d\n", status);
                    }
                    break;
                }
                case GX_PIXEL_FORMAT_BAYER_GR10:
                case GX_PIXEL_FORMAT_BAYER_RG10:
                case GX_PIXEL_FORMAT_BAYER_GB10:
                case GX_PIXEL_FORMAT_BAYER_BG10:
                case GX_PIXEL_FORMAT_BAYER_GR12:
                case GX_PIXEL_FORMAT_BAYER_RG12:
                case GX_PIXEL_FORMAT_BAYER_GB12:
                case GX_PIXEL_FORMAT_BAYER_BG12:
                {
                    // Convert to the Raw8 image
                    status = DxRaw16toRaw8((unsigned char*)pFrame->pImgBuf, g_pRaw8Image, pFrame->nWidth, pFrame->nHeight, DX_BIT_2_9);
                    if (status != DX_OK)
                    {
                        printf("DxRaw16toRaw8 Failed, Error Code: %d\n", status);

                    }
                    // Convert to the RGB24 image
                    status = DxRaw8toRGB24((unsigned char*)g_pRaw8Image, g_pRGBImageBuf, pFrame->nWidth, pFrame->nHeight,
                                           RAW2RGB_NEIGHBOUR, BAYERRG, false);
                    if (status != DX_OK)
                    {
                        printf("DxRaw8toRGB24 Failed, Error Code: %d\n", status);
                    }
                    break;
                }
                default:
                {
                    printf("Error : PixelFormat of this camera is not supported\n");
                }
            }
            status = DxImageImprovment(g_pRGBImageBuf,g_pRGBImageBuf,pFrame->nWidth,pFrame->nHeight,NULL, NULL, pGammaLut);
            cv::Mat temp= cv::Mat::zeros(1024,1280,CV_8UC3);
            memcpy(temp.data,g_pRGBImageBuf,pFrame->nWidth*pFrame->nHeight*3);
            cv::cvtColor(temp,val_out.mat,cv::COLOR_RGB2BGR);//这里不转为BGR会把红色识别为蓝色
            val_out.time=clock();
//            counter_.countEnd();
//            std::cout<<1000/counter_.getTimeMs()<<std::endl;
//            counter_.countBegin();
//            std::cout<<"time_DAHEN: "<<val_out.time<<std::endl;
            return;
        }
    }


    int DAH_Camera::SetPixelFormat(int64_t pixelformat)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        // //读 取 当 前 pixelformat
        // int64_t nPixelFormat = 0;
        // status = GXGetEnum(m_CamHandle, GX_ENUM_PIXEL_FORMAT, &nPixelFormat);
        // //设 置 pixelformat 为 bayer 格 式 为 BG 的 位 深 数 据
        // nPixelFormat = GX_PIXEL_FORMAT_BAYER_BG10;
        status = GXSetEnum(m_CamHandle, GX_ENUM_PIXEL_FORMAT, pixelformat);
        if (status != GX_STATUS_SUCCESS)
        {
            printf("SetPixelFormat fail! \n");
            return -1;
        }
        return 1;
    }

    int DAH_Camera::SetResolution(unsigned int width, unsigned int height)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        status = GXSetInt(m_CamHandle, GX_INT_WIDTH, width);
        status = GXSetInt(m_CamHandle, GX_INT_HEIGHT, height);
        if (status != GX_STATUS_SUCCESS)
        {
            printf("SetWidth|SetHeight fail!\n");
            return -1;
        }
        return 1;
    }

    int DAH_Camera::SetTriggerMode(unsigned int mode)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        // 设置触发模式
        // set trigger mode
        if(mode!=0){
            //设 置 触 发 模 式 为 ON
            status =GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
        //设 置 触 发 激 活 方 式 为 上 升 沿
            status = GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
            //设 置 触 发 激 活 方 式 为 外 部 触 发 模 式
            status =GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_SWITCH,GX_TRIGGER_SWITCH_ON);
            status =GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_SOURCE,GX_TRIGGER_SOURCE_LINE2);
//        status =GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_SELECTOR,GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
        }
        else
        {
            status =GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        //设 置 触 发 激 活 方 式 为 上 升 沿
            status = GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
            status =GXSetEnum(m_CamHandle,GX_ENUM_TRIGGER_SOURCE,GX_TRIGGER_SOURCE_SOFTWARE);
        }
        if (status != GX_STATUS_SUCCESS)
        {
            printf("SetTriggerMode fail! \n");
            return -1;
        }
        return 1;
    }

    int DAH_Camera::SetFPS(double fps)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        //使 能 采 集 帧 率 调 节 模 式
        status = GXSetEnum(m_CamHandle, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,GX_ACQUISITION_FRAME_RATE_MODE_ON);
        //设 置 采 集 帧 率,假 设 设 置 为 10.0, 用 户 按 照 实 际 需 求 设 置 此 值
        status = GXSetFloat(m_CamHandle, GX_FLOAT_ACQUISITION_FRAME_RATE,fps);
        if (status != GX_STATUS_SUCCESS)
        {
            printf("SetFrameRate fail! \n");
            return 0;
        }


        printf("FPS: %f\n",fps);


        return 1;
    }

    int DAH_Camera::SetExposureTime(float time)
    {

        GX_STATUS status = GX_STATUS_SUCCESS;



        //设置曝光模式为TIMED模式
        status = GXSetEnum(m_CamHandle, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
        //设 置 曝 光 延 迟  time (us)
        status = GXSetFloat(m_CamHandle, GX_FLOAT_EXPOSURE_TIME, time);
        if(status!=GX_STATUS_SUCCESS){
            std::cout<<"SetExposureTime Failed!\n";
            return -1;
        }

        return 1;
    }

    int DAH_Camera::SetGain()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        //选 择 增 益 通 道 类 型
        status =GXSetEnum(m_CamHandle, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        status = GXSetFloat(m_CamHandle, GX_FLOAT_GAIN, m_Gain);

//        status =GXSetEnum(m_CamHandle, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
//        status = GXSetFloat(m_CamHandle, GX_FLOAT_GAIN, Blue_Gain);
//
//        status =GXSetEnum(m_CamHandle, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
//        status = GXSetFloat(m_CamHandle, GX_FLOAT_GAIN, Green_Gain);


        if(status!=GX_STATUS_SUCCESS){
            std::cout<<"SetGain Failed!\n";
            return -1;
        }


        return 1;
    }


    int DAH_Camera::SetDeviceName(char *name)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        size_t nSize = 0;
        //首 先 获 取 字 符 串 允 许 的 最 大 长 度(此 长 度 包 含 结 束 符 ’\0’)
        status = GXGetStringMaxLength(m_CamHandle, GX_STRING_DEVICE_USERID, &nSize);
        //根 据 获 取 的 长 度 申 请 内 存
        char *pszText = new char[nSize];
        status = GXGetString(m_CamHandle, GX_STRING_DEVICE_USERID, pszText, &nSize);
        //设 置 用 户 自 定 义 名 称
        status = GXSetString(m_CamHandle, GX_STRING_DEVICE_USERID, name);
        // delete []pszText;

        if (status != GX_STATUS_SUCCESS)
        {
            printf("MV_CC_SetStringValue fail!\n");
            return -1;
        }
        return 1;
    }

    int DAH_Camera::StartStream()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;

        //开 采
        status = GXStreamOn(m_CamHandle);
        if (status != GX_STATUS_SUCCESS)
        {
            printf("StartGrabbing fail! \n");
            return -1;
        }
        return 1;
    }


    void DAH_Camera::SetGamma(double gamma_)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
////使 能 Gamma
//        status = GXSetBool(m_CamHandle, GX_BOOL_GAMMA_ENABLE, true);
////设 置 Gamma 模 式 为 用 户 自 定 义 模 式
//        GX_GAMMA_MODE_ENTRY nValue;
//        nValue = GX_GAMMA_SELECTOR_USER;
//        status = GXSetEnum(m_CamHandle, GX_ENUM_GAMMA_MODE, nValue);

        int nLutLength;
        //判 断 当 前 相 机 是 否 支 持 Gamma 获 取
        bool bIsImplemented = false;
        status = GXIsImplemented(m_CamHandle, GX_FLOAT_GAMMA_PARAM,
                                            &bIsImplemented);
        if (status!= GX_STATUS_SUCCESS)
        {
            return;
        }

        //获 取 Gamma 查 找 表 的 长 度
        VxInt32 DxStatus= DxGetGammatLut(gamma_, NULL, &nLutLength);
        if (DxStatus != DX_OK)
        {
            return;
        }
//为 Gamma 查 找 表 申 请 空 间
        pGammaLut = new unsigned char[nLutLength];
        if (pGammaLut== NULL)
        {
            DxStatus= DX_NOT_ENOUGH_SYSTEM_MEMORY;
            return;
        }
//计 算 Gamma 查 找 表
        DxStatus = DxGetGammatLut(gamma_, pGammaLut, &nLutLength);
//        if (DxStatus != DX_OK)
//        {
//            if (pGammaLut!= NULL)
//            {
//                delete []pGammaLut;
//                pGammaLut= NULL;
//            }
//            return;
//        }

    }

    int DAH_Camera::SetBalanceWhite(double ratioBlue,double ratioRed,double ratioGreen)
    {

        GX_STATUS status = GX_STATUS_SUCCESS;
        //选 择 白 平 衡 通 道(选择蓝色)
        status = GXSetEnum(m_CamHandle,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_BLUE);
        //设 置 白 平 衡 系 数
        status = GXSetFloat(m_CamHandle, GX_FLOAT_BALANCE_RATIO, ratioBlue);

        //选 择 白 平 衡 通 道(选择红色)
        status = GXSetEnum(m_CamHandle,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_RED);
        status = GXSetFloat(m_CamHandle, GX_FLOAT_BALANCE_RATIO, ratioRed);

        //选 择 白 平 衡 通 道(选择绿色)
        status = GXSetEnum(m_CamHandle,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_BLUE);
        status = GXSetFloat(m_CamHandle, GX_FLOAT_BALANCE_RATIO, ratioGreen);




        //设 置 手 动 白 平 衡
        status = GXSetEnum(m_CamHandle,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_OFF);
        if(status!=GX_STATUS_SUCCESS){
            std::cout<<"SetBalanceWhite Failed！\n";
            return -1;
        }
        return 1;
    }


    Mat DAH_Camera::getFrame(){}

}
