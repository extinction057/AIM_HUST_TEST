#ifndef __BUFF_
#define __BUFF_

#include "tools.h"
#include "camera.h"
#include "armor.h"
#include "lightBar.h"
#include "solvePNP.h"
#include "transform.h"
//#include "preDetector.h"
#include "RM_BUFF.h"
#include "serialPort.h"
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"


namespace ly
{
/*
 * 能量机关
 * */
    class BUFF_Detector
    {
    public:
        BUFF_Detector(std::string cfg_Path,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,cam_param cam_config);
        ~BUFF_Detector();
        bool process(Mat src,cv::Point3f &est_Point);

    private:
        RM_BUFF_Dec *new_Buff_Detector;
        serialPort *communicator;
        cv::VideoWriter cap;
        double ave_Fps,c_Cnt;
        receiveData rec_Data;
        bool is_Record,is_Find_mode;
        int rec_Num,wait_Num,find_Num;
        cv::Mat old_Img;
        serialPortReadThread *serialPortRead_;
        serialPortWriteThread *serialPortWrite_;
    };
}

#endif //__BUFF_
