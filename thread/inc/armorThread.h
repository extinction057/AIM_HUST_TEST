#ifndef __ARMOR_THREAD_H
#define __ARMOR_THREAD_H
#include "solvePNP.h"
#include "armor.h"
#include "thread.h"
#include "lightBarThread.h"
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"
#include <mutex>
#include <thread>

#define PREDICT 0
#define VISUALIZE 0
#define TIMEIT 0

#define constraint(a, min, max) \
    fmin(fmax(a, min), max)

namespace ly{
class armorThread:public armor,public thread
{
public:
    armorThread() = default;
    explicit armorThread(armor_param config, armor_param large,cam_param cam_config,lightBarThread *lightBar,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,cameraThread *camera);
    ~armorThread();

private:

    void process() override;
    bool verifyArmor(cv::Mat pic,std::priority_queue<armorNode> armor);
    cv::Mat getArmorRoi(armorNode);

    lightBarThread *lightBar_;
    serialPortReadThread *serialPortRead_;
    serialPortWriteThread *serialPortWrite_;
    solvePNP *pnp_;
    cameraThread * camera_;
    score* score_;

    bool debug_ = true;
    bool update_ = false;
    receiveData shootType_;

    armorNode best_armor_;
    time counter_;

    std::string path = "../data/pic_blue/tmp/img_";

    bool isLoad = false;
    cv::Ptr<cv::ml::SVM> model;

    cv::Size roiSize = cv::Size(32, 32);
};
}




#endif //__ARMOR_THREAD_H
