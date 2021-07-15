#ifndef __ARMOR_DETECTOR__
#define __ARMOR_DETECTOR__

#include "tools.h"
#include "camera.h"
#include "armor.h"
#include "lightBar.h"
#include "score.h"
#include "solvePNP.h"
#include "transform.h"
#include "armorThread.h"
#include "lightBarThread.h"
#include "preProcessThread.h"
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"
#include "cameraThread.h"

#include <boost/filesystem.hpp>
#include <fstream>

#define testSerialPort 0
namespace ly
{
    class armorDetector
    {
    public:
        armorDetector();
        ~armorDetector();

    private:
        config *config_;
        armorThread *armor_;
        cameraThread *camera_;
        lightBarThread *light_;
        preProcessThread *preProcess_;
        serialPortReadThread *serialPortRead_;
        serialPortWriteThread *serialPortWrite_;
    };
}



#endif //__ARMOR_DETECTOR__
