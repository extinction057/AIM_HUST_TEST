#ifndef __TOOL_TIME_H
#define __TOOL_TIME_H

#include "sys/time.h"

namespace ly{
    class time
    {
    private:
        double countMs;//ms
        struct timeval start, end;
        int startFlag = 0;
    public:
        void countBegin();
        double countEnd();
        double getTimeMs(void);
    };
}

#endif //__TOOL_TIME_H