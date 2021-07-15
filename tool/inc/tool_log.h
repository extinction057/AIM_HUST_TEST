#ifndef __TOOL_LOG_H
#define __TOOL_LOG_H

#include "glog/logging.h"

namespace ly{
    class log
    {
    public:
        log(const char* argv0);
        ~log();
    };
}

#endif //__TOOL_LOG_H