#include "thread.h"
#include <unistd.h>
namespace ly{
    std::mutex thread::mutex_serial_receive_;
    std::mutex thread::mutex_serial_send_;
    std::mutex thread::mutex_pic_;
    std::mutex thread::mutex_pic2_;
    std::mutex thread::mutex_ROI_;
    std::mutex thread::mutex_lightBar_;
    void thread::start(int delay)
    {
        delay_ = delay;
        pthread_create(&thread_, NULL, pthread_fun, (void *) this);
    }
    void * thread::pthread_fun(void *__this)
    {
        auto * _this =(thread *)__this;
        while (true)
        {
            try
            {
                _this->process();
            }
            catch (...)
            {
                break;
            }
            usleep(_this->delay_);
        }
    }
}
