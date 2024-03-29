#ifndef __THREAD_H
#define __THREAD_H

#include <mutex>
#include <thread>

namespace ly
{
    // 多线程类，纯虚类
    class thread
    {
    public:
        static std::mutex mutex_serial_receive_;
        static std::mutex mutex_serial_send_;
        static std::mutex mutex_pic_;
        static std::mutex mutex_pic2_;
        static std::mutex mutex_ROI_;
        static std::mutex mutex_lightBar_;
        static std::mutex mutex_armor_;
        static std::mutex mutex_write_;

        thread() = default;
        ~thread() = default;
        void start(int delay);
        static void* pthread_fun(void* __this);
        int delay_{};// initialize as zero
        virtual void process() = 0;
        pthread_t thread_{};
    };
}
#endif //__THREAD_H
