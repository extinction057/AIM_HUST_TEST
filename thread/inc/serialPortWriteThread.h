#ifndef __SERIAL_WRITE_PORT_THREAD_H
#define __SERIAL_WRITE_PORT_THREAD_H

#include "serialPort.h"
#include <mutex>
#include <thread>
#include "tools.h"
#include "thread.h"

namespace ly
{
    // 串口设备的发送接口类
    class serialPortWriteThread:public serialPort,public thread
    {
    public:
        serialPortWriteThread() = default;
        explicit serialPortWriteThread(const serialPort_dev& config);
        ~serialPortWriteThread();
        void setSendMsg(sendData data);
    private:
        int max_receive_len_ = 10;
        sendData send_{};
        void process() override;
        void writeData();
    };
}
#endif //__SERIAL_PORT_THREAD_H
