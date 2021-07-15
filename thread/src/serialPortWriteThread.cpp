#include "serialPortWriteThread.h"
#include <iostream>
#include <utility>
#include "tool_log.h"

namespace ly{
    /**
     * @brief 构造函数
     * @param portName 串口名称
     */
    serialPortWriteThread::serialPortWriteThread(const serialPort_dev& config)
    {
        if(config.enable)
        {
            init(config.deviceName);
            start(10000);
        }
    }
    serialPortWriteThread::~serialPortWriteThread()
    {
    }
    /**
     * @brief 串口线程函数
     */
    void serialPortWriteThread::process()
    {
        writeData();
    }
    /**
     * @brief 写数据
     */
    void serialPortWriteThread::writeData()
    {
	    unsigned char msg[max_receive_len_];

        mutex_serial_send_.lock();
        auto temp = send_;
        mutex_serial_send_.unlock();

        //@TODO 添加CRC
	    msg[0] = '!';
        unsigned char *tmp = (unsigned char*)(&temp.shootStatus);
	    msg[1] = tmp[0];
//        std::cout<<"shoot:"<<(int)tmp[0]<<std::endl;
	    tmp = (unsigned char*)(&temp.pitch);
//	    std::cout<<"send pitch:"<<temp.pitch<<std::endl;
	    msg[2]=tmp[1];
	    msg[3]=tmp[0];

	    tmp = (unsigned char*)(&temp.yaw);
//	    std::cout<<"send yaw:"<<temp.yaw<<"\n";
	    msg[4]=tmp[3];
	    msg[5]=tmp[2];
        msg[6]=tmp[1];
        msg[7]=tmp[0];

	    msg[8]=temp.distance;

	    msg[9] = '#';
	    Append_CRC8_Check_Sum(msg, max_receive_len_);

        //@TODO 发送数据
	    SP_Write(msg, max_receive_len_);
    }
    /**
     * @brief 设置需要发送的数据
     * @param data 发送数据
     */
    void serialPortWriteThread::setSendMsg(sendData data)
    {
        mutex_serial_send_.lock();
        send_ = data;
        mutex_serial_send_.unlock();
    }
}
