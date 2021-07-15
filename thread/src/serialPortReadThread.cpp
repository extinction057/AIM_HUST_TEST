#include "serialPortReadThread.h"
#include <iostream>
#include <utility>
#include "tool_log.h"
namespace ly{
    std::vector<receiveData> receive_;
    /**
     * @brief 构造函数
     * @param portName 串口名称
     */
    serialPortReadThread::serialPortReadThread(const serialPort_dev& config)
    {
        if(config.enable)
        {
            init(config.deviceName);
            start(100);
        }
        receive_.resize(150);
    }
    serialPortReadThread::~serialPortReadThread()
    {
        //delete read_data_;
    }
    /**
     * @brief 串口线程函数
     */
    void serialPortReadThread::process()
    {
        readData();
//        counter_.countEnd();
//        std::cout<<counter_.getTimeMs()<<std::endl;
//        counter_.countBegin();
//        time_t end = clock();
//        std::cout<<"timeDATA: "<<end<<std::endl;
    }
    /**
     * @brief 读数据
     */
    void serialPortReadThread::readData()
    {
        short flag, counter, Gyro_Pitch, Gyro_Yaw;
        SP_Read(read_data_,max_receive_len_);
        //@TODO 读取数据
        memcpy(temptemp+10,read_data_,10);

        for(int start_bit=0;start_bit<10;start_bit++)
        {
            if(temptemp[start_bit]=='!')
            {
                for(int j=0;j<10;j++)
                {
                    temp[j]=temptemp[start_bit+j];
                }
                //@TODO CRC校验
                if(Verify_CRC8_Check_Sum(temp,10))//CRC校验
                {
                    stm32.flag = temp[1];
                    stm32.pitch = (temp[2]<<8)|temp[3];
                    stm32.yaw = (((int)temp[4])<<24)|(((int)temp[5])<<16)|(((int)temp[6])<<8)|(temp[7]);
                    stm32.Data_Time = clock();
//                    std::cout<<"-----------stm32.pitch:"<<stm32.pitch<<std::endl;
//                    std::cout<<"stm32.yaw:"<<stm32.yaw<<std::endl;

                }
                break;
            }
        }

        memcpy(temptemp,temptemp+10,10);
        //互斥锁
        mutex_serial_receive_.lock();
        if(receive_.size()>149)
        {
            receive_.erase(receive_.begin());
        }
        receive_.emplace_back(stm32);

        mutex_serial_receive_.unlock();
    }
    /**
     * @brief 获取收到的数据
     * @return 当前数据
     */
    std::vector<receiveData> serialPortReadThread::getReceiveMsg()
    {
        mutex_serial_receive_.lock();
        auto data = receive_;
        mutex_serial_receive_.unlock();
        return data;
    }
}

