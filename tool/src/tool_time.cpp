/**
 * @file time.cpp
 * @brief get running time of the code
 * @author lyc
 * @typical usage
 {
    ly::time test;
    test.countBegin();
    for(int i = 0;i<1000;i++)
    {
        for(int j = 0;j<1000;j++)
        {
            for(int k = 0;k<1000;k++)
            {
                double x = 3;
            }
        }
    }
    test.countEnd();
    std::cout<<"time ms"<<test.getTimeMs()<<std::endl;
 }
 */
#include "tool_time.h"
#include <iostream>
namespace ly{
    void time::countBegin()
    {
        gettimeofday(&start, NULL);
        startFlag = 1;
    }
    double time::countEnd()
    {
        gettimeofday(&end, NULL);
        if(startFlag == 1)
        {
            countMs = (double) (1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)) / 1000;
            startFlag = 0;
            return countMs;
        }
        else
        {
            std::cerr<<"try to count time without begin !!!!!"<<std::endl;
            return -1;
        }
    }
    double time::getTimeMs(void)
    {
        return countMs;
    }
}

