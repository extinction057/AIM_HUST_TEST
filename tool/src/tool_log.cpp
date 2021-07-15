/**
 * @file log.cpp
 * @brief create log file,use google log system glog,
 *        install glog by command ./autogen.sh && ./configure && make && make install
 * @author lyc
 * @typical usage
 {
    ly::log test(argv[0]);
    char str[20] = "hello log!";
    LOG(INFO) << str;
    LOG(INFO) << "hello google!";
    LOG(INFO) << "info test" << "hello log!";  //输出一个Info日志
    LOG(WARNING) << "warning test";  //输出一个Warning日志
    LOG(ERROR) << "error test";  //输出一个Error日志
    DLOG(INFO) << "info test" << "hello log!";  //debug  输出一个Info日志
    DLOG(WARNING) << "warning test";  //debug 输出一个Warning日志
    DLOG(ERROR) << "error test";  //debug 输出一个Error日志
 }
 */
#include "tool_log.h"
namespace ly{

    log::log(const char* argv0)
    {
        FLAGS_log_dir = "../tool/log_files";
        google::InitGoogleLogging(argv0);
        google::SetLogDestination(google::GLOG_INFO, "../tool/log_files/INFO_");
        google::SetLogDestination(google::GLOG_WARNING, "../tool/log_files/WARNING_");
        google::SetLogDestination(google::GLOG_ERROR, "../tool/log_files/ERROR_");
        google::SetStderrLogging(google::GLOG_INFO);
        google::SetLogFilenameExtension("log_");
        FLAGS_colorlogtostderr = true;  // Set log color
        FLAGS_logbufsecs = 0;  // Set log output speed(s)
        FLAGS_max_log_size = 100;  //unit M, Set max log file size
        FLAGS_stop_logging_if_full_disk = true;  // If disk is full
    }
    log::~log()
    {
        google::ShutdownGoogleLogging();
    }
}