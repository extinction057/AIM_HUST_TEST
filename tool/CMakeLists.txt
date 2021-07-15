include_directories(
        inc
        /usr/local/include/glog
        /lib/config/include/json
)

add_library(
        tools
        src/tool_config.cpp
        src/tool_log.cpp
        src/tool_time.cpp
        inc/tool_cvex.h src/tool_cvex.cpp)
target_link_libraries(
        tools
        /usr/local/lib/libglog.a
        /usr/local/lib/libjson-c.a
)
