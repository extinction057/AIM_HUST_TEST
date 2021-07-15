find_package(OpenCV REQUIRED)

include_directories(
        inc
        ${OpenCV_INCLUDE_DIRS}
        ../tool/inc
        ../thread/inc
        /usr/include/eigen3
		../solvePNP/inc
		../driver/camera/inc
		../driver/serialPort/inc
)

add_library(
        armor
        src/armor.cpp
        src/lightBar.cpp
        src/score.cpp
		src/RM_BUFF.cpp
		src/buff.cpp
)
target_link_libraries(
        armor
        tools
        ${OpenCV_LIBS}
        -lopencv_video
)
