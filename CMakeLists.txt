cmake_minimum_required(VERSION 3.5)
project(armorDetector)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_STANDARD 11)
#set(TENSORFLOW_DIR /home/narrow/tensorflow)

add_subdirectory(./armor)
add_subdirectory(./tool)
add_subdirectory(./driver)
add_subdirectory(./solvePNP)

find_package(OpenCV  REQUIRED)

include_directories(
        tool/inc
        armor/inc
        driver/camera/inc
        driver/serialPort/inc
        driver/camera/lib/hik_include
		driver/camera/lib/dahen_include
        solvePNP/inc
#		${PROJECT_SOURCE_DIR}/tool/lib/config/include/
        ${OpenCV_INCLUDE_DIRS}
        /usr/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        thread/inc
#		${TENSORFLOW_DIR}
#		${TENSORFLOW_DIR}/bazel-genfiles
#		${TENSORFLOW_DIR}/bazel-bin/tensorflow
)
#link_directories(${TENSORFLOW_DIR}/bazel-bin/tensorflow)

add_executable(armorDetector
        main.cpp
        armorDetector.cpp
        )
add_library(thread
        thread/src/serialPortReadThread.cpp
		thread/src/serialPortWriteThread.cpp
        driver/serialPort/src/serialPort.cpp
        driver/camera/src/HIK_camera.cpp
		driver/camera/src/DAHEN_camera.cpp
		driver/camera/src/GxCamera.cpp
        thread/src/cameraThread.cpp
        thread/src/preProcessThread.cpp
        thread/src/lightBarThread.cpp
        armor/src/lightBar.cpp
        armor/src/armor.cpp
        armor/src/RM_BUFF.cpp
        armor/src/buff.cpp
        thread/src/armorThread.cpp
        thread/src/thread.cpp
        solvePNP/src/solvePNP.cpp
		solvePNP/src/transform.cpp
        )

target_link_libraries(
        armorDetector
        tools
        camera
        armor
        solvePNP
        thread
		${catkin_LIBRARIES}
		jsoncpp
#		${TENSORFLOW_DIR}/bazel-bin/tensorflow/libtensorflow_cc.so
		${OpenCV_LIBS}
		-lopencv_video
		-lboost_system
		-lboost_filesystem
		-lpthread
        )

