find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        inc
        ../../tool/inc
        ${PROJECT_SOURCE_DIR}/driver/camera/lib/hik_include
		${PROJECT_SOURCE_DIR}/driver/camera/lib/dahen_include
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}

)

add_library(
        camera
        src/camera.cpp
        src/HIK_camera.cpp
		src/DAHEN_camera.cpp
		src/GxCamera.cpp)
target_link_libraries(
        camera
        ${OpenCV_LIBS}
#		${PROJECT_SOURCE_DIR}/driver/camera/lib/dahen_lib/libgxiapi.so
		/usr/lib/libgxiapi.so
#		${PROJECT_SOURCE_DIR}/driver/camera/lib/hik_lib/pc_64/libMvCameraControl.so
        -lpthread
)
