find_package( OpenCV REQUIRED)
find_package( Sophus REQUIRED )
find_package( Pangolin )
find_package( Threads REQUIRED)

#set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)

include_directories(
        inc
        ../tool/inc
        ../armor/inc
		../thread/inc/
		../driver/serialPort/inc
        /usr/include/eigen3
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
		${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
		/usr/include/math
)
add_library(
        solvePNP
        src/solvePNP.cpp

)
add_library(
		trans
		src/transform.cpp
)
target_link_libraries(
        solvePNP
		trans
        ${OpenCV_LIBS}
		${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES}
		-lpthread
        -lboost_system
		-lopencv_video
		-lopencv_imgproc
		-lopencv_core
)
