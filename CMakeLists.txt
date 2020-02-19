cmake_minimum_required(VERSION 3.15)
project(ekf)

set(CMAKE_CXX_STANDARD 14)

# 添加Eigen头文件
#include_directories( ${PROJECT_SOURCE_DIR}/ThirdPartyLib/Eigen)
include_directories("D:\\eign\\eigen\\Eigen")

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        )
MESSAGE(STATUS "This is SOURCE dir "${PROJECT_SOURCE_DIR})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/GetData.cpp
        src/ExtendedKalmanFilter.cpp
        src/SensorFusion.cpp
        include/DataPackage.h
        )

#add_executable(ekf test_main/main.cpp include/GetData.h src/GetData.cpp include/DataPackage.h include/ExtendedKalmanFilter.h src/ExtendedKalmanFilter.cpp src/SensorFusion.cpp include/SensorFusion.h)

add_executable(ekf1 test_main/main.cpp)
target_link_libraries(ekf1 ${PROJECT_NAME})
