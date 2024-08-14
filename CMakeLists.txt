cmake_minimum_required(VERSION 3.5)
project(Kalman_Filter)

find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(
    include    
    ${EIGEN3_INCLUDE_DIR})

add_executable(kf src/lidar_kf.cpp )
# target_link_libraries(kf)

set(CMAKE_INSTALL_PREFIX "../install")
install(
    TARGETS
    kf
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lidar_kf
)