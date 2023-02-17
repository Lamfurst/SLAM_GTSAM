cmake_minimum_required( VERSION 2.8 )

project( GTSAMTest )

set(CMAKE_CXX_STANDARD 11)


add_executable( GTSAM_SLAM src/GTSAM_SLAM.cpp src/io.cpp)


find_package( GTSAM REQUIRED )
include_directories(${GTSAM_INCLUDE_DIR})
target_link_libraries( GTSAM_SLAM gtsam)
