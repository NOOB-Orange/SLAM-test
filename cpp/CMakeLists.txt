cmake_minimum_required(VERSION 3.16)
project(slam_video_runner LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT ORB_SLAM3_ROOT)
  message(FATAL_ERROR "Set ORB_SLAM3_ROOT to the patched official source")
endif()
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)
add_executable(mono_video mono_video.cc)
target_include_directories(mono_video PRIVATE
  "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(mono_video PRIVATE
  "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so"
  ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Eigen3::Eigen Threads::Threads)
set_target_properties(mono_video PROPERTIES BUILD_RPATH
  "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")
