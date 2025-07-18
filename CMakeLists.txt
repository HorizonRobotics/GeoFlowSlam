cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)


#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Debug)
#ENDIF()
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
option(ENABLE_VIEWER "enable render viewer" ON)
MESSAGE("ENABLE_VIEWER: ${ENABLE_VIEWER}")
# In RGB-D mode, set to off.There is some bug
option(ENABLE_ASYNC "enable async" ON)
MESSAGE("ENBLE_ASYNC: ${ENBLE_ASYNC}")
option(ENABLE_OMP "enable omp" ON)
MESSAGE("ENBLE_OMP: ${ENBLE_OMP}")
option(REGISTER_TIMES "register times" ON)
MESSAGE("REGISTER_TIMES: ${REGISTER_TIMES}")

option(ENABLE_ROS2 "enable ros2" ON)
MESSAGE("ENBLE_ROS2: ${ENBLE_ROS2}")
if (ENABLE_VIEWER)
    add_definitions(-DENABLE_VIEWER)
    find_package(Pangolin REQUIRED)
endif()
if (ENABLE_ASYNC)
    add_definitions(-DENABLE_ASYNC)
endif()
if (ENABLE_OMP)
    add_definitions(-DENABLE_OMP)
endif()
if(REGISTER_TIMES)
    add_definitions(-DREGISTER_TIMES)
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ")
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    message("-- Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(realsense2)
find_package(GLEW REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
# find_package(PCL 1.10 REQUIRED)
   if(PCL_FOUND)
      message(STATUS "PCL found.")
   else()
      message(FATAL_ERROR "PCL not found.")
   endif()
# You can also let it find the package by itself
# file(GLOB PCL_LIBRARIES /usr/lib/x86_64-linux-gnu/libpcl_*)
# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10)  #指定pcl1.8路径
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/util
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${PROJECT_SOURCE_DIR}/Thirdparty/small_gicp/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${GLEW_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
# ${GLOG_INCLUDE_DIRS}
)

include_directories(${GLEW_INCLUDE_DIRS})
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
src/ImuInitializer.cc
src/polynomial.cc
src/ThreadPool.cpp
src/RegistrationGICP.cc
src/LidarMapping.cc
src/Lidar.cc 
src/LidarProcess.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h
include/ImuInitializer.h
include/util/eigen_defs.h 
include/util/svd.h 
include/imu_ceres.h 
include/methods.h 
include/polynomial.h
include/RegistrationGICP.h
include/LidarMapping.h
include/Lidar.h
include/LidarProcess.h
)

add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/small_gicp)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${CERES_LIBRARIES}
${Boost_LIBRARIES}
${GLEW_LIBRARIES}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/Thirdparty/small_gicp/lib/libsmall_gicp.so
${PCL_LIBRARIES}
# /usr/local/opencv_4.2/lib/libopencv_core.so.4.2
# /usr/local/opencv_4.2/lib/libopencv_imgproc.so.4.2
-lboost_serialization
-lcrypto
)

## If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
   include_directories(${PROJECT_NAME}
   ${realsense_INCLUDE_DIR}
   )
   target_link_libraries(${PROJECT_NAME}
   ${realsense2_LIBRARY}
   )
endif()


# Build examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})
add_executable(rgbd_l515
        Examples/RGB-D/rgbd_l515.cc)
target_link_libraries(rgbd_l515 ${PROJECT_NAME})
#
# if(realsense2_FOUND)
#    add_executable(rgbd_realsense_D435i
#            Examples/RGB-D/rgbd_realsense_D435i.cc)
#    target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
# endif()


## RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)
#
# if(realsense2_FOUND)
#    add_executable(rgbd_inertial_realsense_D435i
#            Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
#    target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
# endif()
# rgbd_inertial.cc
add_executable(rgbd_inertial
        Examples/RGB-D-Inertial/rgbd_inertial.cc)
target_link_libraries(rgbd_inertial ${PROJECT_NAME})
#Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti
#         Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})
#
#add_executable(stereo_euroc
#        Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})
#
#add_executable(stereo_tum_vi
#        Examples/Stereo/stereo_tum_vi.cc)
#target_link_libraries(stereo_tum_vi ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(stereo_realsense_t265
#            Examples/Stereo/stereo_realsense_t265.cc)
#    target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})
#
#    add_executable(stereo_realsense_D435i
#            Examples/Stereo/stereo_realsense_D435i.cc)
#    target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
#endif()

#Monocular examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum
#         Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})
# add_executable(mono_l515
#         Examples/Monocular/mono_l515.cc)
# target_link_libraries(mono_l515 ${PROJECT_NAME})
#add_executable(mono_kitti
#        Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})
#
# add_executable(mono_euroc
#        Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})
#
#add_executable(mono_tum_vi
#        Examples/Monocular/mono_tum_vi.cc)
#target_link_libraries(mono_tum_vi ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(mono_realsense_t265
#            Examples/Monocular/mono_realsense_t265.cc)
#    target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})
#
#    add_executable(mono_realsense_D435i
#            Examples/Monocular/mono_realsense_D435i.cc)
#    target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
#endif()

##Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)
#
add_executable(mono_inertial
       Examples/Monocular-Inertial/mono_inertial_l515.cc)
target_link_libraries(mono_inertial ${PROJECT_NAME})
#
#add_executable(mono_inertial_tum_vi
#        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
#target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})
add_executable(mono_inertial_euroc
       Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

# mono_inertial_gopro.cc
add_executable(mono_inertial_gopro
       Examples/Monocular-Inertial/mono_inertial_gopro.cc)
target_link_libraries(mono_inertial_gopro ${PROJECT_NAME})

#
#if(realsense2_FOUND)
#    add_executable(mono_inertial_realsense_t265
#            Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
#    target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})
#
#    add_executable(mono_inertial_realsense_D435i
#            Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
#    target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
#endif()

##Stereo Inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)
#
#add_executable(stereo_inertial_euroc
#        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
#target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})
#
# add_executable(stereo_inertial_tum_vi
#        Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
# target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(stereo_inertial_realsense_t265
#            Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
#    target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})
#
#    add_executable(stereo_inertial_realsense_D435i
#            Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
#    target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
#endif()
#
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
#if(realsense2_FOUND)
#    add_executable(recorder_realsense_D435i
#            Examples/Calibration/recorder_realsense_D435i.cc)
#    target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})
#
#    add_executable(recorder_realsense_T265
#            Examples/Calibration/recorder_realsense_T265.cc)
#    target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
#endif()

###Old examples
##
## RGB-D examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D)
#
#add_executable(rgbd_tum_old
#        Examples_old/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum_old ${PROJECT_NAME})

#if(realsense2_FOUND)
#    add_executable(rgbd_realsense_D435i_old
#            Examples_old/RGB-D/rgbd_realsense_D435i.cc)
#    target_link_libraries(rgbd_realsense_D435i_old ${PROJECT_NAME})
#endif()


## RGB-D inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D-Inertial)
#
#if(realsense2_FOUND)
#    add_executable(rgbd_inertial_realsense_D435i_old
#            Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
#    target_link_libraries(rgbd_inertial_realsense_D435i_old ${PROJECT_NAME})
#endif()

##Stereo examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo)
#
#add_executable(stereo_kitti_old
#        Examples_old/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti_old ${PROJECT_NAME})

#add_executable(stereo_euroc_old
#        Examples_old/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc_old ${PROJECT_NAME})
#
#add_executable(stereo_tum_vi_old
#        Examples_old/Stereo/stereo_tum_vi.cc)
#target_link_libraries(stereo_tum_vi_old ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(stereo_realsense_t265_old
#            Examples_old/Stereo/stereo_realsense_t265.cc)
#    target_link_libraries(stereo_realsense_t265_old ${PROJECT_NAME})
#
#    add_executable(stereo_realsense_D435i_old
#            Examples_old/Stereo/stereo_realsense_D435i.cc)
#    target_link_libraries(stereo_realsense_D435i_old ${PROJECT_NAME})
#endif()

##Monocular examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular)
#
#add_executable(mono_tum_old
#        Examples_old/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum_old ${PROJECT_NAME})
#
#add_executable(mono_kitti_old
#        Examples_old/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti_old ${PROJECT_NAME})
#
#add_executable(mono_euroc_old
#        Examples_old/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc_old ${PROJECT_NAME})
#
#add_executable(mono_tum_vi_old
#        Examples_old/Monocular/mono_tum_vi.cc)
#target_link_libraries(mono_tum_vi_old ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(mono_realsense_t265_old
#            Examples_old/Monocular/mono_realsense_t265.cc)
#    target_link_libraries(mono_realsense_t265_old ${PROJECT_NAME})
#
#    add_executable(mono_realsense_D435i_old
#            Examples_old/Monocular/mono_realsense_D435i.cc)
#    target_link_libraries(mono_realsense_D435i_old ${PROJECT_NAME})
#endif()

##Monocular inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular-Inertial)
#
#add_executable(mono_inertial_euroc_old
#        Examples_old/Monocular-Inertial/mono_inertial_euroc.cc)
#target_link_libraries(mono_inertial_euroc_old ${PROJECT_NAME})
#
#add_executable(mono_inertial_tum_vi_old
#        Examples_old/Monocular-Inertial/mono_inertial_tum_vi.cc)
#target_link_libraries(mono_inertial_tum_vi_old ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(mono_inertial_realsense_t265_old
#            Examples_old/Monocular-Inertial/mono_inertial_realsense_t265.cc)
#    target_link_libraries(mono_inertial_realsense_t265_old ${PROJECT_NAME})
#
#    add_executable(mono_inertial_realsense_D435i_old
#            Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
#    target_link_libraries(mono_inertial_realsense_D435i_old ${PROJECT_NAME})
#endif()

##Stereo Inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo-Inertial)
#
#add_executable(stereo_inertial_euroc_old
#        Examples_old/Stereo-Inertial/stereo_inertial_euroc.cc)
#target_link_libraries(stereo_inertial_euroc_old ${PROJECT_NAME})
#
#add_executable(stereo_inertial_tum_vi_old
#        Examples_old/Stereo-Inertial/stereo_inertial_tum_vi.cc)
#target_link_libraries(stereo_inertial_tum_vi_old ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(stereo_inertial_realsense_t265_old
#            Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
#    target_link_libraries(stereo_inertial_realsense_t265_old ${PROJECT_NAME})
#
#    add_executable(stereo_inertial_realsense_D435i_old
#            Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
#    target_link_libraries(stereo_inertial_realsense_D435i_old ${PROJECT_NAME})
#endif()

